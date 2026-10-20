/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023-2026 Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * CPU based debayering class
 */

#include "debayer_cpu.h"

#include <algorithm>
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif
#include <stdlib.h>
#include <optional>
#include <unistd.h>

#include "raw64/raw-abi-transport.h"
#include <sys/ioctl.h>
#include <utility>

#include <linux/dma-buf.h>
#include <libyuv/convert_from_argb.h>
#include <libyuv/scale_uv.h>
#include <libyuv/scale.h>
#include <libyuv/cpu_id.h>
#include <libyuv/scale_row.h>
#include <libyuv/planar_functions.h>

#include <libcamera/base/thread.h>
#include <libcamera/base/utils.h>

#include <libcamera/formats.h>
#include <libcamera/control_ids.h>
#include <libcamera/request.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera_manager.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/global_configuration.h"
#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

// shortcut: saturation is uncalibrated; replace after sensor CCM measurements.
static constexpr int8_t colourMatrix[] = {
    74, -7, -3, 0, -1, 69, -4, 0,
    -1, -7, 72, 0, 0, 0, 0, 64,
};

static_assert(colourMatrix[3] == 0 && colourMatrix[7] == 0 && colourMatrix[11] == 0);
static_assert(colourMatrix[12] == 0 && colourMatrix[13] == 0 && colourMatrix[14] == 0 && colourMatrix[15] == 64);

// The fixed matrix must not overflow the int16 arithmetic below.
static_assert([] {
    for (unsigned int row = 0; row < 3; ++row) {
        unsigned int bound = 0;
        for (unsigned int col = 0; col < 3; ++col) {
            const int coefficient = colourMatrix[row * 4 + col];
            bound += coefficient < 0 ? -coefficient : coefficient;
        }
        if (bound > 128) return false;
    }
    return true;
}());

#if defined(__ARM_NEON)
static uint8x8x4_t correctColour8(uint8x8x4_t p)
{
    const int16x8_t b = vreinterpretq_s16_u16(vmovl_u8(p.val[0]));
    const int16x8_t g = vreinterpretq_s16_u16(vmovl_u8(p.val[1]));
    const int16x8_t r = vreinterpretq_s16_u16(vmovl_u8(p.val[2]));
    p.val[0] = vqshrun_n_s16(vmlaq_n_s16(vmlaq_n_s16(vmulq_n_s16(b, colourMatrix[0]), g, colourMatrix[1]), r, colourMatrix[2]), 6);
    p.val[1] = vqshrun_n_s16(vmlaq_n_s16(vmlaq_n_s16(vmulq_n_s16(b, colourMatrix[4]), g, colourMatrix[5]), r, colourMatrix[6]), 6);
    p.val[2] = vqshrun_n_s16(vmlaq_n_s16(vmlaq_n_s16(vmulq_n_s16(b, colourMatrix[8]), g, colourMatrix[9]), r, colourMatrix[10]), 6);
    return p;
}

static uint8x8_t colourLuma8(const uint8x8x4_t &p)
{
    uint16x8_t y = vmull_u8(p.val[0], vdup_n_u8(25));
    y = vmlal_u8(y, p.val[1], vdup_n_u8(129));
    y = vmlal_u8(y, p.val[2], vdup_n_u8(66));
    return vshrn_n_u16(vaddq_u16(y, vdupq_n_u16(0x1080)), 8);
}

// The caller overwrites RGB or reuses it as scratch after conversion.
static int colourToNv12Fused(const uint8_t *rgb, int rgbStride, uint8_t *y, int yStride,
                            uint8_t *uv, int uvStride, int width, int height)
{
    for (int row = 0; row < height; row += 2) {
        const uint8_t *first = rgb + size_t(row) * rgbStride;
        const bool secondRow = row + 1 < height;
        const uint8_t *second = first + (secondRow ? rgbStride : 0);
        for (int x = 0; x < width; x += 8) {
            const auto top = correctColour8(vld4_u8(first + x * 4));
            const auto bottom = secondRow ? correctColour8(vld4_u8(second + x * 4)) : top;
            vst1_u8(y + size_t(row) * yStride + x, colourLuma8(top));
            if (secondRow) {
                vst1_u8(y + size_t(row + 1) * yStride + x, colourLuma8(bottom));
            }
            const auto average = [](uint8x8_t a, uint8x8_t b) {
                return vreinterpret_s16_u16(vrshr_n_u16(vadd_u16(vpaddl_u8(a), vpaddl_u8(b)), 2));
            };
            const int16x4_t b = average(top.val[0], bottom.val[0]);
            const int16x4_t g = average(top.val[1], bottom.val[1]);
            const int16x4_t r = average(top.val[2], bottom.val[2]);
            const int16x4_t u = vmls_n_s16(vmls_n_s16(vmul_n_s16(b, 112), g, 74), r, 38);
            const int16x4_t v = vmls_n_s16(vmls_n_s16(vmul_n_s16(r, 112), g, 94), b, 18);
            const uint8x8_t planar = vshrn_n_u16(vaddq_u16(
                vreinterpretq_u16_s16(vcombine_s16(u, v)), vdupq_n_u16(0x8000)), 8);
            const uint8x8_t interleaved = vzip_u8(planar, vext_u8(planar, planar, 4)).val[0];
            vst1_u8(uv + size_t(row / 2) * uvStride + x, interleaved);
        }
    }
    return 0;
}
#endif

static int colourToNv12(uint8_t *rgb, int rgbStride, uint8_t *y, int yStride,
			uint8_t *uv, int uvStride, int width, int height, bool colourBoost)
{
	if (!rgb || !y || !uv || width <= 0 || height <= 0)
		return -EINVAL;
	if (!colourBoost)
		return libyuv::ARGBToNV12(rgb, rgbStride, y, yStride, uv, uvStride, width, height);

#if defined(__ARM_NEON)
    if (!(width & 7) && rgbStride / 4 >= width && yStride >= width && uvStride >= width)
        return colourToNv12Fused(rgb, rgbStride, y, yStride, uv, uvStride, width, height);
#endif

	// Keep corrected RGB rows hot instead of rereading a full intermediate image.
	for (int row = 0; row < height; row += 2) {
		uint8_t *source = rgb + size_t(row) * rgbStride;
		const int rows = std::min(2, height - row);
		int ret = libyuv::ARGBColorMatrix(source, rgbStride, source, rgbStride,
						colourMatrix, width, rows);
		if (ret)
			return ret;
		ret = libyuv::ARGBToNV12(source, rgbStride, y + size_t(row) * yStride, yStride,
					uv + size_t(row / 2) * uvStride, uvStride, width, rows);
		if (ret)
			return ret;
	}
	return 0;
}

static void smoothLumaRow(uint8_t *destination, const uint8_t *original, const uint8_t *smooth, size_t width)
{
	size_t col = 0;
#if defined(__ARM_NEON)
	for (; col + 16 <= width; col += 16) {
		const uint8x16_t source = vld1q_u8(original + col);
		const uint8x16_t filtered = vld1q_u8(smooth + col);
		const uint8x16_t mask = vcleq_u8(vabdq_u8(source, filtered), vdupq_n_u8(8));
		vst1q_u8(destination + col, vbslq_u8(mask, filtered, source));
	}
#endif
	for (; col < width; ++col)
		destination[col] = std::abs(int(original[col]) - smooth[col]) <= 8 ? smooth[col] : original[col];
}

static size_t smoothLumaScratchBytes(unsigned int width, unsigned int height, unsigned int stride)
{
    return size_t(stride) * ((width > 2 && height > 1) ? 2 : height);
}

static int smoothLumaPlane(uint8_t *destination, const uint8_t *original,
                            const uint8_t *small, uint8_t *rows,
                            unsigned int width, unsigned int height, unsigned int stride)
{
    ASSERT(width && height);
    auto scaleRows = libyuv::ScaleRowUp2_Bilinear_Any_C;
#ifdef HAS_SCALEROWUP2_BILINEAR_NEON
    if (libyuv::TestCpuFlag(libyuv::kCpuHasNEON))
        scaleRows = libyuv::ScaleRowUp2_Bilinear_Any_NEON;
#endif
    const unsigned int halfWidth = (width + 1) / 2;
    const unsigned int halfHeight = (height + 1) / 2;
    auto blend = [&](unsigned int row, const uint8_t *filtered) {
        const size_t offset = size_t(row) * stride;
        smoothLumaRow(destination + offset, original + offset, filtered, width);
        if (destination != original && stride > width)
            memcpy(destination + offset + width, original + offset + width, stride - width);
    };
    if (width <= 2 || height == 1) {
        const int ret = libyuv::ScalePlane(small, halfWidth, halfWidth, halfHeight,
                                          rows, stride, width, height, libyuv::kFilterBilinear);
        if (ret)
            return ret;
        for (unsigned int row = 0; row < height; ++row)
            blend(row, rows + size_t(row) * stride);
        return 0;
    }
    scaleRows(small, 0, rows, 0, width);
    blend(0, rows);
    for (unsigned int row = 0; row + 1 < halfHeight; ++row) {
        scaleRows(small + size_t(row) * halfWidth, halfWidth, rows, stride, width);
        blend(row * 2 + 1, rows);
        blend(row * 2 + 2, rows + stride);
    }
    if (!(height & 1)) {
        scaleRows(small + size_t(halfHeight - 1) * halfWidth, 0, rows, 0, width);
        blend(height - 1, rows);
    }
    return 0;
}



/**
 * \brief Class representing one CPU debayering thread
 *
 * Implementation for CPU based debayering threads.
 */
class DebayerCpuThread : public Thread, public Object
{
public:
	DebayerCpuThread(DebayerCpu *debayer, unsigned int threadIndex,
			 bool enableInputMemcpy);

	void configure(unsigned int yStart, unsigned int yEnd);
	void process(uint32_t frame, const uint8_t *src, uint8_t *dst);

private:
	void setupInputMemcpy(const uint8_t *linePointers[]);
	void shiftLinePointers(const uint8_t *linePointers[], const uint8_t *src);
	void memcpyNextLine(const uint8_t *linePointers[]);
	int process2(uint32_t frame, const uint8_t *src, uint8_t *dst);
	int process4(uint32_t frame, const uint8_t *src, uint8_t *dst);
	int convertRows(unsigned int first, unsigned int count);

	/* Max. supported Bayer pattern height is 4, debayering this requires 5 lines */
	static constexpr unsigned int kMaxLineBuffers = 5;

	DebayerCpu *debayer_;
	unsigned int threadIndex_;
	unsigned int yStart_;
	unsigned int yEnd_;
	unsigned int lineBufferLength_;
	unsigned int lineBufferPadding_;
	unsigned int lineBufferIndex_;
	std::vector<uint8_t> lineBuffers_[kMaxLineBuffers];
	std::vector<uint8_t> rgbRows_;
	bool enableInputMemcpy_;
};

/**
 * \brief Construct a DebayerCpuThread object
 * \param[in] debayer pointer back to the DebayerCpuObject this thread belongs to
 * \param[in] threadIndex 0 .. n thread-index value for the thread
 * \param[in] enableInputMemcpy when set copy input data to a heap buffer before use
 */
DebayerCpuThread::DebayerCpuThread(DebayerCpu *debayer, unsigned int threadIndex,
				   bool enableInputMemcpy)
	: Thread("DebayerCpu:" + std::to_string(threadIndex)),
	  debayer_(debayer), threadIndex_(threadIndex),
	  enableInputMemcpy_(enableInputMemcpy)
{
	moveToThread(this);
}

/**
 * \class DebayerCpu
 * \brief Class for debayering on the CPU
 *
 * Implementation for CPU based debayering
 */

/**
 * \brief Constructs a DebayerCpu object
 * \param[in] stats Pointer to the stats object to use
 * \param[in] cm The camera manager
 */
DebayerCpu::DebayerCpu(std::unique_ptr<SwStatsCpu> stats, const CameraManager &cm)
	: Debayer(cm), stats_(std::move(stats))
{
	/*
	 * Reading from uncached buffers may be very slow.
	 * In such a case, it's better to copy input buffer data to normal memory.
	 * But in case of cached buffers, copying the data is unnecessary overhead.
	 * enable_input_memcpy_ makes this behavior configurable.  At the moment, we
	 * always set it to true as the safer choice but this should be changed in
	 * future.
	 *
	 * \todo Make memcpy automatic based on runtime detection of platform
	 * capabilities.
	 */
	const GlobalConfiguration &configuration = cm._d()->configuration();
	bool enableInputMemcpy =
		configuration.option<bool>({ "software_isp", "copy_input_buffer" }).value_or(true);

	unsigned int threadCount =
		configuration.option<unsigned int>({ "software_isp", "threads" }).value_or(kDefaultThreads);
	threadCount = std::clamp(threadCount, kMinThreads, kMaxThreads);
	threads_.resize(threadCount);
	chromaDenoise_ = configuration.option<bool>({ "software_isp", "chroma_denoise" }).value_or(false);
	lumaDenoise_ = configuration.option<bool>({ "software_isp", "luma_denoise" }).value_or(false);
	colourBoost_ = configuration.option<bool>({ "software_isp", "colour_boost" }).value_or(false);

	for (unsigned int i = 0; i < threads_.size(); i++)
		threads_[i] = std::make_unique<DebayerCpuThread>(this, i, enableInputMemcpy);

	LOG(Debayer, Debug) << "Thread count " << threadCount;
}

DebayerCpu::~DebayerCpu() = default;

#define DECLARE_SRC_POINTERS(pixel_t)                            \
	const pixel_t *prev = (const pixel_t *)src[0] + xShift_; \
	const pixel_t *curr = (const pixel_t *)src[1] + xShift_; \
	const pixel_t *next = (const pixel_t *)src[2] + xShift_;

#define GAMMA(value) \
	*dst++ = gammaLut_[std::clamp(value, 0, static_cast<int>(gammaLut_.size()) - 1)]

#define STORE_PIXEL(b_, g_, r_)                         \
	if constexpr (ccmEnabled) {                     \
		const CcmColumn &blue = blueCcm_[b_];   \
		const CcmColumn &green = greenCcm_[g_]; \
		const CcmColumn &red = redCcm_[r_];     \
		GAMMA(blue.b + green.b + red.b);        \
		GAMMA(blue.g + green.g + red.g);        \
		GAMMA(blue.r + green.r + red.r);        \
	} else {                                        \
		*dst++ = blue_[b_];                     \
		*dst++ = green_[g_];                    \
		*dst++ = red_[r_];                      \
	}                                               \
	if constexpr (addAlphaByte)                     \
		*dst++ = 255;                           \
	x++;

/*
 * RGR
 * GBG
 * RGR
 */
#define BGGR_BGR888(p, n, div)                                                 \
	STORE_PIXEL(                                                           \
		curr[x] / (div),                                               \
		(prev[x] + curr[x - p] + curr[x + n] + next[x]) / (4 * (div)), \
		(prev[x - p] + prev[x + n] + next[x - p] + next[x + n]) / (4 * (div)))

/*
 * GBG
 * RGR
 * GBG
 */
#define GRBG_BGR888(p, n, div)                     \
	STORE_PIXEL(                               \
		(prev[x] + next[x]) / (2 * (div)), \
		curr[x] / (div),                   \
		(curr[x - p] + curr[x + n]) / (2 * (div)))

/*
 * GRG
 * BGB
 * GRG
 */
#define GBRG_BGR888(p, n, div)                             \
	STORE_PIXEL(                                       \
		(curr[x - p] + curr[x + n]) / (2 * (div)), \
		curr[x] / (div),                           \
		(prev[x] + next[x]) / (2 * (div)))

/*
 * BGB
 * GRG
 * BGB
 */
#define RGGB_BGR888(p, n, div)                                                         \
	STORE_PIXEL(                                                                   \
		(prev[x - p] + prev[x + n] + next[x - p] + next[x + n]) / (4 * (div)), \
		(prev[x] + curr[x - p] + curr[x + n] + next[x]) / (4 * (div)),         \
		curr[x] / (div))

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer8_BGBG_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint8_t)

	for (int x = 0; x < (int)window_.width;) {
		BGGR_BGR888(1, 1, 1)
		GBRG_BGR888(1, 1, 1)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer8_GRGR_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint8_t)

	for (int x = 0; x < (int)window_.width;) {
		GRBG_BGR888(1, 1, 1)
		RGGB_BGR888(1, 1, 1)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10_BGBG_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint16_t)

	for (int x = 0; x < (int)window_.width;) {
		/* divide values by 4 for 10 -> 8 bpp value */
		BGGR_BGR888(1, 1, 4)
		GBRG_BGR888(1, 1, 4)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10_GRGR_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint16_t)

	for (int x = 0; x < (int)window_.width;) {
		/* divide values by 4 for 10 -> 8 bpp value */
		GRBG_BGR888(1, 1, 4)
		RGGB_BGR888(1, 1, 4)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer12_BGBG_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint16_t)

	for (int x = 0; x < (int)window_.width;) {
		/* divide values by 16 for 12 -> 8 bpp value */
		BGGR_BGR888(1, 1, 16)
		GBRG_BGR888(1, 1, 16)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer12_GRGR_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint16_t)

	for (int x = 0; x < (int)window_.width;) {
		/* divide values by 16 for 12 -> 8 bpp value */
		GRBG_BGR888(1, 1, 16)
		RGGB_BGR888(1, 1, 16)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10P_BGBG_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 5 / 4;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	/*
	 * For the first pixel getting a pixel from the previous column uses
	 * x - 2 to skip the 5th byte with least-significant bits for 4 pixels.
	 * Same for last pixel (uses x + 2) and looking at the next column.
	 */
	for (int x = 0; x < widthInBytes;) {
		/* First pixel */
		BGGR_BGR888(2, 1, 1)
		/* Second pixel BGGR -> GBRG */
		GBRG_BGR888(1, 1, 1)
		/* Same thing for third and fourth pixels */
		BGGR_BGR888(1, 1, 1)
		GBRG_BGR888(1, 2, 1)
		/* Skip 5th src byte with 4 x 2 least-significant-bits */
		x++;
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10P_GRGR_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 5 / 4;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	for (int x = 0; x < widthInBytes;) {
		/* First pixel */
		GRBG_BGR888(2, 1, 1)
		/* Second pixel GRBG -> RGGB */
		RGGB_BGR888(1, 1, 1)
		/* Same thing for third and fourth pixels */
		GRBG_BGR888(1, 1, 1)
		RGGB_BGR888(1, 2, 1)
		/* Skip 5th src byte with 4 x 2 least-significant-bits */
		x++;
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10P_GBGB_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 5 / 4;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	for (int x = 0; x < widthInBytes;) {
		/* Even pixel */
		GBRG_BGR888(2, 1, 1)
		/* Odd pixel GBGR -> BGGR */
		BGGR_BGR888(1, 1, 1)
		/* Same thing for next 2 pixels */
		GBRG_BGR888(1, 1, 1)
		BGGR_BGR888(1, 2, 1)
		/* Skip 5th src byte with 4 x 2 least-significant-bits */
		x++;
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10P_RGRG_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 5 / 4;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	for (int x = 0; x < widthInBytes;) {
		/* Even pixel */
		RGGB_BGR888(2, 1, 1)
		/* Odd pixel RGGB -> GRBG */
		GRBG_BGR888(1, 1, 1)
		/* Same thing for next 2 pixels */
		RGGB_BGR888(1, 1, 1)
		GRBG_BGR888(1, 2, 1)
		/* Skip 5th src byte with 4 x 2 least-significant-bits */
		x++;
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer12P_BGBG_BGR888(uint8_t *dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 3 / 2;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	for (int x = 0; x < widthInBytes;) {
		/* Even pixel */
		BGGR_BGR888(2, 1, 1)
		/* Odd pixel BGGR -> GBRG */
		GBRG_BGR888(1, 2, 1)
		/* Skip 3rd src byte with 2 x 4 least-significant-bits */
		x++;
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer12P_GRGR_BGR888(uint8_t *dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 3 / 2;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	for (int x = 0; x < widthInBytes;) {
		/* Even pixel */
		GRBG_BGR888(2, 1, 1)
		/* Odd pixel GRBG -> RGGB */
		RGGB_BGR888(1, 2, 1)
		/* Skip 3rd src byte with 2 x 4 least-significant-bits */
		x++;
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer12P_GBGB_BGR888(uint8_t *dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 3 / 2;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	for (int x = 0; x < widthInBytes;) {
		/* Even pixel */
		GBRG_BGR888(2, 1, 1)
		/* Odd pixel GBRG -> BGGR */
		BGGR_BGR888(1, 2, 1)
		/* Skip 3rd src byte with 2 x 4 least-significant-bits */
		x++;
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer12P_RGRG_BGR888(uint8_t *dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 3 / 2;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	for (int x = 0; x < widthInBytes;) {
		/* Even pixel */
		RGGB_BGR888(2, 1, 1)
		/* Odd pixel RGGB -> GRBG */
		GRBG_BGR888(1, 2, 1)
		/* Skip 3rd src byte with 2 x 4 least-significant-bits */
		x++;
	}
}

/*
 * Setup the Debayer object according to the passed in parameters.
 * Return 0 on success, a negative errno value on failure
 * (unsupported parameters).
 */
int DebayerCpu::getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);
	std::vector<PixelFormat> outputFormats = { formats::NV12,
						   formats::RGB888,
						   formats::XRGB8888,
						   formats::ARGB8888,
						   formats::BGR888,
						   formats::XBGR8888,
						   formats::ABGR8888 };

	if ((bayerFormat.bitDepth == 8 || bayerFormat.bitDepth == 10 || bayerFormat.bitDepth == 12) &&
	    bayerFormat.packing == BayerFormat::Packing::None &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = (bayerFormat.bitDepth + 7) & ~7;
		config.patternSize.width = 2;
		config.patternSize.height = 2;
		config.outputFormats = outputFormats;
		return 0;
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2 &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = 10;
		config.patternSize.width = 4; /* 5 bytes per *4* pixels */
		config.patternSize.height = 2;
		config.outputFormats = outputFormats;
		return 0;
	}

	if (bayerFormat.bitDepth == 12 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2 &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = 12;
		config.patternSize.width = 2; /* 3 bytes per *2* pixels */
		config.patternSize.height = 2;
		config.outputFormats = outputFormats;
		return 0;
	}

	LOG(Debayer, Debug)
		<< "Unsupported input format " << inputFormat;
	return -EINVAL;
}

/*
 * Check for standard Bayer orders and set xShift_ and swap debayer0/1, so that
 * a single pair of BGGR debayer functions can be used for all 4 standard orders.
 */
int DebayerCpu::setupStandardBayerOrder(BayerFormat::Order order)
{
	switch (order) {
	case BayerFormat::BGGR:
		break;
	case BayerFormat::GBRG:
		xShift_ = 1; /* BGGR -> GBRG */
		break;
	case BayerFormat::GRBG:
		std::swap(debayer0_, debayer1_); /* BGGR -> GRBG */
		break;
	case BayerFormat::RGGB:
		xShift_ = 1; /* BGGR -> GBRG */
		std::swap(debayer0_, debayer1_); /* GBRG -> RGGB */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define SET_DEBAYER_METHODS(method0, method1)                                                                        \
	debayer0_ = addAlphaByte                                                                                     \
			    ? (ccmEnabled ? &DebayerCpu::method0<true, true> : &DebayerCpu::method0<true, false>)    \
			    : (ccmEnabled ? &DebayerCpu::method0<false, true> : &DebayerCpu::method0<false, false>); \
	debayer1_ = addAlphaByte                                                                                     \
			    ? (ccmEnabled ? &DebayerCpu::method1<true, true> : &DebayerCpu::method1<true, false>)    \
			    : (ccmEnabled ? &DebayerCpu::method1<false, true> : &DebayerCpu::method1<false, false>);

int DebayerCpu::setDebayerFunctions(PixelFormat inputFormat,
				    PixelFormat outputFormat,
				    bool ccmEnabled)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);
	bool addAlphaByte = false;

	xShift_ = 0;
	swapRedBlueGains_ = false;

	auto invalidFmt = []() -> int {
		LOG(Debayer, Error) << "Unsupported input output format combination";
		return -EINVAL;
	};

	switch (outputFormat) {
	case formats::XRGB8888:
	case formats::ARGB8888:
		addAlphaByte = true;
		[[fallthrough]];
	case formats::RGB888:
		break;
	case formats::XBGR8888:
	case formats::ABGR8888:
		addAlphaByte = true;
		[[fallthrough]];
	case formats::BGR888:
		/* Swap R and B in bayer order to generate BGR888 instead of RGB888 */
		swapRedBlueGains_ = true;

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
			bayerFormat.order = BayerFormat::RGGB;
			break;
		case BayerFormat::GBRG:
			bayerFormat.order = BayerFormat::GRBG;
			break;
		case BayerFormat::GRBG:
			bayerFormat.order = BayerFormat::GBRG;
			break;
		case BayerFormat::RGGB:
			bayerFormat.order = BayerFormat::BGGR;
			break;
		default:
			return invalidFmt();
		}
		break;
	default:
		return invalidFmt();
	}

	if ((bayerFormat.bitDepth == 8 || bayerFormat.bitDepth == 10 || bayerFormat.bitDepth == 12) &&
	    bayerFormat.packing == BayerFormat::Packing::None &&
	    isStandardBayerOrder(bayerFormat.order)) {
		switch (bayerFormat.bitDepth) {
		case 8:
			SET_DEBAYER_METHODS(debayer8_BGBG_BGR888, debayer8_GRGR_BGR888)
			break;
		case 10:
			SET_DEBAYER_METHODS(debayer10_BGBG_BGR888, debayer10_GRGR_BGR888)
			break;
		case 12:
			SET_DEBAYER_METHODS(debayer12_BGBG_BGR888, debayer12_GRGR_BGR888)
			break;
		}
		setupStandardBayerOrder(bayerFormat.order);
		return 0;
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
			SET_DEBAYER_METHODS(debayer10P_BGBG_BGR888, debayer10P_GRGR_BGR888)
			return 0;
		case BayerFormat::GBRG:
			SET_DEBAYER_METHODS(debayer10P_GBGB_BGR888, debayer10P_RGRG_BGR888)
			return 0;
		case BayerFormat::GRBG:
			SET_DEBAYER_METHODS(debayer10P_GRGR_BGR888, debayer10P_BGBG_BGR888)
			return 0;
		case BayerFormat::RGGB:
			SET_DEBAYER_METHODS(debayer10P_RGRG_BGR888, debayer10P_GBGB_BGR888)
			return 0;
		default:
			break;
		}
	}

	if (bayerFormat.bitDepth == 12 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
			SET_DEBAYER_METHODS(debayer12P_BGBG_BGR888, debayer12P_GRGR_BGR888)
			return 0;
		case BayerFormat::GBRG:
			SET_DEBAYER_METHODS(debayer12P_GBGB_BGR888, debayer12P_RGRG_BGR888)
			return 0;
		case BayerFormat::GRBG:
			SET_DEBAYER_METHODS(debayer12P_GRGR_BGR888, debayer12P_BGBG_BGR888)
			return 0;
		case BayerFormat::RGGB:
			SET_DEBAYER_METHODS(debayer12P_RGRG_BGR888, debayer12P_GBGB_BGR888)
			return 0;
		default:
			break;
		}
	}

	return invalidFmt();
}

int DebayerCpu::configure(const StreamConfiguration &inputCfg,
			  const std::vector<std::reference_wrapper<const StreamConfiguration>> &outputCfgs,
			  bool ccmEnabled)
{
	if (getInputConfig(inputCfg.pixelFormat, inputConfig_) != 0) {
		LOG(Debayer, Error)
			<< "Unsupported input format " << inputCfg.pixelFormat;
		return -EINVAL;
	}

	if (stats_->configure(inputCfg, threads_.size()) != 0)
		return -EINVAL;

	const Size &statsPatternSize = stats_->patternSize();
	if (inputConfig_.patternSize.width != statsPatternSize.width ||
	    inputConfig_.patternSize.height != statsPatternSize.height) {
		LOG(Debayer, Error)
			<< "mismatching stats and debayer pattern sizes for "
			<< inputCfg.pixelFormat.toString();
		return -EINVAL;
	}

	inputConfig_.stride = inputCfg.stride;

	if (outputCfgs.size() != 1) {
		LOG(Debayer, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	const StreamConfiguration &outputCfg = outputCfgs[0];
	SizeRange outSizeRange = sizes(inputCfg.pixelFormat, inputCfg.size);
	std::tie(outputConfig_.stride, outputConfig_.frameSize) =
		strideAndFrameSize(outputCfg.pixelFormat, outputCfg.size);

	if (!outSizeRange.contains(outputCfg.size) || outputConfig_.stride != outputCfg.stride) {
		LOG(Debayer, Error)
			<< "Invalid output size/stride: "
			<< "\n  " << outputCfg.size << " (" << outSizeRange << ")"
			<< "\n  " << outputCfg.stride << " (" << outputConfig_.stride << ")";
		return -EINVAL;
	}

	int ret = setDebayerFunctions(inputCfg.pixelFormat,
				      outputCfg.pixelFormat == formats::NV12 ? formats::XRGB8888 : outputCfg.pixelFormat,
				      ccmEnabled);
	if (ret != 0)
		return -EINVAL;

	window_.x = ((inputCfg.size.width - outputCfg.size.width) / 2) &
		    ~(inputConfig_.patternSize.width - 1);
	window_.y = ((inputCfg.size.height - outputCfg.size.height) / 2) &
		    ~(inputConfig_.patternSize.height - 1);
	window_.width = outputCfg.size.width;
	window_.height = outputCfg.size.height;

	raw64Pattern_ = -1;
#if defined(__arm__)
	// shortcut: only validated full-height RAW10 photos; extend after live preview/front parity checks.
	if (!ccmEnabled && colourBoost_ && window_.y == 0 && !(window_.width & 7) &&
	    window_.width * window_.height > 8000000 &&
	    access("/vendor/bin/libcamera/raw64_worker", X_OK) == 0) {
		if (debayer0_ == &DebayerCpu::debayer10P_BGBG_BGR888<true, false>) raw64Pattern_ = 0;
		if (debayer0_ == &DebayerCpu::debayer10P_GRGR_BGR888<true, false>) raw64Pattern_ = 1;
		if (debayer0_ == &DebayerCpu::debayer10P_GBGB_BGR888<true, false>) raw64Pattern_ = 2;
		if (debayer0_ == &DebayerCpu::debayer10P_RGRG_BGR888<true, false>) raw64Pattern_ = 3;
	}
#endif
	raw64Mapping_.reset();
	raw64Scratch_.reset();
	nv12Pixels_ = {};
	nv12Output_ = outputCfg.pixelFormat == formats::NV12;
	nv12Stride_ = outputCfg.stride;
	if (nv12Output_) {
		outputConfig_.stride = outputCfg.size.width * 4;
		const size_t smallLuma = size_t((outputCfg.size.width + 1) / 2) * ((outputCfg.size.height + 1) / 2);
		lumaBuffer_.resize(lumaDenoise_ ? std::min(size_t(outputConfig_.stride) * outputCfg.size.height, smallLuma + smoothLumaScratchBytes(outputCfg.size.width, outputCfg.size.height, nv12Stride_)) : 0);
		const unsigned int nv12Size = nv12Stride_ * (outputCfg.size.height + (outputCfg.size.height + 1) / 2);
		if (raw64Pattern_ >= 0) {
			DmaBufAllocator allocator(DmaBufAllocator::DmaBufAllocatorFlag::SystemHeap);
			std::vector<std::unique_ptr<FrameBuffer>> buffers;
			if (allocator.isValid() && allocator.exportBuffers(1, { nv12Size }, &buffers) == 1) {
				raw64Scratch_ = std::move(buffers[0]);
				raw64Mapping_ = std::make_unique<MappedFrameBuffer>(raw64Scratch_.get(), MappedFrameBuffer::MapFlag::ReadWrite);
				if (!raw64Mapping_->isValid()) {
					raw64Mapping_.reset();
					raw64Scratch_.reset();
				}
			}
		}
		nv12Buffer_.resize(raw64Mapping_ ? 0 : nv12Size);

		const size_t chromaWidth = (outputCfg.size.width / 2 + 1) / 2;
		const size_t chromaHeight = (outputCfg.size.height + 3) / 4;
		chromaBuffer_.resize(chromaDenoise_ ? chromaWidth * chromaHeight * 2 : 0);
	} else {
		lumaBuffer_.clear();
		nv12Buffer_.clear();
		chromaBuffer_.clear();
	}
	/* Release full-resolution scratch memory when returning to preview. */
	lumaBuffer_.shrink_to_fit();
	nv12Buffer_.shrink_to_fit();
	chromaBuffer_.shrink_to_fit();
	nv12Pixels_ = raw64Mapping_ ? raw64Mapping_->planes()[0] : std::span<uint8_t>(nv12Buffer_);

	ccmEnabled_ = ccmEnabled;

	/*
	 * Lookup tables must be initialized because the initial value is used for
	 * the first two frames, i.e. until stats processing starts providing its
	 * own parameters. Let's enforce recomputing lookup tables by setting the
	 * stored last used gamma to an out-of-range value.
	 */
	params_.gamma = 1.0;


	/*
	 * Set the stats window to the whole processed window. Its coordinates are
	 * relative to the debayered area since debayering passes only the part of
	 * data to be processed to the stats; see SwStatsCpu::setWindow.
	 */
	stats_->setWindow(Rectangle(window_.size()));

	unsigned int yStart = 0;
	unsigned int linesPerThread = (window_.height / threads_.size()) &
				      ~(inputConfig_.patternSize.height - 1);
	unsigned int i;

	for (i = 0; i < (threads_.size() - 1); i++) {
		threads_[i]->configure(yStart, yStart + linesPerThread);
		yStart += linesPerThread;
	}
	threads_[i]->configure(yStart, window_.height);

	return 0;
}

/**
 * \brief Configure thread to process a specific part of the image
 * \param[in] yStart y coordinate of first line to process
 * \param[in] yEnd y coordinate of the line at which to stop processing
 *
 * Configure the thread to process lines from yStart to yEnd - 1.
 */
void DebayerCpuThread::configure(unsigned int yStart, unsigned int yEnd)
{
	Debayer::DebayerInputConfig &inputConfig = debayer_->inputConfig_;

	yStart_ = yStart;
	yEnd_ = yEnd;
	rgbRows_.resize(debayer_->nv12Output_ && yStart < yEnd ?
		debayer_->outputConfig_.stride * inputConfig.patternSize.height : 0);
	rgbRows_.shrink_to_fit();

	/* pad with patternSize.Width on both left and right side */
	lineBufferPadding_ = inputConfig.patternSize.width * inputConfig.bpp / 8;
	lineBufferLength_ = debayer_->window_.width * inputConfig.bpp / 8 +
			    2 * lineBufferPadding_;

	if (enableInputMemcpy_) {
		for (unsigned int i = 0; i <= inputConfig.patternSize.height; i++)
			lineBuffers_[i].resize(lineBufferLength_);
	}
}

/*
 * Get width and height at which the bayer-pattern repeats.
 * Return pattern-size or an empty Size for an unsupported inputFormat.
 */
Size DebayerCpu::patternSize(PixelFormat inputFormat)
{
	DebayerCpu::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return {};

	return config.patternSize;
}

std::vector<PixelFormat> DebayerCpu::formats(PixelFormat inputFormat)
{
	DebayerCpu::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return std::vector<PixelFormat>();

	return config.outputFormats;
}

std::tuple<unsigned int, unsigned int>
DebayerCpu::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	/* round up to multiple of 8 for 64 bits alignment */
	const PixelFormatInfo &info = PixelFormatInfo::info(outputFormat);
	// Match the linear NV12 pitch of Osprey minigbm camera buffers.
	const unsigned int alignment = outputFormat == formats::NV12 ? 128 : 8;
	return std::make_tuple(info.stride(size.width, 0, alignment), info.frameSize(size, alignment));
}

void DebayerCpuThread::setupInputMemcpy(const uint8_t *linePointers[])
{
	const unsigned int patternHeight = debayer_->inputConfig_.patternSize.height;

	if (!enableInputMemcpy_)
		return;

	for (unsigned int i = 0; i < patternHeight; i++) {
		memcpy(lineBuffers_[i].data(),
		       linePointers[i + 1] - lineBufferPadding_,
		       lineBufferLength_);
		linePointers[i + 1] = lineBuffers_[i].data() + lineBufferPadding_;
	}

	/* Point lineBufferIndex_ to first unused lineBuffer */
	lineBufferIndex_ = patternHeight;
}

void DebayerCpuThread::shiftLinePointers(const uint8_t *linePointers[], const uint8_t *src)
{
	const unsigned int patternHeight = debayer_->inputConfig_.patternSize.height;

	for (unsigned int i = 0; i < patternHeight; i++)
		linePointers[i] = linePointers[i + 1];

	linePointers[patternHeight] =
		src + (patternHeight / 2) * (int)debayer_->inputConfig_.stride;
}

void DebayerCpuThread::memcpyNextLine(const uint8_t *linePointers[])
{
	const unsigned int patternHeight = debayer_->inputConfig_.patternSize.height;

	if (!enableInputMemcpy_)
		return;

	memcpy(lineBuffers_[lineBufferIndex_].data(),
	       linePointers[patternHeight] - lineBufferPadding_,
	       lineBufferLength_);
	linePointers[patternHeight] = lineBuffers_[lineBufferIndex_].data() + lineBufferPadding_;

	lineBufferIndex_ = (lineBufferIndex_ + 1) % (patternHeight + 1);
}

/**
 * \brief Process part of the image assigned to this debayer thread
 * \param[in] frame The frame number
 * \param[in] src The source buffer
 * \param[in] dst The destination buffer
 */
void DebayerCpuThread::process(uint32_t frame, const uint8_t *src, uint8_t *dst)
{
	Rectangle &window = debayer_->window_;

	/* Adjust src to top left corner of the window */
	src += (window.y + yStart_) * debayer_->inputConfig_.stride +
	       window.x * debayer_->inputConfig_.bpp / 8;
	if (debayer_->nv12Output_)
		dst = rgbRows_.data();
	else
		dst += yStart_ * debayer_->outputConfig_.stride;

	const int ret = debayer_->inputConfig_.patternSize.height == 2 ?
		process2(frame, src, dst) : process4(frame, src, dst);

	debayer_->workPendingMutex_.lock();
	if (ret)
		debayer_->conversionResult_ = ret;
	debayer_->workPending_ &= ~(1 << threadIndex_);
	debayer_->workPendingMutex_.unlock();
	debayer_->workPendingCv_.notify_one();
}

int DebayerCpuThread::convertRows(unsigned int first, unsigned int count)
{
	if (!debayer_->nv12Output_ || debayer_->raw64StatsOnly_)
		return 0;
	const size_t stride = debayer_->nv12Stride_;
	uint8_t *y = debayer_->nv12Pixels_.data() + first * stride;
	uint8_t *uv = debayer_->nv12Pixels_.data() + debayer_->window_.height * stride + (first / 2) * stride;
	return colourToNv12(rgbRows_.data(), debayer_->outputConfig_.stride,
			    y, stride, uv, stride, debayer_->window_.width, count,
			    debayer_->colourBoost_);
}

int DebayerCpuThread::process2(uint32_t frame, const uint8_t *src, uint8_t *dst)
{
	unsigned int outputStride = debayer_->outputConfig_.stride;
	unsigned int inputStride = debayer_->inputConfig_.stride;
	Rectangle &window = debayer_->window_;
	unsigned int yEnd = yEnd_;
	/* Holds [0] previous- [1] current- [2] next-line */
	const uint8_t *linePointers[3];

	/* [x] becomes [x - 1] after initial shiftLinePointers() call */
	if (window.y + yStart_) {
		linePointers[1] = src - inputStride; /* previous-line */
		linePointers[2] = src;
	} else {
		/* Top line, use the next line as prev line */
		linePointers[1] = src + inputStride;
		linePointers[2] = src;
	}

	if (window.y == 0 && yEnd_ == window.height) {
		/*
		 * Last 2 lines also need special handling.
		 * (And configure() ensures that yEnd >= 2.)
		 */
		yEnd -= 2;
	}

	setupInputMemcpy(linePointers);

	/*
	 * Note y is the line-number *inside* the window, since stats_' window
	 * is the stats window inside/relative to the debayer window. IOW for
	 * single thread rendering y goes from 0 to window.height.
	 */
	for (unsigned int y = yStart_; y < yEnd; y += 2) {
		if (debayer_->nv12Output_) dst = rgbRows_.data();
		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		debayer_->stats_->processLine0(frame, y, linePointers, threadIndex_);
		if (!debayer_->raw64StatsOnly_) debayer_->debayer0(dst, linePointers);
		src += inputStride;
		dst += outputStride;

		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		if (!debayer_->raw64StatsOnly_) debayer_->debayer1(dst, linePointers);
		src += inputStride;
		dst += outputStride;
		int ret = convertRows(y, 2);
		if (ret) return ret;
	}

	if (window.y == 0 && yEnd_ == window.height) {
		if (debayer_->nv12Output_) dst = rgbRows_.data();
		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		debayer_->stats_->processLine0(frame, yEnd, linePointers, threadIndex_);
		if (!debayer_->raw64StatsOnly_) debayer_->debayer0(dst, linePointers);
		src += inputStride;
		dst += outputStride;

		shiftLinePointers(linePointers, src);
		/* next line may point outside of src, use prev. */
		linePointers[2] = linePointers[0];
		if (!debayer_->raw64StatsOnly_) debayer_->debayer1(dst, linePointers);
		src += inputStride;
		dst += outputStride;
		return convertRows(yEnd, 2);
	}
	return 0;
}

int DebayerCpuThread::process4(uint32_t frame, const uint8_t *src, uint8_t *dst)
{
	unsigned int outputStride = debayer_->outputConfig_.stride;
	unsigned int inputStride = debayer_->inputConfig_.stride;

	/*
	 * This holds pointers to [0] 2-lines-up [1] 1-line-up [2] current-line
	 * [3] 1-line-down [4] 2-lines-down.
	 */
	const uint8_t *linePointers[5];

	/* [x] becomes [x - 1] after initial shiftLinePointers() call */
	linePointers[1] = src - 2 * inputStride;
	linePointers[2] = src - inputStride;
	linePointers[3] = src;
	linePointers[4] = src + inputStride;

	setupInputMemcpy(linePointers);

	/*
	 * Note y is the line-number *inside* the window, since stats_' window
	 * is the stats window inside/relative to the debayer window. IOW for
	 * single thread rendering y goes from 0 to window.height.
	 */
	for (unsigned int y = yStart_; y < yEnd_; y += 4) {
		if (debayer_->nv12Output_) dst = rgbRows_.data();
		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		debayer_->stats_->processLine0(frame, y, linePointers, threadIndex_);
		debayer_->debayer0(dst, linePointers);
		src += inputStride;
		dst += outputStride;

		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		debayer_->debayer1(dst, linePointers);
		src += inputStride;
		dst += outputStride;

		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		debayer_->stats_->processLine2(frame, y, linePointers, threadIndex_);
		debayer_->debayer2(dst, linePointers);
		src += inputStride;
		dst += outputStride;

		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		debayer_->debayer3(dst, linePointers);
		src += inputStride;
		dst += outputStride;
		int ret = convertRows(y, 4);
		if (ret) return ret;
	}
	return 0;
}

void DebayerCpu::updateGammaTable(const DebayerParams &params)
{
	const float gamma = params.gamma;
	const float contrastExp = params.contrastExp;

	const float divisor = gammaTable_.size() - 1.0;
	for (auto [i, value] : utils::enumerate(gammaTable_)) {
		float normalized = i / divisor;
		/* Convert 0..2 to 0..infinity; avoid actual inifinity at tan(pi/2) */
		/* Apply simple S-curve */
		if (normalized < 0.5)
			normalized = 0.5 * std::pow(normalized / 0.5, contrastExp);
		else
			normalized = 1.0 - 0.5 * std::pow((1.0 - normalized) / 0.5, contrastExp);
		value = UINT8_MAX * std::pow(normalized, gamma);
	}
}

void DebayerCpu::updateLookupTables(const DebayerParams &params)
{
	const bool gammaUpdateNeeded =
		params.gamma != params_.gamma ||
		params.blackLevel != params_.blackLevel ||
		params.contrastExp != params_.contrastExp;
	if (gammaUpdateNeeded)
		updateGammaTable(params);

	/* Processing order: black level -> gains -> gamma */
	auto matrixChanged = [](const Matrix<float, 3, 3> &m1, const Matrix<float, 3, 3> &m2) -> bool {
		return !std::equal(m1.data().begin(), m1.data().end(), m2.data().begin());
	};
	const unsigned int gammaTableSize = gammaTable_.size();

	const RGB<double> blackIndex = params.blackLevel * kRGBLookupSize;
	const RGB<double> gains = params.gains;
	const RGB<double> div = (RGB<double>(kRGBLookupSize) - blackIndex).max(1.0);

	if (ccmEnabled_) {
		if (gammaUpdateNeeded ||
		    matrixChanged(params.combinedMatrix, params_.combinedMatrix) ||
		    params.gains != params_.gains) {
			auto &red = swapRedBlueGains_ ? blueCcm_ : redCcm_;
			auto &green = greenCcm_;
			auto &blue = swapRedBlueGains_ ? redCcm_ : blueCcm_;
			const unsigned int redIndex = swapRedBlueGains_ ? 2 : 0;
			const unsigned int greenIndex = 1;
			const unsigned int blueIndex = swapRedBlueGains_ ? 0 : 2;
			for (unsigned int i = 0; i < kRGBLookupSize; i++) {
				const RGB<double> rgb = (gains * (RGB<double>(i) - blackIndex) * kRGBLookupSize / div)
							       .clamp(0.0, kRGBLookupSize - 1);
				red[i].r = std::round(rgb.r() * params.combinedMatrix[redIndex][0]);
				red[i].g = std::round(rgb.r() * params.combinedMatrix[greenIndex][0]);
				red[i].b = std::round(rgb.r() * params.combinedMatrix[blueIndex][0]);
				green[i].r = std::round(rgb.g() * params.combinedMatrix[redIndex][1]);
				green[i].g = std::round(rgb.g() * params.combinedMatrix[greenIndex][1]);
				green[i].b = std::round(rgb.g() * params.combinedMatrix[blueIndex][1]);
				blue[i].r = std::round(rgb.b() * params.combinedMatrix[redIndex][2]);
				blue[i].g = std::round(rgb.b() * params.combinedMatrix[greenIndex][2]);
				blue[i].b = std::round(rgb.b() * params.combinedMatrix[blueIndex][2]);
				gammaLut_[i] = gammaTable_[i * gammaTableSize / kRGBLookupSize];
			}
		}
	} else {
		if (gammaUpdateNeeded || params.gains != params_.gains) {
			auto &red = swapRedBlueGains_ ? blue_ : red_;
			auto &green = green_;
			auto &blue = swapRedBlueGains_ ? red_ : blue_;
			for (unsigned int i = 0; i < kRGBLookupSize; i++) {
				const RGB<double> lutGains =
					(gains * (RGB<double>(i) - blackIndex) * gammaTableSize / div)
						.clamp(0.0, gammaTableSize - 1);
				red[i] = gammaTable_[lutGains.r()];
				green[i] = gammaTable_[lutGains.g()];
				blue[i] = gammaTable_[lutGains.b()];
			}
		}
	}

	LOG(Debayer, Debug)
		<< "Debayer parameters: blackLevel=" << params.blackLevel
		<< "; gamma=" << params.gamma
		<< "; contrastExp=" << params.contrastExp
		<< "; gains=" << params.gains
		<< "; matrix=" << params.combinedMatrix;

	params_ = params;
}

void DebayerCpu::process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, const DebayerParams &params)
{
	bench_.startFrame();

	std::vector<DmaSyncer> dmaSyncers;

	dmaSyncBegin(dmaSyncers, input, output);

	updateLookupTables(params);

	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	stats_->startFrame(frame);
	auto complete = utils::scope_exit{ [&]() {
		dmaSyncers.clear();
		bench_.finishFrame();
		stats_->finishFrame(frame, 0, metadata.status != FrameMetadata::FrameError);
		outputBufferReady.emit(output);
		inputBufferReady.emit(input);
	} };

	MappedFrameBuffer in(input, MappedFrameBuffer::MapFlag::Read);
	MappedFrameBuffer out(output, output->_d()->isCpuCached() ?
        MappedFrameBuffer::MapFlag::ReadWrite : MappedFrameBuffer::MapFlag::Write);
	if (!in.isValid() || !out.isValid()) {
		LOG(Debayer, Error) << "mmap-ing buffer(s) failed";
		metadata.status = FrameMetadata::FrameError;
		return;
	}

	const auto scratchPixels = nv12Pixels_;
    auto restorePixels = utils::scope_exit{ [&]() { nv12Pixels_ = scratchPixels; } };
    const Rectangle uncropped(0, 0, window_.width, window_.height);
    const Rectangle requestedCrop = output->request() ?
        output->request()->controls().get(controls::ScalerCrop).value_or(uncropped) : uncropped;
    const size_t directYSize = size_t(nv12Stride_) * window_.height;
    const size_t directUvSize = size_t(nv12Stride_) * ((window_.height + 1) / 2);
    const bool directCachedOutput = nv12Output_ && !raw64Scratch_ &&
        uint64_t(window_.width) * window_.height <= 2073600 &&
        requestedCrop == uncropped && output->_d()->isCpuCached() &&
        output->planes().size() == 2 &&
        output->planes()[0].fd == output->planes()[1].fd &&
        output->planes()[1].offset == output->planes()[0].offset + directYSize &&
        out.planes().size() == 2 && out.planes()[0].size() >= directYSize &&
        out.planes()[1].size() >= directUvSize &&
        out.planes()[1].data() == out.planes()[0].data() + directYSize;
    if (directCachedOutput) {
        nv12Pixels_ = {out.planes()[0].data(), directYSize + directUvSize};
    }
    if (frame == 0)
        LOG(Debayer, Info) << "NV12 direct cached output=" << directCachedOutput;


	rawabi::Request rawRequest;
	bool useRaw64 = raw64Client_ && raw64Scratch_ && raw64Pattern_ >= 0 &&
		input->planes().size() == 1 && input->planes()[0].offset == 0;
	if (useRaw64) {
		rawRequest.sequence = frame;
		rawRequest.width = window_.width;
		rawRequest.height = window_.height;
		rawRequest.inputStride = inputConfig_.stride;
		rawRequest.outputStride = nv12Stride_;
		rawRequest.inputOffset = window_.x * inputConfig_.bpp / 8;
		rawRequest.pattern = raw64Pattern_;
		rawRequest.packed = 1;
		rawRequest.format = 1;
		rawRequest.blue = blue_; rawRequest.green = green_; rawRequest.red = red_;
		useRaw64 = rawabi::valid(rawRequest, input->planes()[0].length, nv12Pixels_.size());
	}
	std::optional<DmaSyncer> scratchSync;
	if (raw64Scratch_ && !useRaw64)
		scratchSync.emplace(raw64Scratch_->planes()[0].fd);
	raw64StatsOnly_ = useRaw64;
	auto cpuProcess = [&]() {
		workPendingMutex_.lock();
		workPending_ = (1 << threads_.size()) - 1;
		conversionResult_ = 0;
		workPendingMutex_.unlock();

		for (auto &thread : threads_)
			thread->invokeMethod(&DebayerCpuThread::process,
					     ConnectionTypeQueued, frame,
					     in.planes()[0].data(),
					     nv12Output_ ? nullptr : out.planes()[0].data());

		{
			MutexLocker locker(workPendingMutex_);
			workPendingCv_.wait(locker, [&]() LIBCAMERA_TSA_REQUIRES(workPendingMutex_) {
				return workPending_ == 0;
			});
			return conversionResult_;
		}

	};
	int conversionResult = cpuProcess();
	if (useRaw64) {
		const int fds[]{ input->planes()[0].fd.get(), raw64Scratch_->planes()[0].fd.get() };
		rawabi::Reply reply;
		const auto result = raw64Client_->render(rawRequest, fds, 2000, &reply);
		if (!result.quiescent)
			LOG(Debayer, Fatal) << "RAW64 still owns frame buffers after timeout; refusing buffer reuse";
		scratchSync.emplace(raw64Scratch_->planes()[0].fd);
		if (result.status) {
			LOG(Debayer, Warning) << "RAW64 disabled for stream: " << result.status;
			raw64Client_.reset();
			raw64StatsOnly_ = false;
			stats_->startFrame(frame);
			conversionResult = cpuProcess();
		} else {
			LOG(Debayer, Info) << "RAW64 photo frame=" << frame << " workerUs=" << reply.workerTimeLow;
		}
	}

    const int noiseMode = output->request() ?
        output->request()->controls().get(controls::draft::NoiseReductionMode).value_or(controls::draft::NoiseReductionModeFast) :
        controls::draft::NoiseReductionModeFast;

	if (nv12Output_) {
		const size_t ySize = nv12Stride_ * window_.height;
		const size_t uvSize = nv12Stride_ * ((window_.height + 1) / 2);
		const auto &planes = out.planes();
		if (planes[0].size() < ySize ||
		    (planes.size() == 1 ? planes[0].size() < ySize + uvSize
				       : planes[1].size() < uvSize)) {
			metadata.status = FrameMetadata::FrameError;
		} else {
			const Rectangle full(0, 0, window_.width, window_.height);
			Rectangle crop = output->request() ?
				output->request()->controls().get(controls::ScalerCrop).value_or(full) : full;
			crop = crop.boundedTo(full);
			bool lumaCopied = false;
			// Filter cached memory; DMA output is written once and never read back.
			uint8_t *y = nv12Pixels_.data();
			uint8_t *uv = y + ySize;
			// ARGBToNV12 overwrites active pixels; initialize only row padding.
			if (nv12Stride_ > window_.width)
				for (unsigned int row = 0; row < window_.height; ++row)
					std::fill(y + row * nv12Stride_ + window_.width,
						  y + (row + 1) * nv12Stride_, 16);
			const unsigned int uvWidth = ((window_.width + 1) / 2) * 2;
			if (nv12Stride_ > uvWidth)
				for (unsigned int row = 0; row < (window_.height + 1) / 2; ++row)
					std::fill(uv + row * nv12Stride_ + uvWidth,
						  uv + (row + 1) * nv12Stride_, 128);
			/* DRM XRGB8888 stores B,G,R,X bytes, matching libyuv ARGB. */
			int ret = conversionResult;
			if (ret)
				metadata.status = FrameMetadata::FrameError;
			else if (chromaDenoise_ && noiseMode != controls::draft::NoiseReductionModeOff) {
				// libyuv's NEON filter smooths chroma; luminance stays untouched.
				const int width = window_.width / 2;
				const int height = (window_.height + 1) / 2;
				const int halfWidth = (width + 1) / 2;
				const int halfHeight = (height + 1) / 2;
				uint8_t *smooth = chromaBuffer_.data();
				ret = libyuv::UVScale(uv, nv12Stride_, width, height,
							 smooth, halfWidth * 2, halfWidth, halfHeight,
							 libyuv::kFilterBox);
				if (!ret)
					ret = libyuv::UVScale(smooth, halfWidth * 2, halfWidth, halfHeight,
							     uv, nv12Stride_, width, height,
							     libyuv::kFilterBilinear);
				if (ret)
					metadata.status = FrameMetadata::FrameError;
			}
			if (metadata.status != FrameMetadata::FrameError && lumaDenoise_ && noiseMode == controls::draft::NoiseReductionModeHighQuality) {
				const int width = window_.width, height = window_.height;
				const int halfWidth = (width + 1) / 2, halfHeight = (height + 1) / 2;
				const size_t smallSize = halfWidth * halfHeight;
				if (lumaBuffer_.size() >= smallSize + smoothLumaScratchBytes(width, height, nv12Stride_)) {
					uint8_t *original = y;
					uint8_t *small = lumaBuffer_.data(), *smooth = small + smallSize;
					ret = libyuv::ScalePlane(original, nv12Stride_, width, height,
								 small, halfWidth, halfWidth, halfHeight, libyuv::kFilterBox);
					const bool copyLuma = crop == full && !(width & 1) && !(height & 1);
					uint8_t *target = copyLuma ? planes[0].data() : original;
					if (!ret)
						ret = smoothLumaPlane(target, original, small, smooth, width, height, nv12Stride_);
					if (ret)
						metadata.status = FrameMetadata::FrameError;
					else
						lumaCopied = copyLuma;
				}
			}
			if (metadata.status != FrameMetadata::FrameError) {
				uint8_t *dstY = planes[0].data();
				uint8_t *dstUV = planes.size() == 1 ? dstY + ySize : planes[1].data();
				if (window_.width * window_.height > 8000000)
					LOG(Debayer, Info) << "Photo rendered crop=" << crop << " full=" << full;
				crop.x &= ~1; crop.y &= ~1;
				crop.width &= ~1U; crop.height &= ~1U;
				if (crop.width < 2 || crop.height < 2) {
					metadata.status = FrameMetadata::FrameError;
				} else if (crop != full) {
					ret = libyuv::NV12Scale(y + crop.y * nv12Stride_ + crop.x, nv12Stride_,
						uv + (crop.y / 2) * nv12Stride_ + crop.x, nv12Stride_,
						crop.width, crop.height, dstY, nv12Stride_, dstUV, nv12Stride_,
						window_.width, window_.height, libyuv::kFilterBilinear);
					if (ret) metadata.status = FrameMetadata::FrameError;
				} else {
					if (!directCachedOutput) {
                        if (!lumaCopied) memcpy(dstY, y, ySize);
                        memcpy(dstUV, uv, uvSize);
                    }
				}
			}
		}
	}
	for (unsigned int i = 0; i < out.planes().size(); ++i)
		metadata.planes()[i].bytesused = out.planes()[i].size();

}

int DebayerCpu::start()
{
	if (raw64Scratch_ && raw64Pattern_ >= 0) {
		raw64Client_ = std::make_unique<rawabi::Client>();
		if (int ret = raw64Client_->start("/vendor/bin/libcamera/raw64_worker")) {
			LOG(Debayer, Warning) << "RAW64 start failed: " << ret;
			raw64Client_.reset();
		}
	}
	for (auto &thread : threads_)
		thread->start();

	return 0;
}

void DebayerCpu::stop()
{
	raw64Client_.reset();
	raw64StatsOnly_ = false;
	for (auto &thread : threads_)
		thread->exit();

	for (auto &thread : threads_)
		thread->wait();
}

SizeRange DebayerCpu::sizes(PixelFormat inputFormat, const Size &inputSize)
{
	Size patternSize = this->patternSize(inputFormat);
	unsigned int borderHeight = patternSize.height;

	if (patternSize.isNull())
		return {};

	/* No need for top/bottom border with a pattern height of 2 */
	if (patternSize.height == 2)
		borderHeight = 0;

	/*
	 * For debayer interpolation a border is kept around the entire image
	 * and the minimum output size is pattern-height x pattern-width.
	 */
	if (inputSize.width < (3 * patternSize.width) ||
	    inputSize.height < (2 * borderHeight + patternSize.height)) {
		LOG(Debayer, Warning)
			<< "Input format size too small: " << inputSize.toString();
		return {};
	}

	return SizeRange(Size(patternSize.width, patternSize.height),
			 Size((inputSize.width - 2 * patternSize.width) & ~(patternSize.width - 1),
			      (inputSize.height - 2 * borderHeight) & ~(patternSize.height - 1)),
			 patternSize.width, patternSize.height);
}

} /* namespace libcamera */
