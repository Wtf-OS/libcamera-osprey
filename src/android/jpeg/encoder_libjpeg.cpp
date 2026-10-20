/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * JPEG encoding using libjpeg native API
 */

#include "encoder_libjpeg.h"

#include <fcntl.h>
#include <algorithm>
#include <array>
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif
#include <chrono>
#include <cstring>
#include <memory>
#include <new>
#include <pthread.h>
#include <libyuv/planar_functions.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

#include "../camera_buffer.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(JPEG)

#include "jpeg64/jpeg64-client.h"

namespace {

class JpegDestination : public jpeg_destination_mgr
{
public:
	explicit JpegDestination(std::span<uint8_t> buffer)
		: buffer_(buffer)
	{
		init_destination = [](j_compress_ptr cinfo) {
			auto *dest = static_cast<JpegDestination *>(cinfo->dest);
			dest->next_output_byte = dest->buffer_.data();
			dest->free_in_buffer = dest->buffer_.size();
		};
		empty_output_buffer = [](j_compress_ptr cinfo) -> boolean {
			auto *dest = static_cast<JpegDestination *>(cinfo->dest);
			dest->overflow_ |= dest->spilling_;
			dest->spilling_ = true;
			/* Finish safely into fixed scratch rather than reallocating HAL memory. */
			dest->next_output_byte = dest->scratch_.data();
			dest->free_in_buffer = dest->scratch_.size();
			return TRUE;
		};
		term_destination = [](j_compress_ptr) {};
	}

	int size() const
	{
		if (overflow_ || (spilling_ && free_in_buffer != scratch_.size()))
			return -ENOSPC;
		return spilling_ ? buffer_.size() : buffer_.size() - free_in_buffer;
	}

private:
	std::span<uint8_t> buffer_;
	std::array<JOCTET, 4096> scratch_;
	bool spilling_ = false;
	bool overflow_ = false;
};



int prepareJpegStrip(std::span<uint8_t> bytes, unsigned int width,
                     unsigned int height, unsigned int interval)
{
    if (bytes.size() < 4 || bytes[0] != 0xff || bytes[1] != 0xd8 ||
        bytes[bytes.size() - 2] != 0xff || bytes.back() != 0xd9)
        return -EINVAL;
    bool sof = false, dri = false;
    for (size_t p = 2; p + 4 <= bytes.size();) {
        if (bytes[p] != 0xff)
            return -EINVAL;
        const unsigned int marker = bytes[p + 1];
        if (marker != 0xc0 && marker != 0xc4 && marker != 0xdb && marker != 0xdd &&
            marker != 0xda && marker != 0xfe && (marker < 0xe0 || marker > 0xef))
            return -EINVAL;
        p += 2;
        const unsigned int length = (unsigned(bytes[p]) << 8) | bytes[p + 1];
        if (length < 2 || length > bytes.size() - p)
            return -EINVAL;
        if (marker == 0xc0) {
            if (sof || length != 17 || bytes[p + 2] != 8 || bytes[p + 7] != 3 ||
                ((unsigned(bytes[p + 5]) << 8) | bytes[p + 6]) != width)
                return -EINVAL;
            sof = true;
            bytes[p + 3] = height >> 8;
            bytes[p + 4] = height & 255;
        }
        if (marker == 0xdd) {
            if (dri || length != 4 ||
                ((unsigned(bytes[p + 2]) << 8) | bytes[p + 3]) != interval)
                return -EINVAL;
            dri = true;
        }
        if (marker == 0xda) {
            if (!sof || !dri || length != 12 || p + length > bytes.size() - 2)
                return -EINVAL;
            const size_t begin = p + length;
            for (size_t i = begin; i < bytes.size() - 2; ++i) {
#ifdef __ARM_NEON
                if (bytes.size() - 2 - i >= 16) {
                    const uint64x2_t markers = vreinterpretq_u64_u8(
                        vceqq_u8(vld1q_u8(bytes.data() + i), vdupq_n_u8(0xff)));
                    if (!(vgetq_lane_u64(markers, 0) | vgetq_lane_u64(markers, 1))) {
                        i += 15;
                        continue;
                    }
                }
#endif
                if (bytes[i] == 0xff) {
                    if (i + 1 >= bytes.size() - 2 || bytes[i + 1] != 0)
                        return -EINVAL;
                    ++i;
                }
            }
            return begin;
        }
        p += length;
    }
    return -EINVAL;
}

constexpr uint8_t expandJpegLuma(uint8_t sample)
{
	return std::clamp(((int(sample) - 16) * 298 + 128) >> 8, 0, 255);
}

constexpr uint8_t expandJpegChroma(uint8_t sample)
{
	return std::clamp((int(sample) * 292 - 4480) >> 8, 0, 255);
}


void expandJpegRow(const uint8_t *source, uint8_t *destination,
		   unsigned int width, bool chroma)
{
	unsigned int x = 0;
#ifdef __ARM_NEON
	const int16_t scale = chroma ? 292 : 298;
	const int32x4_t bias = vdupq_n_s32(chroma ? -4480 : -4640);
	for (; x + 8 <= width; x += 8) {
		const int16x8_t samples = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(source + x)));
		const int32x4_t low = vmlal_n_s16(bias, vget_low_s16(samples), scale);
		const int32x4_t high = vmlal_n_s16(bias, vget_high_s16(samples), scale);
		const uint16x8_t expanded = vcombine_u16(vqshrun_n_s32(low, 8),
						       vqshrun_n_s32(high, 8));
		vst1_u8(destination + x, vqmovn_u16(expanded));
	}
#endif
	for (; x < width; ++x)
		destination[x] = chroma ? expandJpegChroma(source[x]) : expandJpegLuma(source[x]);
}

struct JPEGPixelFormatInfo {
	J_COLOR_SPACE colorSpace;
	const PixelFormatInfo &pixelFormatInfo;
	bool nvSwap;
};

const std::map<PixelFormat, JPEGPixelFormatInfo> pixelInfo{
	{ formats::R8, { JCS_GRAYSCALE, PixelFormatInfo::info(formats::R8), false } },

	{ formats::RGB888, { JCS_EXT_BGR, PixelFormatInfo::info(formats::RGB888), false } },
	{ formats::BGR888, { JCS_EXT_RGB, PixelFormatInfo::info(formats::BGR888), false } },

	{ formats::NV12, { JCS_YCbCr, PixelFormatInfo::info(formats::NV12), false } },
	{ formats::NV21, { JCS_YCbCr, PixelFormatInfo::info(formats::NV21), true } },
	{ formats::NV16, { JCS_YCbCr, PixelFormatInfo::info(formats::NV16), false } },
	{ formats::NV61, { JCS_YCbCr, PixelFormatInfo::info(formats::NV61), true } },
	{ formats::NV24, { JCS_YCbCr, PixelFormatInfo::info(formats::NV24), false } },
	{ formats::NV42, { JCS_YCbCr, PixelFormatInfo::info(formats::NV42), true } },
};

const struct JPEGPixelFormatInfo &findPixelInfo(const PixelFormat &format)
{
	static const struct JPEGPixelFormatInfo invalidPixelFormat {
		JCS_UNKNOWN, PixelFormatInfo(), false
	};

	const auto iter = pixelInfo.find(format);
	if (iter == pixelInfo.end()) {
		LOG(JPEG, Error) << "Unsupported pixel format for JPEG encoder: "
				 << format;
		return invalidPixelFormat;
	}

	return iter->second;
}

} /* namespace */

EncoderLibJpeg::EncoderLibJpeg(size_t maxJpegBufferSize)
	: maxJpegBufferSize_(maxJpegBufferSize)
{
	/* \todo Expand error handling coverage with a custom handler. */
	compress_.err = jpeg_std_error(&jerr_);

	jpeg_create_compress(&compress_);
}

EncoderLibJpeg::~EncoderLibJpeg()
{
	jpeg_destroy_compress(&compress_);
}

int EncoderLibJpeg::configure(const StreamConfiguration &cfg)
{
	const struct JPEGPixelFormatInfo info = findPixelInfo(cfg.pixelFormat);
	if (info.colorSpace == JCS_UNKNOWN)
		return -ENOTSUP;

	compress_.image_width = cfg.size.width;
	compress_.image_height = cfg.size.height;
	compress_.in_color_space = info.colorSpace;

	compress_.input_components = info.colorSpace == JCS_GRAYSCALE ? 1 : 3;

	jpeg_set_defaults(&compress_);

	pixelFormatInfo_ = &info.pixelFormatInfo;

	nv_ = pixelFormatInfo_->numPlanes() == 2;
	limitedRange_ = nv_ && cfg.colorSpace && cfg.colorSpace->range == ColorSpace::Range::Limited;
	nvSwap_ = info.nvSwap;
	raw420_ = cfg.pixelFormat == formats::NV12 || cfg.pixelFormat == formats::NV21;
	compress_.raw_data_in = raw420_;
	strides_[0] = cfg.stride;
	const unsigned int packedStride = pixelFormatInfo_->stride(cfg.size.width, 0);
	if (!packedStride || cfg.stride < packedStride)
		return -EINVAL;
	strides_[1] = nv_ ? cfg.stride * pixelFormatInfo_->stride(cfg.size.width, 1) / packedStride : 0;

	return 0;
}

void EncoderLibJpeg::compressRGB(const std::vector<std::span<uint8_t>> &planes)
{
	unsigned char *src = const_cast<unsigned char *>(planes[0].data());
	unsigned int stride = strides_[0];

	JSAMPROW row_pointer[1];

	while (compress_.next_scanline < compress_.image_height) {
		row_pointer[0] = &src[compress_.next_scanline * stride];
		jpeg_write_scanlines(&compress_, row_pointer, 1);
	}
}

/*
 * Compress the incoming buffer from a supported NV format.
 * This naively unpacks the semi-planar NV12 to a YUV888 format for libjpeg.
 */
void EncoderLibJpeg::compressNV(const std::vector<std::span<uint8_t>> &planes)
{
	std::vector<uint8_t> tmprowbuf(compress_.image_width * 3);

	/*
	 * \todo Use the raw api, and only unpack the cb/cr samples to new line
	 * buffers. If possible, see if we can set appropriate pixel strides
	 * too to save even that copy.
	 *
	 * Possible hints at:
	 * https://sourceforge.net/p/libjpeg/mailman/message/30815123/
	 */
	unsigned int y_stride = strides_[0];
	unsigned int c_stride = strides_[1];

	unsigned int horzSubSample = 2 * compress_.image_width / pixelFormatInfo_->stride(compress_.image_width, 1);
	unsigned int vertSubSample = pixelFormatInfo_->planes[1].verticalSubSampling;

	unsigned int c_inc = horzSubSample == 1 ? 2 : 0;
	unsigned int cb_pos = nvSwap_ ? 1 : 0;
	unsigned int cr_pos = nvSwap_ ? 0 : 1;

	const unsigned char *src = planes[0].data();
	const unsigned char *src_c = planes[1].data();

	JSAMPROW row_pointer[1];
	row_pointer[0] = tmprowbuf.data();

	for (unsigned int y = 0; y < compress_.image_height; y++) {
		unsigned char *dst = tmprowbuf.data();

		const unsigned char *src_y = src + y * y_stride;
		const unsigned char *src_cb = src_c + (y / vertSubSample) * c_stride + cb_pos;
		const unsigned char *src_cr = src_c + (y / vertSubSample) * c_stride + cr_pos;

		for (unsigned int x = 0; x < compress_.image_width; x += 2) {
			dst[0] = *src_y;
			dst[1] = *src_cb;
			dst[2] = *src_cr;
			src_y++;
			src_cb += c_inc;
			src_cr += c_inc;
			dst += 3;

			dst[0] = *src_y;
			dst[1] = *src_cb;
			dst[2] = *src_cr;
			src_y++;
			src_cb += 2;
			src_cr += 2;
			dst += 3;
		}

		if (limitedRange_) {
			for (unsigned int x = 0; x < compress_.image_width; ++x) {
				tmprowbuf[x * 3] = expandJpegLuma(tmprowbuf[x * 3]);
				tmprowbuf[x * 3 + 1] = expandJpegChroma(tmprowbuf[x * 3 + 1]);
				tmprowbuf[x * 3 + 2] = expandJpegChroma(tmprowbuf[x * 3 + 2]);
			}
		}
		jpeg_write_scanlines(&compress_, row_pointer, 1);
	}
}


bool EncoderLibJpeg::validNVRaw(const std::vector<std::span<uint8_t>> &planes) const
{
    const unsigned int width = compress_.image_width, height = compress_.image_height;
    const unsigned int cw = (width + 1) / 2, ch = (height + 1) / 2;
    return width && height && planes.size() == 2 && planes[0].data() && planes[1].data() &&
           strides_[0] >= width && strides_[1] >= cw * 2 &&
           planes[0].size() >= width && planes[1].size() >= cw * 2 &&
           height - 1 <= (planes[0].size() - width) / strides_[0] &&
           ch - 1 <= (planes[1].size() - cw * 2) / strides_[1];
}

int EncoderLibJpeg::compressNVRaw(const std::vector<std::span<uint8_t>> &planes)
{
	const unsigned int width = compress_.image_width, height = compress_.image_height;
	const unsigned int cw = (width + 1) / 2, ch = (height + 1) / 2;
	if (!validNVRaw(planes))
		return -EINVAL;

	const unsigned int yWidth = (width + 15) & ~15U, cWidth = yWidth / 2;
	std::vector<uint8_t> yBlock(yWidth * 16), cbBlock(cWidth * 8), crBlock(cWidth * 8);
	std::array<JSAMPROW, 16> yRows;
	std::array<JSAMPROW, 8> cbRows, crRows;
	for (unsigned int row = 0; row < 16; ++row)
		yRows[row] = yBlock.data() + row * yWidth;
	for (unsigned int row = 0; row < 8; ++row) {
		cbRows[row] = cbBlock.data() + row * cWidth;
		crRows[row] = crBlock.data() + row * cWidth;
	}
	JSAMPARRAY rows[] = { yRows.data(), cbRows.data(), crRows.data() };
	while (compress_.next_scanline < height) {
		const unsigned int first = compress_.next_scanline;
		for (unsigned int row = 0; row < 16; ++row) {
			const unsigned int inputRow = std::min(first + row, height - 1);
			const uint8_t *source = planes[0].data() + inputRow * strides_[0];
			if (limitedRange_)
				expandJpegRow(source, yRows[row], width, false);
			else
				memcpy(yRows[row], source, width);
			std::fill(yRows[row] + width, yRows[row] + yWidth, yRows[row][width - 1]);
		}
		const unsigned int count = std::min(8U, ch - first / 2);
		libyuv::SplitUVPlane(planes[1].data() + (first / 2) * strides_[1], strides_[1],
				    nvSwap_ ? crBlock.data() : cbBlock.data(), cWidth,
				    nvSwap_ ? cbBlock.data() : crBlock.data(), cWidth, cw, count);
		for (unsigned int row = 0; row < count; ++row) {
			if (limitedRange_) {
				expandJpegRow(cbRows[row], cbRows[row], cw, true);
				expandJpegRow(crRows[row], crRows[row], cw, true);
			}
			std::fill(cbRows[row] + cw, cbRows[row] + cWidth, cbRows[row][cw - 1]);
			std::fill(crRows[row] + cw, crRows[row] + cWidth, crRows[row][cw - 1]);
		}
		for (unsigned int row = count; row < 8; ++row) {
			memcpy(cbRows[row], cbRows[count - 1], cWidth);
			memcpy(crRows[row], crRows[count - 1], cWidth);
		}
		if (jpeg_write_raw_data(&compress_, rows, 16) != 16)
			return -EIO;
	}
	return 0;
}

int EncoderLibJpeg::encode(Camera3RequestDescriptor::StreamBuffer *buffer,
			   std::span<const uint8_t> exifData,
			   unsigned int quality)
{
	MappedFrameBuffer frame(buffer->srcBuffer,
				MappedFrameBuffer::MapFlag::Read);
	if (!frame.isValid()) {
		LOG(JPEG, Error) << "Failed to map FrameBuffer : "
				 << strerror(frame.error());
		return frame.error();
	}

	const auto destination = buffer->dstBuffer->plane(0);
	const size_t capacity = buffer->dstBuffer->jpegBufferSize(maxJpegBufferSize_);
	if (capacity <= sizeof(camera3_jpeg_blob) || capacity > destination.size())
		return -ENOSPC;
    const auto target = destination.first(capacity - sizeof(camera3_jpeg_blob));
    if (raw420_ && size_t(compress_.image_width) * compress_.image_height > 4000000 &&
        !access("/vendor/bin/libcamera/jpeg64_worker", X_OK)) {
        const int result = jpegabi::encode(*buffer->srcBuffer, target, exifData,
                                          compress_.image_width, compress_.image_height,
                                          strides_[0], strides_[1], quality, nvSwap_, limitedRange_);
        if (result >= 0) return result;
        LOG(JPEG, Warning) << "JPEG64 fallback status=" << result;
    }
    return encode(frame.planes(), target, exifData, quality);
}

int EncoderLibJpeg::encode(const std::vector<std::span<uint8_t>> &src,
			   std::span<uint8_t> dest, std::span<const uint8_t> exifData,
			   unsigned int quality)
{
	if (!pixelFormatInfo_ || src.size() != pixelFormatInfo_->numPlanes() ||
	    dest.empty() || !dest.data() || dest.size() > std::numeric_limits<int>::max() ||
	    !compress_.image_width || !compress_.image_height ||
	    compress_.image_width > JPEG_MAX_DIMENSION || compress_.image_height > JPEG_MAX_DIMENSION ||
	    quality > 100)
		return -EINVAL;
	if (exifData.size() > 65533)
		return -E2BIG;

	const auto started = std::chrono::steady_clock::now();
    if (raw420_ && !compress_.restart_interval &&
        size_t(compress_.image_width) * compress_.image_height > 8000000) {
        const int result = encodeParallel(src, dest, exifData, quality);
        if (result >= 0) {
            LOG(JPEG, Info) << "JPEG parallel size=" << compress_.image_width << "x"
                << compress_.image_height << " encode-us="
                << std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - started).count();
            return result;
        }
    }

	JpegDestination destination(dest);
	compress_.dest = &destination;
	auto cleanup = utils::scope_exit{ [&]() {
		jpeg_abort_compress(&compress_);
		compress_.dest = nullptr;
	} };

	jpeg_set_quality(&compress_, quality, TRUE);

	jpeg_start_compress(&compress_, TRUE);

	if (exifData.size())
		/* Store Exif data in the JPEG_APP1 data block. */
		jpeg_write_marker(&compress_, JPEG_APP0 + 1,
				  static_cast<const JOCTET *>(exifData.data()),
				  exifData.size());

	LOG(JPEG, Debug) << "JPEG Encode Starting:" << compress_.image_width
			 << "x" << compress_.image_height;

	if (raw420_) {
		int ret = compressNVRaw(src);
		if (ret < 0)
			return ret;
	} else if (nv_)
		compressNV(src);
	else
		compressRGB(src);

	jpeg_finish_compress(&compress_);
	LOG(JPEG, Debug) << "JPEG raw420=" << raw420_ << " size="
		<< compress_.image_width << "x" << compress_.image_height << " encode-us="
		<< std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now() - started).count();

	return destination.size();
}

int EncoderLibJpeg::encodeParallel(const std::vector<std::span<uint8_t>> &src,
                                 std::span<uint8_t> dest,
                                 std::span<const uint8_t> exifData, unsigned int quality)
{
    if (!validNVRaw(src) || dest.empty())
        return -EINVAL;
    const unsigned int width = compress_.image_width, height = compress_.image_height;
    const unsigned int bandRows = ((height + 15) / 16 + 3) / 4;
    const unsigned int bandHeight = bandRows * 16;
    const unsigned int count = (height + bandHeight - 1) / bandHeight;
    const unsigned int interval = bandRows * ((width + 15) / 16);
    if (interval > 65535 || !count || count > 4)
        return -ENOTSUP;

    struct Job {
        EncoderLibJpeg *parent;
        const std::vector<std::span<uint8_t>> *src;
        std::span<const uint8_t> exif;
        unsigned int start, height, quality, interval;
        size_t capacityLimit;
        std::unique_ptr<uint8_t[]> bytes;
        int size = -EIO, header = -EINVAL;
    };
    std::array<Job, 4> jobs{};
    const auto worker = [](void *context) -> void * {
        auto &job = *static_cast<Job *>(context);
        const EncoderLibJpeg &parent = *job.parent;
        EncoderLibJpeg encoder;
        encoder.compress_.image_width = parent.compress_.image_width;
        encoder.compress_.image_height = job.height;
        encoder.compress_.in_color_space = JCS_YCbCr;
        encoder.compress_.input_components = 3;
        jpeg_set_defaults(&encoder.compress_);
        encoder.compress_.raw_data_in = TRUE;
        encoder.compress_.restart_interval = job.interval;
        encoder.pixelFormatInfo_ = parent.pixelFormatInfo_;
        encoder.raw420_ = encoder.nv_ = true;
        encoder.nvSwap_ = parent.nvSwap_;
        encoder.limitedRange_ = parent.limitedRange_;
        encoder.strides_[0] = parent.strides_[0];
        encoder.strides_[1] = parent.strides_[1];
        const std::vector<std::span<uint8_t>> planes = {
            (*job.src)[0].subspan(size_t(job.start) * parent.strides_[0]),
            (*job.src)[1].subspan(size_t(job.start / 2) * parent.strides_[1]),
        };
        size_t capacity = std::min(job.capacityLimit,
            size_t(encoder.compress_.image_width) * job.height / 2 + 65536 + job.exif.size());
        for (;;) {
            job.bytes.reset();
            job.bytes.reset(new (std::nothrow) uint8_t[capacity]);
            if (!job.bytes) {
                job.size = -ENOMEM;
                return nullptr;
            }
            job.size = encoder.encode(planes, { job.bytes.get(), capacity }, job.exif, job.quality);
            if (job.size != -ENOSPC || capacity == job.capacityLimit)
                break;
            capacity += std::min(capacity, job.capacityLimit - capacity);
        }
        if (job.size > 0)
            job.header = prepareJpegStrip({ job.bytes.get(), size_t(job.size) },
                encoder.compress_.image_width, parent.compress_.image_height, job.interval);
        return nullptr;
    };

    std::array<pthread_t, 4> threads{};
    std::array<bool, 4> created{};
    for (unsigned int i = 0; i < count; ++i) {
        Job &job = jobs[i];
        job.parent = this;
        job.src = &src;
        job.exif = exifData;
        job.start = i * bandHeight;
        job.height = std::min(bandHeight, height - job.start);
        job.quality = quality;
        job.interval = interval;
        // shortcut: 16 MiB scratch budget; raise only after measuring memory headroom.
        job.capacityLimit = std::min(dest.size(), size_t(4 * 1024 * 1024));
        if (i + 1 < count && !pthread_create(&threads[i], nullptr, worker, &job))
            created[i] = true;
        else
            worker(&job);
    }
    for (unsigned int i = 0; i < count; ++i)
        if (created[i])
            pthread_join(threads[i], nullptr);

    if (jobs[0].size < 0 || jobs[0].header < 0)
        return -EIO;
    const size_t headerSize = jobs[0].header;
    if (headerSize + 2 > dest.size())
        return -ENOSPC;
    size_t total = headerSize + 2;
    for (unsigned int i = 0; i < count; ++i) {
        const Job &job = jobs[i];
        if (job.size < 0 || job.header < 0 || size_t(job.header) != headerSize ||
            std::memcmp(job.bytes.get(), jobs[0].bytes.get(), headerSize))
            return -EIO;
        const size_t bytes = size_t(job.size) - headerSize - 2 + (i ? 2 : 0);
        if (bytes > dest.size() - total)
            return -ENOSPC;
        total += bytes;
    }

    // Do not publish any bytes until all workers, headers and the full output size are valid.
    uint8_t *output = dest.data();
    std::memcpy(output, jobs[0].bytes.get(), headerSize);
    output += headerSize;
    for (unsigned int i = 0; i < count; ++i) {
        const Job &job = jobs[i];
        if (i) {
            *output++ = 0xff;
            *output++ = 0xd0 + (i - 1) % 8;
        }
        const size_t bytes = size_t(job.size) - headerSize - 2;
        std::memcpy(output, job.bytes.get() + headerSize, bytes);
        output += bytes;
    }
    *output++ = 0xff;
    *output = 0xd9;
    return total;
}
