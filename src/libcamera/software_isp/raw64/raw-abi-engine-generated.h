/* SPDX-License-Identifier: LGPL-2.1-or-later */
// Generated from the production-checked RAW fixture.
#pragma once
#include <cstring>
#include <bit>
static_assert(std::endian::native == std::endian::little);

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <chrono>
#include <sys/mman.h>
#include <unistd.h>
struct Tables {
 std::array<uint32_t,256> redPacked_,greenPacked_,bluePacked_;
 struct CcmColumn{int16_t r,g,b;};
 std::array<CcmColumn,256> redCcm_,greenCcm_,blueCcm_;
 std::array<uint8_t,256> red_,green_,blue_,gammaLut_;
 struct{int width;}window_;
};
struct Old : Tables {
template<bool addAlphaByte,bool ccmEnabled> void debayer10P_BGBG_BGR888(uint8_t*,const uint8_t*[]);
template<bool addAlphaByte,bool ccmEnabled> void debayer10P_GRGR_BGR888(uint8_t*,const uint8_t*[]);
template<bool addAlphaByte,bool ccmEnabled> void debayer10P_GBGB_BGR888(uint8_t*,const uint8_t*[]);
template<bool addAlphaByte,bool ccmEnabled> void debayer10P_RGRG_BGR888(uint8_t*,const uint8_t*[]);
};
struct New : Tables {
template<bool addAlphaByte,bool ccmEnabled> void debayer10P_BGBG_BGR888(uint8_t*,const uint8_t*[]);
template<bool addAlphaByte,bool ccmEnabled> void debayer10P_GRGR_BGR888(uint8_t*,const uint8_t*[]);
template<bool addAlphaByte,bool ccmEnabled> void debayer10P_GBGB_BGR888(uint8_t*,const uint8_t*[]);
template<bool addAlphaByte,bool ccmEnabled> void debayer10P_RGRG_BGR888(uint8_t*,const uint8_t*[]);
template<bool addAlphaByte,bool ccmEnabled,unsigned int pattern> int raw10Neon(uint8_t*&,const uint8_t*[]);
};
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
__attribute__((noinline)) void Old::debayer10P_BGBG_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
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
__attribute__((noinline)) void Old::debayer10P_GRGR_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
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
__attribute__((noinline)) void Old::debayer10P_GBGB_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
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
__attribute__((noinline)) void Old::debayer10P_RGRG_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
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

#undef STORE_PIXEL
#define STORE_PIXEL(b_, g_, r_)                         \
	if constexpr (!ccmEnabled && addAlphaByte) { \
		const uint32_t packed = bluePacked_[b_] | greenPacked_[g_] | redPacked_[r_]; \
		std::memcpy(dst, &packed, 4); dst += 4; \
	} else if constexpr (ccmEnabled) {                     \
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
	if constexpr (addAlphaByte && ccmEnabled)                     \
		*dst++ = 255;                           \
	x++;

template<bool addAlphaByte, bool ccmEnabled>
__attribute__((noinline)) void New::debayer10P_BGBG_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
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
__attribute__((noinline)) void New::debayer10P_GRGR_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
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
__attribute__((noinline)) void New::debayer10P_GBGB_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
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
__attribute__((noinline)) void New::debayer10P_RGRG_BGR888(uint8_t *__restrict dst, const uint8_t *src[])
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
