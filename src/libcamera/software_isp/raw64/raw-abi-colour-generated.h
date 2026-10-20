/* SPDX-License-Identifier: LGPL-2.1-or-later */
// Exact current production colour matrix and fused NV12 converter.
#pragma once
#include <arm_neon.h>
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
