/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once
#include <algorithm>
#include <cstdint>
#include <libcamera/geometry.h>

namespace libcamera {
inline Rectangle mapOspreyCrop(Rectangle requested, Size full, Rectangle visible,
                              Size output, Rectangle &applied)
{
    applied = {};
    if (!full.width || !full.height || !visible.width || !visible.height ||
        output.width < 2 || output.height < 2)
        return {};
    unsigned int width = std::clamp(requested.width, std::max(1U, full.width / 4), full.width);
    unsigned int height = std::clamp(requested.height, std::max(1U, full.height / 4), full.height);
    if (uint64_t(width) * output.height > uint64_t(height) * output.width)
        width = uint64_t(height) * output.width / output.height;
    else
        height = uint64_t(width) * output.height / output.width;
    const int left = std::max(int((full.width - width) / 2), visible.x);
    const int top = std::max(int((full.height - height) / 2), visible.y);
    const int right = std::min(int((full.width + width) / 2), visible.x + int(visible.width));
    const int bottom = std::min(int((full.height + height) / 2), visible.y + int(visible.height));
    if (right <= left || bottom <= top)
        return {};
    Rectangle scaled((uint64_t(left - visible.x) * output.width / visible.width) & ~1U,
                     (uint64_t(top - visible.y) * output.height / visible.height) & ~1U,
                     (uint64_t(right - left) * output.width / visible.width) & ~1U,
                     (uint64_t(bottom - top) * output.height / visible.height) & ~1U);
    if (!scaled.width || !scaled.height)
        return {};
    applied = Rectangle(visible.x + uint64_t(scaled.x) * visible.width / output.width,
                        visible.y + uint64_t(scaled.y) * visible.height / output.height,
                        uint64_t(scaled.width) * visible.width / output.width,
                        uint64_t(scaled.height) * visible.height / output.height);
    return scaled;
}
} /* namespace libcamera */
