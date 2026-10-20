/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once
#include "../../../libcamera/software_isp/raw64/raw-abi-protocol.h"

namespace jpegabi {
constexpr uint32_t magic = 0x3147504a, version = 1, headerBytes = 65536;
struct Request {
    uint32_t magicValue = magic, versionValue = version, sequence = 0;
    uint32_t width = 0, height = 0, yStride = 0, uvStride = 0;
    uint32_t yOffset = 0, uvOffset = 0, capacity = 0, exifBytes = 0;
    uint32_t quality = 95, swapped = 0, limited = 1;
};
struct Reply {
    uint32_t magicValue = magic, versionValue = version, sequence = 0;
    int32_t status = 0;
    uint32_t workerTimeLow = 0, workerTimeHigh = 0, encodedBytes = 0;
};
static_assert(sizeof(Request) == 56 && sizeof(Reply) == 28);

inline bool valid(const Request &q, uint64_t inputBytes, uint64_t outputBytes)
{
    if (q.magicValue != magic || q.versionValue != version ||
        q.width < 4 || q.width > 4160 || q.width % 2 ||
        q.height < 2 || q.height > 3120 || q.height % 2 ||
        q.yStride < q.width || q.uvStride < q.width ||
        q.yStride > 65536 || q.uvStride > 65536 ||
        q.quality > 100 || q.swapped > 1 || q.limited > 1 ||
        q.exifBytes > 65533 || !q.capacity || q.capacity > 32 * 1024 * 1024 ||
        inputBytes > 32 * 1024 * 1024 || outputBytes > headerBytes + 32 * 1024 * 1024)
        return false;
    const uint64_t yEnd = uint64_t(q.yOffset) + uint64_t(q.height - 1) * q.yStride + q.width;
    const uint64_t uvEnd = uint64_t(q.uvOffset) + uint64_t(q.height / 2 - 1) * q.uvStride + q.width;
    return yEnd <= q.uvOffset && uvEnd <= inputBytes &&
           uint64_t(headerBytes) + q.capacity <= outputBytes;
}
}
