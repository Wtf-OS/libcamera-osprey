/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once
#include "jpeg64-protocol.h"
#include "../../../libcamera/software_isp/raw64/raw-abi-transport.h"
#include <libcamera/framebuffer.h>
#include <libcamera/base/utils.h>
#include <sys/mman.h>
#include <sys/stat.h>

namespace jpegabi {
inline int encode(const libcamera::FrameBuffer &input, std::span<uint8_t> destination,
                  std::span<const uint8_t> exif, unsigned width, unsigned height,
                  unsigned yStride, unsigned uvStride, unsigned quality, bool swapped, bool limited)
{
    const auto &planes = input.planes();
    if (planes.size() != 2 || destination.size() > 32 * 1024 * 1024 || exif.size() > 65533)
        return -ENOTSUP;
    struct stat first{}, second{};
    if (fstat(planes[0].fd.get(), &first) || fstat(planes[1].fd.get(), &second)) return -errno;
    if (first.st_size <= 0 || first.st_dev != second.st_dev || first.st_ino != second.st_ino)
        return -ENOTSUP;
    Request request;
    request.width = width; request.height = height;
    request.yStride = yStride; request.uvStride = uvStride;
    request.yOffset = planes[0].offset; request.uvOffset = planes[1].offset;
    request.capacity = destination.size(); request.exifBytes = exif.size();
    request.quality = quality; request.swapped = swapped; request.limited = limited;
    const size_t sharedSize = headerBytes + destination.size();
    if (!valid(request, first.st_size, sharedSize) ||
        uint64_t(height - 1) * yStride + width > planes[0].length ||
        uint64_t(height / 2 - 1) * uvStride + width > planes[1].length)
        return -EINVAL;
    libcamera::UniqueFD shared(memfd_create("osprey-jpeg64", MFD_CLOEXEC | MFD_ALLOW_SEALING));
    if (!shared.isValid()) return -errno;
    if (ftruncate(shared.get(), sharedSize) ||
        fcntl(shared.get(), F_ADD_SEALS, F_SEAL_GROW | F_SEAL_SHRINK)) return -errno;
    void *mapping = mmap(nullptr, sharedSize, PROT_READ | PROT_WRITE, MAP_SHARED, shared.get(), 0);
    if (mapping == MAP_FAILED) return -errno;
    const auto cleanup = libcamera::utils::scope_exit{[&]() { munmap(mapping, sharedSize); }};
    auto *bytes = static_cast<uint8_t *>(mapping);
    if (!exif.empty()) memcpy(bytes, exif.data(), exif.size());
    rawabi::ClientT<Request, Reply> client;
    if (const int error = client.start("/vendor/bin/libcamera/jpeg64_worker")) return error;
    const int descriptors[]{planes[0].fd.get(), shared.get()};
    Reply reply;
    const auto result = client.render(request, descriptors, 2000, &reply);
    if (!result.quiescent)
        LOG(JPEG, Fatal) << "JPEG64 still owns frame buffers; refusing buffer reuse";
    if (result.status) return result.status;
    const uint8_t *jpeg = bytes + headerBytes;
    if (reply.encodedBytes < 4 || reply.encodedBytes > destination.size() ||
        jpeg[0] != 0xff || jpeg[1] != 0xd8 ||
        jpeg[reply.encodedBytes - 2] != 0xff || jpeg[reply.encodedBytes - 1] != 0xd9)
        return -EPROTO;
    memcpy(destination.data(), jpeg, reply.encodedBytes);
    LOG(JPEG, Info) << "JPEG64 size=" << width << "x" << height
                   << " worker-us=" << reply.workerTimeLow << " bytes=" << reply.encodedBytes;
    return reply.encodedBytes;
}
}
