/* SPDX-License-Identifier: LGPL-2.1-or-later */
// Fixed-width little-endian protocol for the ARM32/AArch64 RAW10 helper.
#pragma once
#include <array>
#include <bit>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

namespace rawabi {
constexpr uint32_t magic = 0x31574152, version = 2;
struct Request {
    uint32_t magicValue = magic, versionValue = version, sequence = 0;
    uint32_t width = 0, height = 0, inputStride = 0, outputStride = 0;
    uint32_t pattern = 0, packed = 0;
    uint32_t format = 0, inputOffset = 2; // 0 = ARGB32; 1 = tuned NV12.
    std::array<uint8_t,256> blue{}, green{}, red{};
};
struct Reply {
    uint32_t magicValue = magic, versionValue = version, sequence = 0;
    int32_t status = 0;
    uint32_t workerTimeLow = 0, workerTimeHigh = 0;
};
static_assert(std::endian::native == std::endian::little);
static_assert(sizeof(Request) == 812 && offsetof(Request, blue) == 44);
static_assert(sizeof(Reply) == 24 && offsetof(Reply, status) == 12);

inline bool valid(const Request &q, uint64_t inputBytes, uint64_t outputBytes)
{
    if (q.magicValue != magic || q.versionValue != version || q.pattern > 3 || q.packed > 1 || q.format > 1 ||
        q.width < 4 || q.width > 4160 || q.width % 4 || q.height < 2 ||
        q.height > 3120 || q.height % 2 || q.inputStride > 65536 || q.outputStride > 65536 ||
        q.inputOffset < 2 || q.inputOffset > q.inputStride ||
        uint64_t(q.inputOffset) + q.width * 5 / 4 + 1 > q.inputStride ||
        q.outputStride < q.width * (q.format == 0 ? 4 : 1) ||
        (q.format == 1 && (q.width % 8 || q.outputStride % 2)))
        return false;
    const uint64_t requiredInput = uint64_t(q.height - 1) * q.inputStride + q.inputOffset + q.width * 5 / 4 + 1;
    const uint64_t rows = q.format == 0 ? q.height : q.height + q.height / 2;
    const uint64_t requiredOutput = (rows - 1) * q.outputStride + q.width * (q.format == 0 ? 4 : 1);
    return inputBytes <= 32 * 1024 * 1024 && outputBytes <= 64 * 1024 * 1024 &&
           requiredInput <= inputBytes && requiredOutput <= outputBytes;
}

template<typename RequestType>
inline int sendRequest(int socket, const RequestType &q, const int *fds, unsigned count)
{
    if (count > 2) return -EINVAL;
    iovec io{const_cast<RequestType *>(&q), sizeof(q)};
    alignas(cmsghdr) char control[CMSG_SPACE(2 * sizeof(int))]{};
    msghdr msg{}; msg.msg_iov = &io; msg.msg_iovlen = 1;
    if (count) {
        msg.msg_control = control; msg.msg_controllen = CMSG_SPACE(count * sizeof(int));
        auto *c = CMSG_FIRSTHDR(&msg);
        c->cmsg_level = SOL_SOCKET; c->cmsg_type = SCM_RIGHTS;
        c->cmsg_len = CMSG_LEN(count * sizeof(int));
        memcpy(CMSG_DATA(c), fds, count * sizeof(int));
    }
    ssize_t size;
    do { size = sendmsg(socket, &msg, MSG_NOSIGNAL); } while (size < 0 && errno == EINTR);
    return size == sizeof(q) ? 0 : size < 0 ? -errno : -EIO;
}

template<typename RequestType>
inline int receiveRequest(int socket, RequestType &q, std::array<int,2> &fds)
{
    fds.fill(-1);
    iovec io{&q, sizeof(q)};
    alignas(cmsghdr) char control[CMSG_SPACE(2 * sizeof(int))]{};
    msghdr msg{}; msg.msg_iov = &io; msg.msg_iovlen = 1;
    msg.msg_control = control; msg.msg_controllen = sizeof(control);
    ssize_t size;
    do { size = recvmsg(socket, &msg, MSG_CMSG_CLOEXEC); } while (size < 0 && errno == EINTR);
    if (size < 0) return -errno;
    if (size == 0 && msg.msg_controllen == 0) return 0;
    unsigned count = 0;
    bool bad = false;
    for (auto *c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg,c)) {
        if (c->cmsg_level != SOL_SOCKET || c->cmsg_type != SCM_RIGHTS ||
            c->cmsg_len < CMSG_LEN(0) || (c->cmsg_len - CMSG_LEN(0)) % sizeof(int)) {
            bad = true; continue;
        }
        const unsigned length = (c->cmsg_len - CMSG_LEN(0)) / sizeof(int);
        const auto *received = reinterpret_cast<const int *>(CMSG_DATA(c));
        for (unsigned i = 0; i < length; ++i) {
            if (count < fds.size()) fds[count++] = received[i];
            else { close(received[i]); bad = true; }
        }
    }
    return size == sizeof(q) && !(msg.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) &&
           count == 2 && !bad ? 1 : -EINVAL;
}
}
