/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "jpeg64-engine-generated.h"
#include "jpeg64-protocol.h"
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>

static int synchronize(int fd, uint64_t flags)
{
    dma_buf_sync operation{}; operation.flags = flags;
    int result;
    do { result = ioctl(fd, DMA_BUF_IOCTL_SYNC, &operation); } while (result < 0 && errno == EINTR);
    return result < 0 ? -errno : 0;
}

static int process(const jpegabi::Request &q, const std::array<int, 2> &fds, uint32_t &encoded)
{
    struct stat in{}, out{};
    if (fstat(fds[0], &in) || fstat(fds[1], &out)) return -errno;
    if ((in.st_dev == out.st_dev && in.st_ino == out.st_ino) ||
        in.st_size <= 0 || out.st_size <= 0 || !jpegabi::valid(q, in.st_size, out.st_size))
        return -EINVAL;
    const int seals = fcntl(fds[1], F_GET_SEALS);
    if (seals < 0 || (seals & (F_SEAL_GROW | F_SEAL_SHRINK)) != (F_SEAL_GROW | F_SEAL_SHRINK))
        return -EINVAL;
    void *input = mmap(nullptr, in.st_size, PROT_READ, MAP_SHARED, fds[0], 0);
    if (input == MAP_FAILED) return -errno;
    void *output = mmap(nullptr, out.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fds[1], 0);
    if (output == MAP_FAILED) { const int status = -errno; munmap(input, in.st_size); return status; }
    int status = synchronize(fds[0], DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
    if (!status) {
        EncoderLibJpeg encoder(q.width, q.height);
        encoder.info.count = 2; encoder.nv_ = encoder.raw420_ = true;
        encoder.limitedRange_ = q.limited; encoder.nvSwap_ = q.swapped;
        encoder.strides_[0] = q.yStride; encoder.strides_[1] = q.uvStride;
        encoder.compress_.in_color_space = JCS_YCbCr;
        jpeg_set_defaults(&encoder.compress_); encoder.compress_.raw_data_in = TRUE;
        auto *src = static_cast<uint8_t *>(input), *dst = static_cast<uint8_t *>(output);
        const std::vector<std::span<uint8_t>> planes{
            {src + q.yOffset, size_t(in.st_size) - q.yOffset},
            {src + q.uvOffset, size_t(in.st_size) - q.uvOffset}};
        const int size = encoder.encode(planes, {dst + jpegabi::headerBytes, q.capacity},
                                       {dst, q.exifBytes}, q.quality);
        status = size < 0 ? size : 0;
        if (!status) encoded = size;
        const int sync = synchronize(fds[0], DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
        if (!status) status = sync;
    }
    munmap(output, out.st_size); munmap(input, in.st_size);
    return status;
}

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    char *end = nullptr; const long number = strtol(argv[1], &end, 10);
    if (!end || *end || number < 0 || number > 1048575) return 2;
    const int socket = number;
    for (;;) {
        jpegabi::Request q; std::array<int, 2> fds;
        const int received = rawabi::receiveRequest(socket, q, fds);
        if (!received) break;
        if (received < 0 && received != -EINVAL) return 3;
        jpegabi::Reply reply; reply.sequence = q.sequence;
        const auto begin = std::chrono::steady_clock::now();
        reply.status = received < 0 ? received : process(q, fds, reply.encodedBytes);
        for (int fd : fds) if (fd >= 0) close(fd);
        const uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - begin).count();
        reply.workerTimeLow = us; reply.workerTimeHigh = us >> 32;
        ssize_t sent;
        do { sent = send(socket, &reply, sizeof(reply), MSG_NOSIGNAL); } while (sent < 0 && errno == EINTR);
        if (sent != sizeof(reply)) return 4;
    }
    close(socket);
    return 0;
}
