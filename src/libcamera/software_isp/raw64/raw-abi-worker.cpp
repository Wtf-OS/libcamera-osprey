/* SPDX-License-Identifier: LGPL-2.1-or-later */
// Isolated RAW10 compute worker, not an Android camera service.
#include "raw-abi-render.h"
#include <linux/dma-buf.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <cstdlib>

static int synchronize(int fd, unsigned long long flags)
{
    dma_buf_sync operation{}; operation.flags = flags;
    int result;
    do { result = ioctl(fd,DMA_BUF_IOCTL_SYNC,&operation); } while (result < 0 && errno == EINTR);
    return result < 0 ? -errno : 0;
}

static int process(const rawabi::Request &q, const std::array<int,2> &fds)
{
    struct stat in{},out{};
    if (fstat(fds[0],&in) || fstat(fds[1],&out)) return -errno;
    if (in.st_dev == out.st_dev && in.st_ino == out.st_ino) return -EINVAL;
    if (in.st_size <= 0 || out.st_size <= 0 || !rawabi::valid(q,in.st_size,out.st_size)) return -EINVAL;
    void *input = mmap(nullptr,in.st_size,PROT_READ,MAP_SHARED,fds[0],0);
    if (input == MAP_FAILED) return -errno;
    void *output = mmap(nullptr,out.st_size,PROT_READ|PROT_WRITE,MAP_SHARED,fds[1],0);
    if (output == MAP_FAILED) { int status = -errno; munmap(input,in.st_size); return status; }
    int status = synchronize(fds[0],DMA_BUF_SYNC_START|DMA_BUF_SYNC_READ);
    const bool reading = !status;
    if (!status) status = synchronize(fds[1],DMA_BUF_SYNC_START|DMA_BUF_SYNC_WRITE);
    const bool writing = reading && !status;
    if (!status) {
        if (q.packed) { New engine; rawabi::render(engine,q,static_cast<const uint8_t *>(input),static_cast<uint8_t *>(output)); }
        else { Old engine; rawabi::render(engine,q,static_cast<const uint8_t *>(input),static_cast<uint8_t *>(output)); }
    }
    if (writing) { int ret = synchronize(fds[1],DMA_BUF_SYNC_END|DMA_BUF_SYNC_WRITE); if (!status) status = ret; }
    if (reading) { int ret = synchronize(fds[0],DMA_BUF_SYNC_END|DMA_BUF_SYNC_READ); if (!status) status = ret; }
    munmap(output,out.st_size); munmap(input,in.st_size);
    return status;
}

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    char *end = nullptr; long number = strtol(argv[1],&end,10);
    if (!end || *end || number < 0 || number > 1048575) return 2;
    const int socket = number;
    for (;;) {
        rawabi::Request q{}; std::array<int,2> fds;
        int received = rawabi::receiveRequest(socket,q,fds);
        if (received == 0) break;
        if (received < 0 && received != -EINVAL) return 3;
        rawabi::Reply reply; reply.sequence = q.sequence;
        const auto begin = std::chrono::steady_clock::now();
        reply.status = received < 0 ? received : process(q,fds);
        for (int fd : fds) if (fd >= 0) close(fd);
        const uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-begin).count();
        reply.workerTimeLow = us; reply.workerTimeHigh = us >> 32;
        ssize_t sent;
        do { sent = send(socket,&reply,sizeof(reply),MSG_NOSIGNAL); } while (sent < 0 && errno == EINTR);
        if (sent != sizeof(reply)) return 4;
    }
    close(socket);
    return 0;
}
