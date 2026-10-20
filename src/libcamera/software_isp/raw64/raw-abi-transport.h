/* SPDX-License-Identifier: LGPL-2.1-or-later */
// Single-owner-thread RAW10 client; callers must honor Result::quiescent.
#pragma once
#include "raw-abi-protocol.h"
#include <libcamera/internal/process.h>
#include <libcamera/base/unique_fd.h>
#include <algorithm>
#include <chrono>
#include <fcntl.h>
#include <poll.h>

namespace rawabi {
struct Result {
    int status;
    // False means the caller MUST retain both buffers, including on failure.
    bool quiescent;
};

template<typename RequestType = Request, typename ReplyType = Reply>
class ClientT {
public:
    ClientT() = default;
    ClientT(const ClientT &) = delete;
    ClientT &operator=(const ClientT &) = delete;
    ~ClientT() { stop(); }

    int start(const std::string &path)
    {
        if (socket_.isValid() || inFlight_) return -EBUSY;
        int fds[2];
        if (socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, fds)) return -errno;
        libcamera::UniqueFD parent(fds[0]), child(fds[1]);
        if (fcntl(parent.get(), F_SETFL, O_NONBLOCK)) return -errno;
        const std::array args{std::to_string(child.get())};
        const std::array keep{child.get()};
        int ret = process_.start(path, args, keep);
        if (ret) return ret;
        socket_ = std::move(parent);
        return 0;
    }

    bool stop()
    {
        socket_.reset();
        process_.kill();
        if (process_.waitForExit(2000)) return !inFlight_;
        inFlight_ = false;
        return true;
    }

    Result render(const RequestType &request, const int (&fds)[2], int timeoutMs, ReplyType *reply = nullptr)
    {
        if (timeoutMs < 0) return {-EINVAL, !inFlight_};
        if (!socket_.isValid()) return {-ENOTCONN, !inFlight_};
        const auto deadline = Clock::now() + std::chrono::milliseconds(timeoutMs);
        int ret;
        for (;;) {
            ret = sendRequest(socket_.get(), request, fds, 2);
            if (ret != -EAGAIN && ret != -EWOULDBLOCK) break;
            ret = wait(POLLOUT, deadline);
            if (ret) break;
        }
        if (ret) return {ret, stop()};
        inFlight_ = true;

        ReplyType received{};
        for (;;) {
            ret = wait(POLLIN, deadline);
            if (ret) return {ret, stop()};
            ret = receive(received);
            if (ret != -EAGAIN && ret != -EWOULDBLOCK && ret != -EINTR) break;
        }
        if (ret || received.magicValue != request.magicValue || received.versionValue != request.versionValue ||
            received.sequence != request.sequence || received.status > 0 || received.status < -4095)
            return {ret ? ret : -EPROTO, stop()};

        // A valid worker reply is sent only after unmapping and closing both FDs.
        inFlight_ = false;
        if (reply) *reply = received;
        if (received.status) stop();
        return {received.status, true};
    }

private:
    using Clock = std::chrono::steady_clock;

    int wait(short events, Clock::time_point deadline)
    {
        pollfd descriptor{socket_.get(), events, 0};
        for (;;) {
            const auto remaining = std::max(std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now()),
                                            std::chrono::milliseconds::zero());
            int ret = poll(&descriptor, 1, remaining.count());
            if (ret < 0 && errno == EINTR) {
                if (Clock::now() >= deadline) return -ETIMEDOUT;
                continue;
            }
            if (ret < 0) return -errno;
            if (!ret) return -ETIMEDOUT;
            if (descriptor.revents & events) return 0;
            return -EPIPE;
        }
    }

    int receive(ReplyType &reply)
    {
        iovec io{&reply, sizeof(reply)};
        alignas(cmsghdr) char control[CMSG_SPACE(8 * sizeof(int))]{};
        msghdr msg{};
        msg.msg_iov = &io; msg.msg_iovlen = 1;
        msg.msg_control = control; msg.msg_controllen = sizeof(control);
        const ssize_t count = recvmsg(socket_.get(), &msg, MSG_DONTWAIT | MSG_CMSG_CLOEXEC);
        if (count < 0) return -errno;
        bool ancillary = false;
        for (auto *c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
            ancillary = true;
            if (c->cmsg_level != SOL_SOCKET || c->cmsg_type != SCM_RIGHTS) continue;
            const auto *fds = reinterpret_cast<const int *>(CMSG_DATA(c));
            const size_t length = (c->cmsg_len - CMSG_LEN(0)) / sizeof(int);
            for (size_t i = 0; i < length; ++i) close(fds[i]);
        }
        return count == sizeof(reply) && !ancillary && !(msg.msg_flags & (MSG_TRUNC | MSG_CTRUNC))
            ? 0 : -EPROTO;
    }

    libcamera::Process process_;
    libcamera::UniqueFD socket_;
    bool inFlight_ = false;
};
class Client final : public ClientT<> {};
}
