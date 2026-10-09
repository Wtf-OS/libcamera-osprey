/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Generic Android frame buffer backend
 */

#include "../camera_buffer.h"

#include <cros_gralloc_handle.h>
#include <sys/mman.h>
#include <unistd.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

class CameraBuffer::Private : public Extensible::Private,
			      public MappedBuffer
{
	LIBCAMERA_DECLARE_PUBLIC(CameraBuffer)

public:
	Private(CameraBuffer *cameraBuffer, buffer_handle_t camera3Buffer,
		PixelFormat pixelFormat, const Size &size, int flags);
	~Private();

	unsigned int numPlanes() const;

	std::span<uint8_t> plane(unsigned int plane);

	unsigned int stride(unsigned int plane) const;
	unsigned int offset(unsigned int plane) const;
	unsigned int size(unsigned int plane) const;

	size_t jpegBufferSize(size_t maxJpegBufferSize) const;

private:
	struct PlaneInfo {
		int fd;
		unsigned int stride;
		unsigned int offset;
		unsigned int size;
	};

	void map();

	int fd_;
	int flags_;
	off_t bufferLength_;
	bool mapped_;
	std::vector<PlaneInfo> planeInfo_;
};

CameraBuffer::Private::Private([[maybe_unused]] CameraBuffer *cameraBuffer,
			       buffer_handle_t camera3Buffer,
			       PixelFormat pixelFormat,
			       const Size &size, int flags)
	: fd_(-1), flags_(flags), bufferLength_(-1), mapped_(false)
{
	error_ = 0;

	const auto &info = PixelFormatInfo::info(pixelFormat);
	if (!info.isValid()) {
		error_ = -EINVAL;
		LOG(HAL, Error) << "Invalid pixel format: " << pixelFormat;
		return;
	}

 // Minigbm may duplicate plane fds and append a metadata fd.
 const auto *h = reinterpret_cast<const cros_gralloc_handle *>(camera3Buffer);
 if (sizeof(native_handle_t) + (camera3Buffer->numFds + camera3Buffer->numInts) * sizeof(int) == sizeof(*h) && h->magic == 0xABCDDCBA) {
  if (h->num_planes != info.numPlanes() || h->num_planes > DRV_MAX_PLANES || camera3Buffer->numFds < static_cast<int>(h->num_planes)) {
   error_ = -EINVAL;
   return;
  }
  for (unsigned int i = 0; i < h->num_planes; ++i) {
   off_t length = lseek(h->fds[i], 0, SEEK_END);
   if (length < 0 || static_cast<uint64_t>(h->offsets[i]) + h->sizes[i] > static_cast<uint64_t>(length)) {
    error_ = -EINVAL;
    return;
   }
   planeInfo_.push_back({ h->fds[i], h->strides[i], h->offsets[i], h->sizes[i] });
  }
  fd_ = h->fds[0];
  bufferLength_ = lseek(fd_, 0, SEEK_END);
  return;
 }

	/*
	 * As Android doesn't offer an API to query buffer layouts, assume for
	 * now that the buffer is backed by a single dmabuf, with planes being
	 * stored contiguously.
	 */
	for (int i = 0; i < camera3Buffer->numFds; i++) {
		if (camera3Buffer->data[i] == -1 || camera3Buffer->data[i] == fd_)
			continue;

		if (fd_ != -1) {
			error_ = -EINVAL;
			LOG(HAL, Error) << "Discontiguous planes are not supported";
			return;
		}

		fd_ = camera3Buffer->data[i];
	}

	if (fd_ == -1) {
		error_ = -EINVAL;
		LOG(HAL, Error) << "No valid file descriptor";
		return;
	}

	bufferLength_ = lseek(fd_, 0, SEEK_END);
	if (bufferLength_ < 0) {
		error_ = -errno;
		LOG(HAL, Error) << "Failed to get buffer length";
		return;
	}

	const unsigned int numPlanes = info.numPlanes();
	planeInfo_.resize(numPlanes);

	unsigned int offset = 0;
	for (unsigned int i = 0; i < numPlanes; ++i) {
		const unsigned int planeSize = info.planeSize(size, i);

		planeInfo_[i].fd = fd_;
		planeInfo_[i].stride = info.stride(size.width, i, 1u);
		planeInfo_[i].offset = offset;
		planeInfo_[i].size = planeSize;

		if (bufferLength_ < offset + planeSize) {
			LOG(HAL, Error) << "Plane " << i << " is out of buffer:"
					<< " plane offset=" << offset
					<< ", plane size=" << planeSize
					<< ", buffer length=" << bufferLength_;
			return;
		}

		offset += planeSize;
	}
}

CameraBuffer::Private::~Private()
{
}

unsigned int CameraBuffer::Private::numPlanes() const
{
	return planeInfo_.size();
}

std::span<uint8_t> CameraBuffer::Private::plane(unsigned int plane)
{
	if (!mapped_)
		map();
	if (!mapped_)
		return {};

	return planes_[plane];
}

unsigned int CameraBuffer::Private::stride(unsigned int plane) const
{
	if (plane >= planeInfo_.size())
		return 0;

	return planeInfo_[plane].stride;
}

unsigned int CameraBuffer::Private::offset(unsigned int plane) const
{
	if (plane >= planeInfo_.size())
		return 0;

	return planeInfo_[plane].offset;
}

unsigned int CameraBuffer::Private::size(unsigned int plane) const
{
	if (plane >= planeInfo_.size())
		return 0;

	return planeInfo_[plane].size;
}

size_t CameraBuffer::Private::jpegBufferSize(size_t maxJpegBufferSize) const
{
	ASSERT(bufferLength_ >= 0);

	return std::min<unsigned int>(bufferLength_, maxJpegBufferSize);
}

void CameraBuffer::Private::map()
{
	ASSERT(fd_ != -1);
	ASSERT(bufferLength_ >= 0);

 for (const auto &info : planeInfo_) {
  size_t length = static_cast<size_t>(info.offset) + info.size;
  void *address = mmap(nullptr, length, flags_, MAP_SHARED, info.fd, 0);
  if (address == MAP_FAILED) {
   error_ = -errno;
   LOG(HAL, Error) << "Failed to mmap plane";
   return;
  }
  maps_.emplace_back(static_cast<uint8_t *>(address), length);
  planes_.emplace_back(static_cast<uint8_t *>(address) + info.offset, info.size);
 }

	mapped_ = true;
}

PUBLIC_CAMERA_BUFFER_IMPLEMENTATION
