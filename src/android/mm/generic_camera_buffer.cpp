/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Generic Android frame buffer backend
 */

#include "../camera_buffer.h"

#include <algorithm>
#include <list>
#include <memory>
#include <tuple>
#include <cros_gralloc_handle.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>

#include <libcamera/base/log.h>
#include <libcamera/formats.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

namespace {

class PreviewMapping final : public MappedBuffer
{
public:
	PreviewMapping(int fd, size_t length, int flags)
	{
		void *address = mmap(nullptr, length, flags, MAP_SHARED, fd, 0);
		if (address == MAP_FAILED) {
			error_ = -errno;
			return;
		}
		maps_.emplace_back(static_cast<uint8_t *>(address), length);
		planes_ = maps_;
	}
};

std::shared_ptr<PreviewMapping> previewMapping(int fd, size_t length, int flags)
{
	struct stat identity;
	if (fstat(fd, &identity) < 0)
		return {};

	/* Plane FDs can alias one GEM: share its full preview mapping. */
	if (length && identity.st_size >= static_cast<off_t>(length) &&
	    identity.st_size <= 4 * 1024 * 1024)
		length = identity.st_size;

	using Key = std::tuple<dev_t, ino_t, size_t, int>;
	using Entry = std::pair<Key, std::shared_ptr<PreviewMapping>>;
	static thread_local std::list<Entry> cache;
	const Key key{identity.st_dev, identity.st_ino, length, flags};
	auto found = std::find_if(cache.begin(), cache.end(),
				 [&](const Entry &entry) { return entry.first == key; });
	if (found != cache.end()) {
		cache.splice(cache.begin(), cache, found);
		return cache.front().second;
	}

	auto mapping = std::make_shared<PreviewMapping>(fd, length, flags);
	if (mapping->isValid()) {
		cache.emplace_front(key, mapping);
		/* Bound retained views per worker; active requests own their references. */
		if (cache.size() > 8)
			cache.pop_back();
	}
	return mapping;
}

} /* namespace */

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
	bool retainMappings_;
	std::vector<std::shared_ptr<PreviewMapping>> retainedMappings_;
	std::vector<PlaneInfo> planeInfo_;
};

CameraBuffer::Private::Private([[maybe_unused]] CameraBuffer *cameraBuffer,
			       buffer_handle_t camera3Buffer,
			       PixelFormat pixelFormat,
			       const Size &size, int flags)
	: fd_(-1), flags_(flags), bufferLength_(-1), mapped_(false),
	  retainMappings_(pixelFormat == formats::NV12)
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
	if (!retainedMappings_.empty() && (flags_ & PROT_WRITE))
		__sync_synchronize();
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

	// The dma-buf allocation may include padding beyond the mapped BLOB plane.
	return planeInfo_.empty() ? 0 : std::min<size_t>(planeInfo_[0].size, maxJpegBufferSize);
}

void CameraBuffer::Private::map()
{
	ASSERT(fd_ != -1);
	ASSERT(bufferLength_ >= 0);

 for (const auto &info : planeInfo_) {
  size_t length = static_cast<size_t>(info.offset) + info.size;
  /* Retain only small writable preview mappings, not full-resolution still buffers. */
  if (retainMappings_ && flags_ == (PROT_READ | PROT_WRITE) && bufferLength_ <= 4 * 1024 * 1024) {
   auto mapping = previewMapping(info.fd, length, flags_);
   if (!mapping || !mapping->isValid()) {
    error_ = mapping ? mapping->error() : -errno;
    LOG(HAL, Error) << "Failed to map preview plane";
    return;
   }
   planes_.emplace_back(mapping->planes()[0].data() + info.offset, info.size);
   retainedMappings_.push_back(std::move(mapping));
   continue;
  }
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
