/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Simple image thumbnailer
 */

#include "thumbnailer.h"

#include <libcamera/base/log.h>

#include <libcamera/formats.h>
#include <libyuv/scale.h>

#include "libcamera/internal/mapped_framebuffer.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(Thumbnailer)

Thumbnailer::Thumbnailer()
	: valid_(false)
{
}

void Thumbnailer::configure(const Size &sourceSize, PixelFormat pixelFormat, unsigned int stride)
{
	valid_ = false;
	stride_ = stride;
	sourceSize_ = sourceSize;
	pixelFormat_ = pixelFormat;

	if (pixelFormat_ != formats::NV12 || stride_ < sourceSize.width) {
		LOG(Thumbnailer, Error)
			<< "Failed to configure: Pixel Format "
			<< pixelFormat_ << " unsupported.";
		return;
	}

	valid_ = true;
}

void Thumbnailer::createThumbnail(const FrameBuffer &source,
				  const Size &targetSize,
				  std::vector<unsigned char> *destination)
{
	MappedFrameBuffer frame(&source, MappedFrameBuffer::MapFlag::Read);
	if (!frame.isValid()) {
		LOG(Thumbnailer, Error)
			<< "Failed to map FrameBuffer : "
			<< strerror(frame.error());
		return;
	}

	if (!valid_) {
		LOG(Thumbnailer, Error) << "Config is unconfigured or invalid.";
		return;
	}

	const unsigned int tw = targetSize.width;
	const unsigned int th = targetSize.height;
	if (!tw || !th || (tw | th) & 1) {
		LOG(Thumbnailer, Error) << "Invalid NV12 thumbnail size " << targetSize;
		return;
	}

	ASSERT(frame.planes().size() == 2);
	const Size crop = sourceSize_.boundedToAspectRatio(targetSize).alignedDownTo(2, 2);
	if (!crop.width || !crop.height)
		return;
	const unsigned int x = ((sourceSize_.width - crop.width) / 2) & ~1U;
	const unsigned int y = ((sourceSize_.height - crop.height) / 2) & ~1U;
	destination->resize(tw * th * 3 / 2);
	const int ret = libyuv::NV12Scale(
		frame.planes()[0].data() + y * stride_ + x, stride_,
		frame.planes()[1].data() + (y / 2) * stride_ + x, stride_,
		crop.width, crop.height, destination->data(), tw,
		destination->data() + tw * th, tw, tw, th, libyuv::kFilterBilinear);
	if (ret) {
		LOG(Thumbnailer, Error) << "Failed NV12 thumbnail scaling: " << ret;
		destination->clear();
	}

}
