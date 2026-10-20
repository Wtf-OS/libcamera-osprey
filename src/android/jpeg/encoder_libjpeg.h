/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * JPEG encoding using libjpeg
 */

#pragma once

#include "encoder.h"

#include <vector>
#include <limits>

#include "libcamera/internal/formats.h"

#include <jpeglib.h>

class EncoderLibJpeg : public Encoder
{
public:
	EncoderLibJpeg(size_t maxJpegBufferSize = std::numeric_limits<size_t>::max());
	~EncoderLibJpeg();

	int configure(const libcamera::StreamConfiguration &cfg) override;
	int encode(Camera3RequestDescriptor::StreamBuffer *buffer,
		   std::span<const uint8_t> exifData,
		   unsigned int quality) override;
	int encode(const std::vector<std::span<uint8_t>> &planes,
		   std::span<uint8_t> destination,
		   std::span<const uint8_t> exifData,
		   unsigned int quality);

private:
	void compressRGB(const std::vector<std::span<uint8_t>> &planes);
	int compressNVRaw(const std::vector<std::span<uint8_t>> &planes);
	bool validNVRaw(const std::vector<std::span<uint8_t>> &planes) const;
	int encodeParallel(const std::vector<std::span<uint8_t>> &src, std::span<uint8_t> dest,
			   std::span<const uint8_t> exifData, unsigned int quality);
	void compressNV(const std::vector<std::span<uint8_t>> &planes);

	struct jpeg_compress_struct compress_;
	struct jpeg_error_mgr jerr_;

	const libcamera::PixelFormatInfo *pixelFormatInfo_ = nullptr;
	size_t maxJpegBufferSize_;

	unsigned int strides_[2];
	bool raw420_ = false;
	bool limitedRange_ = false;
	bool nv_;
	bool nvSwap_;
};
