/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * JPEG Post Processor
 */

#include "post_processor_jpeg.h"

#include <chrono>

#include "../camera_device.h"
#include "../camera_metadata.h"
#include "../camera_request.h"
#if defined(OS_CHROMEOS)
#include "encoder_jea.h"
#else /* !defined(OS_CHROMEOS) */
#include "encoder_libjpeg.h"
#endif
#include "exif.h"

#include <libcamera/base/log.h>

#include <libcamera/formats.h>
#include "libcamera/internal/dma_buf_allocator.h"

using namespace libcamera;
using namespace std::chrono_literals;

LOG_DEFINE_CATEGORY(JPEG)

PostProcessorJpeg::PostProcessorJpeg(CameraDevice *const device)
	: cameraDevice_(device)
{
}

int PostProcessorJpeg::configure(const StreamConfiguration &inCfg,
				 const StreamConfiguration &outCfg)
{
	if (inCfg.size != outCfg.size) {
		LOG(JPEG, Error) << "Mismatch of input and output stream sizes";
		return -EINVAL;
	}

	if (outCfg.pixelFormat != formats::MJPEG) {
		LOG(JPEG, Error) << "Output stream pixel format is not JPEG";
		return -EINVAL;
	}

	streamSize_ = outCfg.size;
	sourceColorSpace_ = inCfg.colorSpace;

	thumbnailer_.configure(inCfg.size, inCfg.pixelFormat, inCfg.stride);

#if defined(OS_CHROMEOS)
	encoder_ = std::make_unique<EncoderJea>();
#else /* !defined(OS_CHROMEOS) */
	encoder_ = std::make_unique<EncoderLibJpeg>(cameraDevice_->maxJpegBufferSize());
#endif

	return encoder_->configure(inCfg);
}

int PostProcessorJpeg::generateThumbnail(const FrameBuffer &source,
					  const Size &targetSize,
					  unsigned int quality,
					  std::vector<unsigned char> *thumbnail)
{
	/* Stores the raw scaled-down thumbnail bytes. */
	std::vector<unsigned char> rawThumbnail;

	thumbnailer_.createThumbnail(source, targetSize, &rawThumbnail);

	StreamConfiguration thCfg;
	thCfg.size = targetSize;
	thCfg.pixelFormat = thumbnailer_.pixelFormat();
	thCfg.stride = targetSize.width;
	thCfg.colorSpace = sourceColorSpace_;
	int ret = thumbnailEncoder_.configure(thCfg);
	if (ret < 0 || rawThumbnail.empty())
		return ret < 0 ? ret : -EINVAL;

	{
		/*
		 * \todo Avoid value-initialization of all elements of the
		 * vector.
		 */
		thumbnail->resize(rawThumbnail.size());

		/*
		 * Split planes manually as the encoder expects a vector of
		 * planes.
		 *
		 * \todo Pass a vector of planes directly to
		 * Thumbnailer::createThumbnailer above and remove the manual
		 * planes split from here.
		 */
		std::vector<std::span<uint8_t>> thumbnailPlanes;
		const PixelFormatInfo &formatNV12 = PixelFormatInfo::info(formats::NV12);
		size_t yPlaneSize = formatNV12.planeSize(targetSize, 0);
		size_t uvPlaneSize = formatNV12.planeSize(targetSize, 1);
		thumbnailPlanes.push_back({ rawThumbnail.data(), yPlaneSize });
		thumbnailPlanes.push_back({ rawThumbnail.data() + yPlaneSize, uvPlaneSize });

		int jpeg_size = thumbnailEncoder_.encode(thumbnailPlanes,
							 *thumbnail, {}, quality);
		if (jpeg_size == -ENOSPC && thumbnail->size() < 65533) {
			thumbnail->resize(65533);
			jpeg_size = thumbnailEncoder_.encode(thumbnailPlanes, *thumbnail, {}, quality);
		}
		if (jpeg_size < 0) {
			thumbnail->clear();
			LOG(JPEG, Error) << "Failed to encode JPEG thumbnail: " << jpeg_size;
			return jpeg_size;
		}
		thumbnail->resize(jpeg_size);

		LOG(JPEG, Debug)
			<< "Thumbnail compress returned "
			<< jpeg_size << " bytes";
	}
	return 0;
}

void PostProcessorJpeg::process(Camera3RequestDescriptor::StreamBuffer *streamBuffer)
{
	ASSERT(encoder_);

	const FrameBuffer &source = *streamBuffer->srcBuffer;
	std::vector<DmaSyncer> sourceSync;
	for (const auto &plane : source.planes())
		sourceSync.emplace_back(plane.fd, DmaSyncer::SyncType::Read);
	CameraBuffer *destination = streamBuffer->dstBuffer.get();

	ASSERT(destination->numPlanes() == 1);

	const CameraMetadata &requestMetadata = streamBuffer->request->settings_;
	CameraMetadata *resultMetadata = streamBuffer->request->resultMetadata_.get();
	camera_metadata_ro_entry_t entry;
	int ret;

	/* Set EXIF metadata for various tags. */
	Exif exif;
	exif.setMake(cameraDevice_->maker());
	exif.setModel(cameraDevice_->model());

	ret = requestMetadata.getEntry(ANDROID_JPEG_ORIENTATION, &entry);

	const uint32_t jpegOrientation = ret ? *entry.data.i32 : 0;
	resultMetadata->addEntry(ANDROID_JPEG_ORIENTATION, jpegOrientation);
	exif.setOrientation(jpegOrientation);

	exif.setSize(streamSize_);
	/*
	 * We set the frame's EXIF timestamp as the time of encode.
	 * Since the precision we need for EXIF timestamp is only one
	 * second, it is good enough.
	 */
	exif.setTimestamp(std::time(nullptr), 0ms);

	ret = resultMetadata->getEntry(ANDROID_SENSOR_EXPOSURE_TIME, &entry);
	exif.setExposureTime(ret ? *entry.data.i64 : 0);
	ret = resultMetadata->getEntry(ANDROID_LENS_APERTURE, &entry);
	if (ret)
		exif.setAperture(*entry.data.f);

	ret = resultMetadata->getEntry(ANDROID_SENSOR_SENSITIVITY, &entry);
	exif.setISO(ret ? *entry.data.i32 : 100);

	uint8_t flashState = resultMetadata->getEntry(ANDROID_FLASH_STATE, &entry) ?
		*entry.data.u8 : static_cast<uint8_t>(ANDROID_FLASH_STATE_READY);
	int flash = 0;
	if (flashState == ANDROID_FLASH_STATE_UNAVAILABLE) {
		flash = Exif::FlashNotPresent;
	} else {
		if (flashState == ANDROID_FLASH_STATE_FIRED ||
		    flashState == ANDROID_FLASH_STATE_PARTIAL)
			flash |= Exif::Fired;
		const uint8_t aeMode = requestMetadata.getEntry(ANDROID_CONTROL_AE_MODE, &entry) ?
			*entry.data.u8 : static_cast<uint8_t>(ANDROID_CONTROL_AE_MODE_ON);
		const uint8_t flashMode = resultMetadata->getEntry(ANDROID_FLASH_MODE, &entry) ?
			*entry.data.u8 : static_cast<uint8_t>(ANDROID_FLASH_MODE_OFF);
		if (aeMode == ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH ||
		    aeMode == ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE) {
			flash |= Exif::ModeAuto;
			if (aeMode == ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE)
				flash |= Exif::RedEye;
		} else if (aeMode == ANDROID_CONTROL_AE_MODE_ON_ALWAYS_FLASH ||
			   flashMode == ANDROID_FLASH_MODE_SINGLE ||
			   flashMode == ANDROID_FLASH_MODE_TORCH) {
			flash |= Exif::ModeCompulsoryFiring;
		} else {
			flash |= Exif::ModeCompulsorySuppression;
		}
	}
	exif.setFlash(static_cast<Exif::Flash>(flash));
	exif.setWhiteBalance(Exif::WhiteBalance::Auto);

	if (resultMetadata->getEntry(ANDROID_LENS_FOCAL_LENGTH, &entry) && entry.count)
		exif.setFocalLength(*entry.data.f);

	ret = requestMetadata.getEntry(ANDROID_JPEG_GPS_TIMESTAMP, &entry);
	if (ret) {
		exif.setGPSDateTimestamp(*entry.data.i64);
		resultMetadata->addEntry(ANDROID_JPEG_GPS_TIMESTAMP,
					 *entry.data.i64);
	}

	ret = requestMetadata.getEntry(ANDROID_JPEG_THUMBNAIL_SIZE, &entry);
	if (ret) {
		const int32_t *data = entry.data.i32;
		Size thumbnailSize = { static_cast<uint32_t>(data[0]),
				       static_cast<uint32_t>(data[1]) };

		ret = requestMetadata.getEntry(ANDROID_JPEG_THUMBNAIL_QUALITY, &entry);
		uint8_t quality = ret ? *entry.data.u8 : 95;
		resultMetadata->addEntry(ANDROID_JPEG_THUMBNAIL_QUALITY, quality);

		if (thumbnailSize != Size(0, 0)) {
			std::vector<unsigned char> thumbnail;
			ret = generateThumbnail(source, thumbnailSize, quality, &thumbnail);
			if (ret < 0) {
				sourceSync.clear();
				processComplete.emit(streamBuffer, PostProcessor::Status::Error);
				return;
			}
			if (!thumbnail.empty())
				exif.setThumbnail(std::move(thumbnail), Exif::Compression::JPEG);
		}

		resultMetadata->addEntry(ANDROID_JPEG_THUMBNAIL_SIZE, data, 2);
	}

	ret = requestMetadata.getEntry(ANDROID_JPEG_GPS_COORDINATES, &entry);
	if (ret) {
		exif.setGPSLocation(entry.data.d);
		resultMetadata->addEntry(ANDROID_JPEG_GPS_COORDINATES,
					 entry.data.d, 3);
	}

	ret = requestMetadata.getEntry(ANDROID_JPEG_GPS_PROCESSING_METHOD, &entry);
	if (ret) {
		std::string method(entry.data.u8, entry.data.u8 + entry.count);
		exif.setGPSMethod(method);
		resultMetadata->addEntry(ANDROID_JPEG_GPS_PROCESSING_METHOD,
					 entry.data.u8, entry.count);
	}

	if (exif.generate() != 0) {
		LOG(JPEG, Error) << "Failed to generate valid EXIF data";
		sourceSync.clear();
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	ret = requestMetadata.getEntry(ANDROID_JPEG_QUALITY, &entry);
	const uint8_t quality = ret ? *entry.data.u8 : 95;
	resultMetadata->addEntry(ANDROID_JPEG_QUALITY, quality);

	int jpeg_size = encoder_->encode(streamBuffer, exif.data(), quality);
	// Finish CPU reads before completion returns the source buffer for reuse.
	sourceSync.clear();
	const size_t capacity = destination->jpegBufferSize(cameraDevice_->maxJpegBufferSize());
	if (jpeg_size < 0 || capacity <= sizeof(camera3_jpeg_blob) ||
	    size_t(jpeg_size) > capacity - sizeof(camera3_jpeg_blob)) {
		LOG(JPEG, Error) << "Failed to encode stream image: result=" << jpeg_size
			<< " capacity=" << capacity << " mapped=" << destination->plane(0).size();
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	/* Fill in the JPEG blob header. */
	uint8_t *resultPtr = destination->plane(0).data()
			   + capacity - sizeof(struct camera3_jpeg_blob);
	auto *blob = reinterpret_cast<struct camera3_jpeg_blob *>(resultPtr);
	blob->jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
	blob->jpeg_size = jpeg_size;

	/* Update the JPEG result Metadata. */
	resultMetadata->addEntry(ANDROID_JPEG_SIZE, jpeg_size);
	processComplete.emit(streamBuffer, PostProcessor::Status::Success);
}
