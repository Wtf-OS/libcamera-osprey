# Osprey JPEG64 helper

The ARM32 Android camera HAL passes a contiguous NV12/NV21 DMA-BUF and a sealed output memfd to a static AArch64 encoder. The helper uses the same bounded, parallel libjpeg algorithm and quality as the ARM32 implementation. Full-resolution front and rear photos above 4 MP use this path; unsupported buffer layouts and helper errors keep the existing encoder.

The output memfd reserves 64 KiB for EXIF, followed by the bounded JPEG destination. Requests and replies use fixed-width little-endian fields. The worker validates offsets, dimensions, strides, size limits, output seals and non-aliasing descriptors. It unmaps and closes both descriptors before replying. The caller retains the input and output until completion or confirmed process death, and refuses buffer reuse if termination cannot be confirmed.

`jpeg64-engine-generated.h` is generated from the production encoder and the existing standalone codec fixture. Regenerate it using `android17-port/performance/build-camera-jpeg64-transport.py` in the osprey workspace. Build the static ARM64 libjpeg/libyuv dependencies first using `build-camera-jpeg-abi.py` in the same directory. The generator excludes only the Android IPC dispatch from the standalone engine. The underlying JPEG algorithm remains the production implementation.

The Android product installs the validated helper as a prebuilt, alongside the matching HAL. `mainline-camera/jpeg64-runtime-manifest.json` pins both hashes; `android17-port/package.py` rejects stale product/image contents. The executable shares the isolated compute domain with RAW64, with access to passed HAL descriptors and camera-owned memfd files. SELinux remains Enforcing.

Device evidence is recorded in `camera-jpeg64-live-summary.json`, `camera-jpeg64-transport-device.log`, `camera-jpeg64-fault.json`, and `camera-jpeg64-shared-transport-regression.log` in the performance directory. The isolated real-frame check produced byte-identical JPEGs across ARM32/ARM64; this does not imply identical capture latency in all scenes.

Front 2552x1920/2560-stride parity, NV21/EXIF/bounds/timeout checks and live captures are recorded in `camera-front-jpeg-abi-transport.log` and `camera-front-jpeg64-series.json`. The isolated encode is faster; the first three-shot series did not establish a whole-capture latency improvement.
