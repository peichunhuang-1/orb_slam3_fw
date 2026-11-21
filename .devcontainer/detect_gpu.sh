#!/bin/sh
# initializeCommand: runs on the HOST (Pi OS or macOS, so plain sh) before the container is created, and
# tells compose.yaml whether to pass the GPU in. Docker refuses to start with --device on a missing path,
# so on a host without /dev/dri (macOS) a harmless placeholder is passed instead.
cd "$(dirname "$0")" || exit 1
if [ -d /dev/dri ]; then
    dev=/dev/dri
else
    dev=/dev/null:/dev/fw-no-gpu
fi
printf 'FW_GPU_DEVICE=%s\n' "$dev" > .env
echo "detect_gpu: FW_GPU_DEVICE=$dev"
