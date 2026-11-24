#!/bin/sh
# initializeCommand: runs on the HOST (Pi OS, the NXP BSP or macOS, so plain sh) before the container is
# created, and tells compose.yaml which GPU to pass in. Docker refuses to start with --device on a missing
# path, so every device slot the host does not have gets a harmless placeholder instead.
#   NXP i.MX95 : /dev/mali0 (+ /dev/dri, /dev/dma_heap) and the BSP's libmali (/usr/lib/mali-imx)
#   Pi 5       : /dev/dri
#   macOS      : nothing (lavapipe)
cd "$(dirname "$0")" || exit 1
gpu=none
dri=/dev/null:/dev/fw-no-gpu
mali=/dev/null:/dev/fw-no-mali
heap=/dev/null:/dev/fw-no-dma-heap
mali_dir=./no-host-gpu
icds=/opt/mesa-v3dv/share/vulkan/icd.d/broadcom_icd.aarch64.json:/usr/share/vulkan/icd.d/lvp_icd.aarch64.json
[ -d /dev/dri ] && dri=/dev/dri && gpu=v3dv
if [ -c /dev/mali0 ] && [ -e /usr/lib/mali-imx/libmali.so.0 ]; then
    gpu=mali
    mali=/dev/mali0
    [ -d /dev/dma_heap ] && heap=/dev/dma_heap
    mali_dir=/usr/lib/mali-imx
    icds=/opt/mali-compat/mali_icd.json:/usr/share/vulkan/icd.d/lvp_icd.aarch64.json
fi
# macOS keeps building v3dv as before (it finds no device there); only the NXP board skips the build
[ "$gpu" = none ] && gpu=v3dv
cat > .env <<EOF
FW_GPU=$gpu
FW_GPU_DEVICE=$dri
FW_MALI_DEVICE=$mali
FW_DMA_HEAP_DEVICE=$heap
FW_HOST_MALI_DIR=$mali_dir
FW_VK_ICD_FILENAMES=$icds
EOF
echo "detect_gpu: FW_GPU=$gpu (dri=$dri mali=$mali)"
