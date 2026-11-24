#!/bin/bash
# postStartCommand: makes the GPU usable for the container user.
#
# 1. Device nodes. On the host they are root:render / root:video 0660 (or 0664), and those groups' ids
#    differ between host OSes, so instead of --group-add the container's own copies of the nodes (Docker
#    creates them inside the container; the host's nodes are not changed) are opened up.
#      /dev/dri/renderD*, card* : Pi 5 (v3dv; it also opens card* while enumerating the GPU)
#      /dev/mali0               : NXP i.MX95 Mali-G310 (mali_kbase), what libmali's Vulkan runs on
#      /dev/dma_heap/*          : NXP, buffer allocation for libmali
# 2. NXP only: the host's libmali (/opt/host-mali, read-only) is copied to /opt/mali and patched so the
#    container's glibc 2.35 can load it (mali/patch_libmali.py). Redone only when the host's file changes,
#    for example after a BSP update.
shopt -s nullglob
nodes=(/dev/dri/renderD* /dev/dri/card* /dev/mali* /dev/dma_heap/*)
for n in "${nodes[@]}"; do
    [ -r "$n" ] && [ -w "$n" ] || sudo chmod 0666 "$n"
done

src=/opt/host-mali/libmali.so.0
if [ -e "$src" ]; then
    sum=$(sha256sum "$(readlink -f "$src")" | cut -d' ' -f1)
    if [ ! -e /opt/mali/libmali.so.0 ] || [ "$(cat /opt/mali/.source.sha256 2>/dev/null)" != "$sum" ]; then
        python3 /opt/mali-compat/patch_libmali.py "$src" /opt/mali/libmali.so.0 /opt/mali-compat/libmali-compat.so \
            && echo "$sum" > /opt/mali/.source.sha256
    fi
fi

if [ ! -e /dev/mali0 ] && [ ! -e /dev/dri/renderD128 ]; then
    # macOS (Docker Desktop has no GPU in its VM), or a host whose GPU was not detected by detect_gpu.sh
    echo "gpu_access: no GPU device in the container, Vulkan runs on lavapipe (CPU, needs allowCpuDevice)" >&2
    exit 0
fi
if command -v vulkaninfo >/dev/null; then
    vulkaninfo --summary 2>/dev/null | grep -E "deviceName|driverInfo" || echo "gpu_access: vulkaninfo found no GPU" >&2
fi
exit 0
