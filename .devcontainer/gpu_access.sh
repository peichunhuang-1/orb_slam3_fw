#!/bin/bash
# postStartCommand: lets the container user open the GPU's DRM nodes (/dev/dri/*, passed in by
# --device=/dev/dri in devcontainer.json). On the host the nodes are root:render / root:video 0660, and
# those groups' ids differ between host OS versions, so instead of --group-add the container's own copies
# of the nodes (Docker creates them inside the container; the host's nodes are not changed) are opened up.
#   renderD*: the render node, what Vulkan compute (the orb_extractor GPU backend) runs on.
#   card*   : the primary nodes. v3dv also opens them while enumerating the GPU; without access Mesa
#             prints "MESA: error: Opening /dev/dri/cardN failed: Permission denied" (compute still works).
shopt -s nullglob
nodes=(/dev/dri/renderD*)
if [ ${#nodes[@]} -eq 0 ]; then
    # macOS (Docker Desktop has no GPU in its VM), or a Pi whose /dev/dri was not detected by detect_gpu.sh
    echo "gpu_access: no /dev/dri/renderD* in the container, Vulkan runs on lavapipe (CPU, needs allowCpuDevice)" >&2
    exit 0
fi
for n in "${nodes[@]}" /dev/dri/card*; do
    [ -r "$n" ] && [ -w "$n" ] || sudo chmod 0666 "$n"
done
if command -v vulkaninfo >/dev/null; then
    vulkaninfo --summary 2>/dev/null | grep -E "deviceName|driverInfo" || echo "gpu_access: vulkaninfo found no GPU" >&2
fi
exit 0
