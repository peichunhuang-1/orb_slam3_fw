# One image for all hosts, all arm64 Linux under Docker:
#   Raspberry Pi 5 : Vulkan on the VideoCore VII via the patched v3dv below (/dev/dri passed in).
#   NXP i.MX95     : Vulkan on the Mali-G310 via the host BSP's own libmali (mounted in, patched to load on
#                    the container's older glibc by mali/patch_libmali.py at start; /dev/mali0 passed in).
#   macOS (Apple Silicon, Docker Desktop): no GPU in the VM, Vulkan falls back to lavapipe (CPU).
# FW_GPU (v3dv | mali | none, from detect_gpu.sh via compose.yaml) only decides whether v3dv is built.
# The loader is given both drivers (VK_ICD_FILENAMES); v3dv finds no device on the Mac and is skipped,
# and on the Pi gpu::Context prefers the GPU over lavapipe (a CPU device, only used with allowCpuDevice).
#
# The base image holds ROS 2 Humble and fw 0.1.0 (with the behavior tree, M9 recording/viewer and the shm link), and
# both .vsix (fw-editor, fw-viewer) in /usr/local/share/fw/. fw kept the tag 0.1.0 across those changes, so an older
# fw-humble:0.1.0 on this machine has to be rebuilt first (fw/docker/build_base.sh arm64), then
# "Dev Containers: Rebuild Container" and `rm -rf build install log` (the blocks' ABI changed).

# ---------------------------------------------------------------------------------------------------
# Stage 1: Mesa's Raspberry Pi 5 Vulkan driver (v3dv). Ubuntu 22.04's Mesa (23.2) predates the Pi 5's
# V3D 7.1, so the driver is built from the same Mesa release as Raspberry Pi OS on the host, with
# mesa/v3dv-csd-max-sg-id.patch: V3DV_CSD_MAX_SG_ID=15 lets compute run more than one supergroup at a
# time (the GPU ORB extractor 1.6-1.8x faster, same results); without the variable it is stock Mesa.
# Only the installed driver (~20 MB) goes into the final image. Builds in ~10-15 min on a Pi 5
# (a few minutes on a Mac; raise MESA_JOBS there).
# ---------------------------------------------------------------------------------------------------
#
# Skipped (an empty /opt/mesa-v3dv) unless FW_GPU=v3dv: the NXP board has no use for it and little disk.
# A plain `if` and not a stage per GPU, because without buildx Docker's legacy builder builds every stage.
# ---------------------------------------------------------------------------------------------------
FROM fw-humble:0.1.0 AS mesa
USER root
ARG FW_GPU=v3dv
ARG MESA_VERSION=26.2.2
ARG MESA_JOBS=2
COPY mesa/v3dv-csd-max-sg-id.patch /tmp/
# One layer: build deps, download, build, install, delete the sources (keeps the build's disk use down)
RUN mkdir -p /opt/mesa-v3dv && if [ "$FW_GPU" = v3dv ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        build-essential pkg-config ca-certificates curl xz-utils patch bison flex ninja-build python3-pip \
        python3-mako python3-yaml python3-packaging libdrm-dev zlib1g-dev libzstd-dev libexpat1-dev \
      && rm -rf /var/lib/apt/lists/* \
      && pip3 install --no-cache-dir 'meson>=1.4,<2' \
      && cd /tmp && curl -fsSL https://archive.mesa3d.org/mesa-${MESA_VERSION}.tar.xz | tar xJ \
      && cd mesa-${MESA_VERSION} && patch -p1 < /tmp/v3dv-csd-max-sg-id.patch \
      && meson setup build --prefix=/opt/mesa-v3dv -Dbuildtype=release -Dvulkan-drivers=broadcom \
          -Dgallium-drivers= -Dplatforms= -Dglx=disabled -Degl=disabled -Dgles1=disabled -Dgles2=disabled \
          -Dopengl=false -Dllvm=disabled -Dvalgrind=disabled -Dlibunwind=disabled -Dvideo-codecs= \
      && ninja -C build -j${MESA_JOBS} && ninja -C build install \
      && cd / && rm -rf /tmp/mesa-${MESA_VERSION}; \
    fi && rm -f /tmp/v3dv-csd-max-sg-id.patch

# ---------------------------------------------------------------------------------------------------
# Stage 2: the dev image. Vulkan loader + headers, glslangValidator (shaders -> SPIR-V at build time),
# vulkaninfo, validation layers, the v3dv driver from stage 1, and Ubuntu's Mesa only for lavapipe
# (its own 23.2 broadcom ICD does not know the Pi 5 and is left out of VK_ICD_FILENAMES).
# For the NXP board: patchelf + libmali-compat.so (mali/compat.c) and libmali's own dependencies; the
# library itself is the host's (/opt/host-mali), patched into /opt/mali by gpu_access.sh.
# The GPU itself comes from compose.yaml (/dev/dri or /dev/mali0 when the host has one, see detect_gpu.sh).
# ---------------------------------------------------------------------------------------------------
FROM fw-humble:0.1.0
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
        libvulkan1 libvulkan-dev vulkan-tools vulkan-validationlayers glslang-tools mesa-vulkan-drivers \
        libdrm2 libzstd1 libexpat1 patchelf libwayland-client0 libwayland-server0 libffi8 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=mesa /opt/mesa-v3dv /opt/mesa-v3dv
COPY mali/compat.c mali/patch_libmali.py /opt/mali-compat/
RUN gcc -O2 -shared -fPIC -o /opt/mali-compat/libmali-compat.so /opt/mali-compat/compat.c \
    && chmod 0755 /opt/mali-compat/patch_libmali.py \
    && mkdir -p /opt/mali && chown fw:fw /opt/mali \
    && printf '%s\n' '{' '    "file_format_version": "1.0.0",' \
        '    "ICD": { "library_path": "/opt/mali/libmali.so.0", "api_version": "1.3.0" }' '}' \
        > /opt/mali-compat/mali_icd.json
# VK_ICD_FILENAMES: Ubuntu 22.04's loader (1.3.204) predates VK_DRIVER_FILES. v3dv first, lavapipe as fallback.
# On the NXP board compose.yaml overrides it with /opt/mali-compat/mali_icd.json + lavapipe.
ENV VK_ICD_FILENAMES=/opt/mesa-v3dv/share/vulkan/icd.d/broadcom_icd.aarch64.json:/usr/share/vulkan/icd.d/lvp_icd.aarch64.json
# The patch's faster compute scheduling, on for every process (each prints
# "v3dv test patch: CSD MAX_SG_ID = 15" once to stderr). Unset it (or set -1) for stock Mesa behaviour.
ENV V3DV_CSD_MAX_SG_ID=15
USER fw
