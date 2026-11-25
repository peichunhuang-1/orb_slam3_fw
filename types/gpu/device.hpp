#pragma once
// The GPU of a box, as a shared resource (type "gpu/Device", api_guide §7): every block that runs work on the
// GPU takes it, and the graph shows who shares it.
//
//   boxes:
//     main:
//       resources:
//         gpu: { type: gpu/Device }
//   edges:
//     - gpu -> [orb_extractor_left.gpu, orb_extractor_right.gpu, orb_matcher.gpu]
//
// Blocks take it with read access, so they use the GPU at the same time: the one thing Vulkan wants used by
// one thread at a time is the queue, and gpu::Context::submit locks it for the call (types/gpu/context.hpp).
// Write access would lock the device for the whole handler, CPU work included. One device lets one block's shaders read another's
// buffers: orb_extractor's GPU pyramids (types/gpu/context.hpp) go to orb_matcher's sub-pixel shader
// without a CPU copy.
//
// The device is created on first use (context(), inside a handler holding the resource; locked, as several
// readers may call it at once). A block may keep
// GPU objects made on it across handler calls (pipelines, buffers: they hold the Context alive), but uses
// them only in handlers that hold the resource.
#include <memory>
#include <mutex>
#include <string>

#include "fw/fw.hpp"

namespace gpu {

class Context;

struct Device {
    // The device, created on the first call; throws std::runtime_error when there is no usable GPU (or the
    // build has no Vulkan), the same error on every call after a failure.
    // Const: readers (fw::Read<gpu::Device>) create it as well.
    std::shared_ptr<Context> context() const;
    // false: built without Vulkan, or creating the device failed (error())
    bool available() const;
    std::string error() const;

private:
    mutable std::mutex m_;
    mutable std::shared_ptr<Context> ctx_;
    mutable std::string error_;
    mutable bool tried_ = false;
};

}  // namespace gpu

FW_REGISTER_TYPE(gpu::Device, "gpu/Device")
