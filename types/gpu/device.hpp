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
// Blocks take it with write access: Vulkan wants the queue used by one thread at a time, and that is what
// the framework's exclusive lock gives (no lock inside). One device lets one block's shaders read another's
// buffers: orb_extractor's GPU pyramids (types/gpu/context.hpp) go to orb_matcher's sub-pixel shader
// without a CPU copy.
//
// The device is created on first use (context(), inside a handler holding the resource). A block may keep
// GPU objects made on it across handler calls (pipelines, buffers: they hold the Context alive), but uses
// them only in handlers that hold the resource.
#include <memory>
#include <string>

#include "fw/fw.hpp"

namespace gpu {

class Context;

struct Device {
    // The device, created on the first call; throws std::runtime_error when there is no usable GPU (or the
    // build has no Vulkan), the same error on every call after a failure.
    std::shared_ptr<Context> context();
    // false: built without Vulkan, or creating the device failed (error())
    bool available();
    const std::string& error() const { return error_; }

private:
    std::shared_ptr<Context> ctx_;
    std::string error_;
    bool tried_ = false;
};

}  // namespace gpu

FW_REGISTER_TYPE(gpu::Device, "gpu/Device")
