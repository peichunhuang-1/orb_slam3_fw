#include "types/gpu/device.hpp"

#include <exception>
#include <stdexcept>

#ifdef ORB_HAVE_VULKAN
#include "types/gpu/context.hpp"
#endif

namespace gpu {

std::shared_ptr<Context> Device::context() {
    if (!tried_) {
        tried_ = true;
#ifdef ORB_HAVE_VULKAN
        try {
            ctx_ = Context::create();
        } catch (const std::exception& e) {
            error_ = e.what();
        }
#else
        error_ = "built without Vulkan";
#endif
    }
    if (!ctx_) throw std::runtime_error("gpu/Device: " + error_);
    return ctx_;
}

bool Device::available() {
    try {
        context();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace gpu
