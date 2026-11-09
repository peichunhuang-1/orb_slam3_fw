#pragma once
// A Vulkan device and the GPU pyramids handed from orb_extractor to orb_matcher without a CPU copy.
//
// In a graph the device is the box resource gpu/Device (types/gpu/device.hpp): the blocks that use the GPU
// take it with write access, so the framework gives one of them the device at a time (Vulkan wants the queue
// externally synchronized) and the graph shows who shares it. Outside the framework (tests, tools) one
// thread uses a Context, or the caller serializes the use: nothing here locks.
//
// GPU pyramids: the extractor copies its pyramid on the GPU (vkCmdCopyBuffer, ~0.36 ms for 752x480 on a
// Pi 5, no CPU) into a buffer from a PyramidPool and passes a reference-counted Pyramid in orb::Features;
// the buffer returns to the pool when the last message holding it goes away (any thread: the pool locks its
// free list). CPU reads of this memory are slow on the Pi 5 (host-visible, coherent, not cached: ~1 GB/s),
// which is why a pyramid is not read back.
#include <vulkan/vulkan.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace gpu {

class Context {
public:
    // A device of its own: the first integrated (else discrete) GPU with a compute queue; a software
    // device (llvmpipe) only with allowCpuDevice. perfQuery enables VK_KHR_performance_query (profiling).
    // Throws std::runtime_error without a usable device.
    static std::shared_ptr<Context> create(bool allowCpuDevice = false, bool perfQuery = false);
    ~Context();
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;

    VkInstance instance() const { return inst_; }
    VkPhysicalDevice physical() const { return phys_; }
    VkDevice device() const { return dev_; }
    uint32_t queueFamily() const { return qfam_; }
    uint32_t memoryType() const { return memType_; }   // host-visible, coherent (device-local if any)
    const std::string& deviceName() const { return name_; }
    float timestampPeriod() const { return tsPeriod_; }
    bool timestampsSupported() const { return tsSupported_; }

    // vkQueueSubmit on the device's queue (not locked: see the top of this file).
    VkResult submit(const VkSubmitInfo& si, VkFence fence) { return vkQueueSubmit(queue_, 1, &si, fence); }

    // Host-mapped storage buffer of the context's memory type (also a transfer source / destination).
    struct Buffer {
        VkBuffer buf = VK_NULL_HANDLE;
        VkDeviceMemory mem = VK_NULL_HANDLE;
        void* map = nullptr;
        VkDeviceSize size = 0;
    };
    Buffer makeBuffer(VkDeviceSize size);
    void freeBuffer(Buffer& b);

private:
    Context() = default;
    VkInstance inst_ = VK_NULL_HANDLE;
    VkPhysicalDevice phys_ = VK_NULL_HANDLE;
    VkDevice dev_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    uint32_t qfam_ = 0, memType_ = 0;
    std::string name_;
    float tsPeriod_ = 0.f;
    bool tsSupported_ = false;
};

// Geometry of one level of a packed GPU pyramid (OrbVk's layout, LevelInfo): byte offset of pixel (0, 0)
// and row stride, rows padded with repeated edge pixels.
struct Level {
    int w = 0, h = 0, stride = 0, goff = 0;
    float scale = 1.f;
};

// One image's pyramid in a GPU buffer, read only. Valid as long as the shared_ptr lives; its context lives
// as long too.
struct Pyramid {
    std::shared_ptr<Context> ctx;
    VkBuffer buffer = VK_NULL_HANDLE;
    const uint8_t* map = nullptr;   // host mapping (slow to read: not cached on the Pi 5)
    VkDeviceSize bytes = 0;
    std::vector<Level> levels;
};

// Buffers of one size, handed out as Pyramids and recycled when released.
class PyramidPool : public std::enable_shared_from_this<PyramidPool> {
public:
    PyramidPool(std::shared_ptr<Context> ctx, VkDeviceSize bytes) : ctx_(std::move(ctx)), bytes_(bytes) {}
    ~PyramidPool();
    // A free buffer (a new one if none is free); the caller fills it and its levels. The buffer goes back to
    // the pool when the returned pointer and its copies are gone.
    std::shared_ptr<Pyramid> acquire();
    size_t allocated() const;   // buffers created so far

private:
    std::shared_ptr<Context> ctx_;
    VkDeviceSize bytes_;
    mutable std::mutex m_;                   // the free list: released from whichever thread drops the last message
    std::vector<Context::Buffer> free_;      // every buffer is here when the pool goes (leases keep it alive)
    size_t allocated_ = 0;
};

}  // namespace gpu
