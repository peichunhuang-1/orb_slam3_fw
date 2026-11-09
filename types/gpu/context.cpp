// Compiled only with Vulkan (ORB_HAVE_VULKAN), see context.hpp.
#ifdef ORB_HAVE_VULKAN
#include "types/gpu/context.hpp"

#include <cstdio>
#include <stdexcept>

namespace gpu {

namespace {

void check(VkResult r, const char* what) {
    if (r != VK_SUCCESS) {
        char m[160];
        std::snprintf(m, sizeof m, "%s failed (VkResult %d)", what, (int)r);
        throw std::runtime_error(m);
    }
}

}  // namespace

std::shared_ptr<Context> Context::create(bool allowCpuDevice, bool perfQuery) {
    std::shared_ptr<Context> c(new Context());
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "orb_vk";
    app.apiVersion = VK_API_VERSION_1_1;
    VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    ici.pApplicationInfo = &app;
    check(vkCreateInstance(&ici, nullptr, &c->inst_), "vkCreateInstance");

    // Device: prefer integrated, then discrete GPUs; CPU devices (llvmpipe) only if allowed.
    uint32_t n = 0;
    vkEnumeratePhysicalDevices(c->inst_, &n, nullptr);
    std::vector<VkPhysicalDevice> devs(n);
    vkEnumeratePhysicalDevices(c->inst_, &n, devs.data());
    int bestScore = -1;
    for (auto d : devs) {
        VkPhysicalDeviceProperties pr;
        vkGetPhysicalDeviceProperties(d, &pr);
        int sc = pr.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU ? 3
               : pr.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU   ? 2
               : pr.deviceType == VK_PHYSICAL_DEVICE_TYPE_CPU            ? (allowCpuDevice ? 0 : -1)
                                                                          : 1;
        uint32_t qn = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, nullptr);
        std::vector<VkQueueFamilyProperties> qf(qn);
        vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, qf.data());
        for (uint32_t i = 0; i < qn; ++i) {
            if ((qf[i].queueFlags & VK_QUEUE_COMPUTE_BIT) && sc > bestScore) {
                bestScore = sc;
                c->phys_ = d;
                c->qfam_ = i;
                c->tsSupported_ = qf[i].timestampValidBits > 0;
                break;
            }
        }
    }
    if (!c->phys_) throw std::runtime_error("no Vulkan GPU with a compute queue (use --allow-cpu for llvmpipe)");

    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(c->phys_, &props);
    c->name_ = props.deviceName;
    c->tsPeriod_ = props.limits.timestampPeriod;
    c->tsSupported_ = c->tsSupported_ && props.limits.timestampComputeAndGraphics;
    if (props.limits.maxComputeWorkGroupInvocations < 128)
        throw std::runtime_error("device maxComputeWorkGroupInvocations < 128");

    // Memory: unified memory SoCs (Pi 5, i.MX95), so one host-visible, coherent (ideally also
    // device-local) memory type for everything and no staging copies.
    VkPhysicalDeviceMemoryProperties mp;
    vkGetPhysicalDeviceMemoryProperties(c->phys_, &mp);
    const VkMemoryPropertyFlags want[] = {
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};
    bool found = false;
    for (auto f : want) {
        for (uint32_t i = 0; i < mp.memoryTypeCount && !found; ++i)
            if ((mp.memoryTypes[i].propertyFlags & f) == f) {
                c->memType_ = i;
                found = true;
            }
        if (found) break;
    }
    if (!found) throw std::runtime_error("no host-visible coherent memory type");

    float prio = 1.0f;
    VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    qci.queueFamilyIndex = c->qfam_;
    qci.queueCount = 1;
    qci.pQueuePriorities = &prio;
    VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    dci.queueCreateInfoCount = 1;
    dci.pQueueCreateInfos = &qci;
    const char* perfExt = VK_KHR_PERFORMANCE_QUERY_EXTENSION_NAME;
    VkPhysicalDevicePerformanceQueryFeaturesKHR pqf{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PERFORMANCE_QUERY_FEATURES_KHR};
    if (perfQuery) {
        pqf.performanceCounterQueryPools = VK_TRUE;
        dci.pNext = &pqf;
        dci.enabledExtensionCount = 1;
        dci.ppEnabledExtensionNames = &perfExt;
    }
    check(vkCreateDevice(c->phys_, &dci, nullptr, &c->dev_), "vkCreateDevice");
    vkGetDeviceQueue(c->dev_, c->qfam_, 0, &c->queue_);
    return c;
}

Context::~Context() {
    if (dev_) {
        vkDeviceWaitIdle(dev_);   // the last user is gone: nobody submits now
        vkDestroyDevice(dev_, nullptr);
    }
    if (inst_) vkDestroyInstance(inst_, nullptr);
}

Context::Buffer Context::makeBuffer(VkDeviceSize size) {
    Buffer b;
    b.size = size;
    VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bci.size = size;
    bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    check(vkCreateBuffer(dev_, &bci, nullptr, &b.buf), "vkCreateBuffer");
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(dev_, b.buf, &req);
    if (!(req.memoryTypeBits & (1u << memType_))) {
        vkDestroyBuffer(dev_, b.buf, nullptr);
        throw std::runtime_error("memory type not usable for buffer");
    }
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mai.allocationSize = req.size;
    mai.memoryTypeIndex = memType_;
    try {
        check(vkAllocateMemory(dev_, &mai, nullptr, &b.mem), "vkAllocateMemory");
        check(vkBindBufferMemory(dev_, b.buf, b.mem, 0), "vkBindBufferMemory");
        check(vkMapMemory(dev_, b.mem, 0, VK_WHOLE_SIZE, 0, &b.map), "vkMapMemory");
    } catch (...) {
        freeBuffer(b);
        throw;
    }
    return b;
}

void Context::freeBuffer(Buffer& b) {
    if (b.map) vkUnmapMemory(dev_, b.mem);
    if (b.buf) vkDestroyBuffer(dev_, b.buf, nullptr);
    if (b.mem) vkFreeMemory(dev_, b.mem, nullptr);
    b = {};
}

PyramidPool::~PyramidPool() {
    for (auto& b : free_) ctx_->freeBuffer(b);
}

std::shared_ptr<Pyramid> PyramidPool::acquire() {
    Context::Buffer b;
    {
        std::lock_guard<std::mutex> lock(m_);
        if (!free_.empty()) {
            b = free_.back();
            free_.pop_back();
        }
    }
    if (!b.buf) {
        b = ctx_->makeBuffer(bytes_);
        std::lock_guard<std::mutex> lock(m_);
        ++allocated_;
    }
    auto* p = new Pyramid();
    p->ctx = ctx_;
    p->buffer = b.buf;
    p->map = static_cast<const uint8_t*>(b.map);
    p->bytes = bytes_;
    std::shared_ptr<PyramidPool> self = shared_from_this();
    return std::shared_ptr<Pyramid>(p, [self, b](Pyramid* q) {
        delete q;
        std::lock_guard<std::mutex> lock(self->m_);
        self->free_.push_back(b);
    });
}

size_t PyramidPool::allocated() const {
    std::lock_guard<std::mutex> lock(m_);
    return allocated_;
}

}  // namespace gpu
#endif  // ORB_HAVE_VULKAN
