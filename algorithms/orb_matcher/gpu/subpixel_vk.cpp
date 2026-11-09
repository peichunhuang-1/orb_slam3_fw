// Compiled only with Vulkan (ORB_HAVE_VULKAN), see subpixel_vk.hpp.
#ifdef ORB_HAVE_VULKAN
#include "algorithms/orb_matcher/gpu/subpixel_vk.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include "spv_subpixel.h"

namespace orb_matcher {

namespace {

void check(VkResult r, const char* what) {
    if (r != VK_SUCCESS) {
        char m[160];
        std::snprintf(m, sizeof m, "SubpixelVk: %s failed (VkResult %d)", what, (int)r);
        throw std::runtime_error(m);
    }
}

// Must match shaders/subpixel.comp
constexpr int kLevels = 16;
constexpr size_t kHeaderBytes = 2 * kLevels * 4 * sizeof(int32_t);
struct GpuJob {
    float a[4];     // left.xy, dirL.xy
    float b[4];     // right.xy, dirR.xy
    int32_t c[4];   // level
};
static_assert(sizeof(GpuJob) == 48, "Job layout of subpixel.comp");
struct Push {
    int32_t n, w, L, fit;
};

}  // namespace

SubpixelVk::SubpixelVk(std::shared_ptr<gpu::Context> ctx, int half_window, int range, SubpixelFit fit)
    : ctx_(std::move(ctx)), w_(half_window), L_(range), fit_(fit) {
    if (!ctx_) throw std::invalid_argument("SubpixelVk: no GPU context");
    if (!supports(w_, L_))
        throw std::invalid_argument("SubpixelVk: half window must be 1..7 and range 1..8 (the shader's shared memory)");
    dev_ = ctx_->device();
    try {
        VkDescriptorSetLayoutBinding b[4];
        for (uint32_t i = 0; i < 4; ++i)
            b[i] = {i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
        VkDescriptorSetLayoutCreateInfo dl{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        dl.bindingCount = 4;
        dl.pBindings = b;
        check(vkCreateDescriptorSetLayout(dev_, &dl, nullptr, &dsl_), "vkCreateDescriptorSetLayout");
        VkPushConstantRange pcr{VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Push)};
        VkPipelineLayoutCreateInfo pl{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        pl.setLayoutCount = 1;
        pl.pSetLayouts = &dsl_;
        pl.pushConstantRangeCount = 1;
        pl.pPushConstantRanges = &pcr;
        check(vkCreatePipelineLayout(dev_, &pl, nullptr, &layout_), "vkCreatePipelineLayout");
        VkShaderModuleCreateInfo smi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        smi.codeSize = sizeof spv_subpixel;
        smi.pCode = spv_subpixel;
        VkShaderModule sm;
        check(vkCreateShaderModule(dev_, &smi, nullptr, &sm), "vkCreateShaderModule");
        VkComputePipelineCreateInfo cp{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        cp.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_COMPUTE_BIT, sm,
                    "main", nullptr};
        cp.layout = layout_;
        const VkResult r = vkCreateComputePipelines(dev_, VK_NULL_HANDLE, 1, &cp, nullptr, &pipe_);
        vkDestroyShaderModule(dev_, sm, nullptr);
        check(r, "vkCreateComputePipelines");
        VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4};
        VkDescriptorPoolCreateInfo dp{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        dp.maxSets = 1;
        dp.poolSizeCount = 1;
        dp.pPoolSizes = &ps;
        check(vkCreateDescriptorPool(dev_, &dp, nullptr, &dpool_), "vkCreateDescriptorPool");
        VkDescriptorSetAllocateInfo da{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        da.descriptorPool = dpool_;
        da.descriptorSetCount = 1;
        da.pSetLayouts = &dsl_;
        check(vkAllocateDescriptorSets(dev_, &da, &ds_), "vkAllocateDescriptorSets");
        VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        cpi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        cpi.queueFamilyIndex = ctx_->queueFamily();
        check(vkCreateCommandPool(dev_, &cpi, nullptr, &cpool_), "vkCreateCommandPool");
        VkCommandBufferAllocateInfo cba{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cba.commandPool = cpool_;
        cba.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cba.commandBufferCount = 1;
        check(vkAllocateCommandBuffers(dev_, &cba, &cmd_), "vkAllocateCommandBuffers");
        VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        check(vkCreateFence(dev_, &fci, nullptr, &fence_), "vkCreateFence");
        ensure_capacity(512);
    } catch (...) {
        release();
        throw;
    }
}

SubpixelVk::~SubpixelVk() { release(); }

void SubpixelVk::release() {
    if (!dev_) return;
    // refine() waits for its submission: nothing in flight
    if (fence_) vkDestroyFence(dev_, fence_, nullptr);
    if (cpool_) vkDestroyCommandPool(dev_, cpool_, nullptr);
    if (dpool_) vkDestroyDescriptorPool(dev_, dpool_, nullptr);
    if (pipe_) vkDestroyPipeline(dev_, pipe_, nullptr);
    if (layout_) vkDestroyPipelineLayout(dev_, layout_, nullptr);
    if (dsl_) vkDestroyDescriptorSetLayout(dev_, dsl_, nullptr);
    ctx_->freeBuffer(jobs_);
    ctx_->freeBuffer(out_);
    fence_ = VK_NULL_HANDLE;
    cpool_ = VK_NULL_HANDLE;
    dpool_ = VK_NULL_HANDLE;
    pipe_ = VK_NULL_HANDLE;
    layout_ = VK_NULL_HANDLE;
    dsl_ = VK_NULL_HANDLE;
    dev_ = VK_NULL_HANDLE;
}

void SubpixelVk::ensure_capacity(int n) {
    if (n <= capacity_) return;
    const int cap = std::max(n, 2 * capacity_);
    ctx_->freeBuffer(jobs_);
    ctx_->freeBuffer(out_);
    jobs_ = ctx_->makeBuffer(kHeaderBytes + size_t(cap) * sizeof(GpuJob));
    out_ = ctx_->makeBuffer(size_t(cap) * 2 * sizeof(uint32_t));
    capacity_ = cap;
}

void SubpixelVk::set_pyramids(std::shared_ptr<const gpu::Pyramid> left, std::shared_ptr<const gpu::Pyramid> right) {
    if (!left || !right) throw std::invalid_argument("SubpixelVk::set_pyramids: no pyramid");
    if (left->ctx.get() != ctx_.get() || right->ctx.get() != ctx_.get())
        throw std::invalid_argument("SubpixelVk::set_pyramids: pyramid of another GPU context");
    left_ = std::move(left);
    right_ = std::move(right);
}

void SubpixelVk::refine(const SubpixelJob* jobs, int n, float* offset, uint8_t* ok) {
    if (n <= 0) return;
    if (!left_ || !right_) throw std::logic_error("SubpixelVk::refine: set_pyramids first");
    ensure_capacity(n);

    // header (level geometry) and jobs, written straight into the mapped buffer
    int32_t geo[2][kLevels][4] = {};
    const gpu::Pyramid* pyr[2] = {left_.get(), right_.get()};
    for (int v = 0; v < 2; ++v)
        for (size_t l = 0; l < pyr[v]->levels.size() && l < size_t(kLevels); ++l) {
            const gpu::Level& g = pyr[v]->levels[l];
            geo[v][l][0] = g.w;
            geo[v][l][1] = g.h;
            geo[v][l][2] = g.stride;
            geo[v][l][3] = g.goff;
        }
    uint8_t* base = static_cast<uint8_t*>(jobs_.map);
    std::memcpy(base, geo, kHeaderBytes);
    GpuJob* gj = reinterpret_cast<GpuJob*>(base + kHeaderBytes);
    for (int k = 0; k < n; ++k) {
        const SubpixelJob& j = jobs[k];
        const GpuJob g{{j.left.x(), j.left.y(), j.dirL.x(), j.dirL.y()},
                       {j.right.x(), j.right.y(), j.dirR.x(), j.dirR.y()},
                       {j.level, 0, 0, 0}};
        std::memcpy(&gj[k], &g, sizeof g);
    }

    // the descriptor set is not in use: the previous refine() waited for its submission
    VkDescriptorBufferInfo bi[4] = {{left_->buffer, 0, VK_WHOLE_SIZE},
                                    {right_->buffer, 0, VK_WHOLE_SIZE},
                                    {jobs_.buf, 0, VK_WHOLE_SIZE},
                                    {out_.buf, 0, VK_WHOLE_SIZE}};
    VkWriteDescriptorSet wr[4];
    for (uint32_t i = 0; i < 4; ++i) {
        wr[i] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        wr[i].dstSet = ds_;
        wr[i].dstBinding = i;
        wr[i].descriptorCount = 1;
        wr[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        wr[i].pBufferInfo = &bi[i];
    }
    vkUpdateDescriptorSets(dev_, 4, wr, 0, nullptr);

    check(vkResetCommandBuffer(cmd_, 0), "vkResetCommandBuffer");
    VkCommandBufferBeginInfo bb{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bb.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    check(vkBeginCommandBuffer(cmd_, &bb), "vkBeginCommandBuffer");
    // the pyramid copies (transfer, earlier submissions on the queue) and the host's job list
    VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    mb.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT | VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_HOST_WRITE_BIT;
    mb.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_TRANSFER_BIT | VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_HOST_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &mb, 0, nullptr, 0, nullptr);
    vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_COMPUTE, pipe_);
    vkCmdBindDescriptorSets(cmd_, VK_PIPELINE_BIND_POINT_COMPUTE, layout_, 0, 1, &ds_, 0, nullptr);
    const Push pc{n, w_, L_, fit_ == SubpixelFit::SadParabola ? 0 : fit_ == SubpixelFit::SadV ? 1 : 2};
    vkCmdPushConstants(cmd_, layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof pc, &pc);
    vkCmdDispatch(cmd_, uint32_t(n), 1, 1);
    mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    mb.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
    vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &mb, 0, nullptr, 0,
                         nullptr);
    check(vkEndCommandBuffer(cmd_), "vkEndCommandBuffer");
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cmd_;
    check(ctx_->submit(si, fence_), "vkQueueSubmit");
    check(vkWaitForFences(dev_, 1, &fence_, VK_TRUE, UINT64_MAX), "vkWaitForFences");
    check(vkResetFences(dev_, 1, &fence_), "vkResetFences");

    // one read of the results (uncached memory: one block, not element by element)
    std::vector<uint32_t>& res = results_;
    res.resize(size_t(n) * 2);
    std::memcpy(res.data(), out_.map, res.size() * sizeof(uint32_t));
    for (int k = 0; k < n; ++k) {
        ok[k] = res[2 * k] != 0;
        std::memcpy(&offset[k], &res[2 * k + 1], sizeof(float));
    }
}

}  // namespace orb_matcher
#endif  // ORB_HAVE_VULKAN
