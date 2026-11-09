#pragma once
// Sub-pixel refinement on the GPU (shaders/subpixel.comp): a SubpixelBackend for StereoMatcher that runs
// the searches on the pyramids orb_extractor left on the GPU (orb::Features::gpu_pyramid), so the pixels
// never go through the CPU. One submission per stereo pair (all matches at once); the CPU only writes the
// job list and reads one offset per match.
//
// Results agree with the CPU version (SubpixelRefiner) up to float rounding: sums and grid positions are
// computed in another order, so an offset can differ by ~1e-4 px, and a near tie between two positions
// can go either way (orb_matcher_subpixel_gpu_test).
// Compiled only with Vulkan (ORB_HAVE_VULKAN).
#ifdef ORB_HAVE_VULKAN
#include <cstdint>
#include <memory>
#include <vector>

#include "types/gpu/context.hpp"
#include "algorithms/orb_matcher/stereo_matcher.hpp"

namespace orb_matcher {

class SubpixelVk : public SubpixelBackend {
public:
    // Window and range the shader handles (its shared memory is sized for them).
    static bool supports(int half_window, int range) {
        return half_window >= 1 && half_window <= 7 && range >= 1 && range <= 8;
    }
    // Throws std::invalid_argument for an unsupported window / range, std::runtime_error for a Vulkan error.
    SubpixelVk(std::shared_ptr<gpu::Context> ctx, int half_window, int range, SubpixelFit fit);
    ~SubpixelVk() override;
    SubpixelVk(const SubpixelVk&) = delete;
    SubpixelVk& operator=(const SubpixelVk&) = delete;

    // The two views' pyramids for the next refine() calls; both on this context.
    void set_pyramids(std::shared_ptr<const gpu::Pyramid> left, std::shared_ptr<const gpu::Pyramid> right);
    void clear_pyramids() { left_.reset(); right_.reset(); }   // their buffers go back to the extractor's pool
    void refine(const SubpixelJob* jobs, int n, float* offset, uint8_t* ok) override;

    const gpu::Context* context() const { return ctx_.get(); }
    int half_window() const { return w_; }
    int range() const { return L_; }
    SubpixelFit fit() const { return fit_; }

private:
    void ensure_capacity(int n);
    void release();

    std::shared_ptr<gpu::Context> ctx_;
    int w_, L_;
    SubpixelFit fit_;
    std::shared_ptr<const gpu::Pyramid> left_, right_;
    VkDevice dev_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout dsl_ = VK_NULL_HANDLE;
    VkPipelineLayout layout_ = VK_NULL_HANDLE;
    VkPipeline pipe_ = VK_NULL_HANDLE;
    VkDescriptorPool dpool_ = VK_NULL_HANDLE;
    VkDescriptorSet ds_ = VK_NULL_HANDLE;
    VkCommandPool cpool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    gpu::Context::Buffer jobs_, out_;
    int capacity_ = 0;
    std::vector<uint32_t> results_;
};

}  // namespace orb_matcher
#endif  // ORB_HAVE_VULKAN
