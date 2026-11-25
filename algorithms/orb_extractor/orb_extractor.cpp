// orb_extractor: ORB features (ORB-SLAM3 conventions) of an image, on the GPU (Vulkan compute,
// vk/) or on the CPU (orb_cpu.hpp). Both give the same keypoints and descriptors for the same image
// and parameters; the CPU one is the fallback when there is no usable GPU.
//
//   image    : fw::Image, mono8 (or any CV_8UC1), bgr8 / rgb8 / bgra8 / rgba8 (converted to gray)
//   params   : OrbExtractorParams (latest; optional, every field has a default). A change rebuilds
//              the extractor from the next image on.
//   gpu      : gpu::Device (resource, read), the box's GPU (types/gpu/device.hpp): shared with the other
//              GPU blocks (orb_matcher reads the GPU pyramids on it), which run at the same time (the device
//              locks its queue for each submission only). Taken on every image, also with the
//              CPU backend (the port is the same; nothing runs on the GPU then).
//   features : orb::Features, keypoints in full-resolution coordinates (octave = level) and
//              N x 32 CV_8U descriptors (cv::ORB bit layout), same stamp as the image; the image size,
//              the levels' scale factors and (params keep_pyramid) the pyramid without a CPU copy:
//              GPU: orb::Features::gpu_pyramid (copied on the GPU), CPU: orb::Features::pyramid (shared)
//
// backend auto: the GPU, set up on the first image (~0.1-0.2 s); if that fails, or the GPU fails
// later (e.g. device lost), the CPU from then on, reported as degraded. gpu: GPU only (a failure is
// reported as error and the frame is skipped). cpu: CPU only. Without Vulkan at build time
// (CMakeLists.txt, ORB_HAVE_VULKAN) there is only the CPU.
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fw/fw.hpp"
#include "fw/ros/msgs/image.hpp"
#include "algorithms/orb_extractor/orb_cpu.hpp"
#include "params/orb_extractor_params.hpp"
#include "types/gpu/device.hpp"
#include "types/orb/features.hpp"
#ifdef ORB_HAVE_VULKAN
#include "algorithms/orb_extractor/vk/orb_vk_slam.hpp"
#endif

namespace {

orb_cpu::Config to_config(const OrbExtractorParams& p) {
    orb_cpu::Config c;
    c.nfeatures = p.nfeatures;
    c.scale_factor = (float)p.scale_factor;
    c.nlevels = p.nlevels;
    c.ini_th_fast = p.ini_th_fast;
    c.min_th_fast = p.min_th_fast;
    c.per_cell_threshold = p.per_cell_threshold;
    c.distribute = p.distribute;
    c.cells_per_feature = (float)p.cells_per_feature;
    c.abort_corner_fraction = (float)p.abort_corner_fraction;
    c.edge_threshold = p.edge_threshold;
    return c;
}

#ifdef ORB_HAVE_VULKAN
orbvk::ExtractorConfig to_gpu_config(const orb_cpu::Config& c) {
    orbvk::ExtractorConfig g;
    g.nFeatures = c.nfeatures;
    g.scaleFactor = c.scale_factor;
    g.nLevels = c.nlevels;
    g.iniThFAST = c.ini_th_fast;
    g.minThFAST = c.min_th_fast;
    g.cellThreshold = c.per_cell_threshold;
    // A fixed threshold as the CPU backend (no adaptation between frames, no retry), so both
    // backends give the same result for every frame.
    g.adaptiveThreshold = false;
    g.retryBelow = 0.f;
    g.edgeThreshold = c.edge_threshold;
    g.distribute = c.distribute;
    g.cellsPerFeature = c.cells_per_feature;
    g.abortCornerFraction = c.abort_corner_fraction;
    return g;
}
#endif

// 8-bit gray view (no copy) or conversion. Luma with OpenCV's fixed-point BT.601 weights.
const cv::Mat& to_gray(const fw::Image& im, cv::Mat& buf) {
    const cv::Mat& m = im.mat;
    if (m.type() == CV_8UC1) return m;
    if (m.depth() != CV_8U || (m.channels() != 3 && m.channels() != 4))
        throw std::invalid_argument("orb_extractor: unsupported image (encoding '" + im.encoding +
                                    "'); need mono8, bgr8, rgb8, bgra8 or rgba8");
    const bool rgb = im.encoding.rfind("rgb", 0) == 0;
    const int cn = m.channels(), ri = rgb ? 0 : 2, bi = rgb ? 2 : 0;
    buf.create(m.rows, m.cols, CV_8UC1);
    for (int y = 0; y < m.rows; ++y) {
        const uint8_t* s = m.ptr<uint8_t>(y);
        uint8_t* d = buf.ptr<uint8_t>(y);
        for (int x = 0; x < m.cols; ++x, s += cn)
            d[x] = (uint8_t)((s[ri] * 4899 + s[1] * 9617 + s[bi] * 1868 + 8192) >> 14);
    }
    return buf;
}

struct OrbExtractor {
    orb_cpu::Config cfg;
    std::string backend;           // params.backend of the current extractors
    bool keep_pyramid = true;
    bool configured = false;
    std::unique_ptr<orb_cpu::Extractor> cpu;
#ifdef ORB_HAVE_VULKAN
    std::unique_ptr<orbvk::ORBextractorVk> gpu;
    bool gpu_failed = false;       // auto: the GPU failed, the CPU from now on
    std::vector<int> lap{0, 0};    // no lapping area
#endif
    cv::Mat gray;
    fw::Health health = fw::Health::ok;
    std::string health_msg;

    bool direct_report = true;     // false: OrbStereoExtractor reports both eyes' health as one

    void report(fw::Context& ctx, fw::Health h, const std::string& msg) {
        if (h == health && msg == health_msg) return;
        health = h;
        health_msg = msg;
        if (direct_report) ctx.report(h, msg);
    }

    void reconfigure(const OrbExtractorParams& p) {
        if (p.backend != "auto" && p.backend != "gpu" && p.backend != "cpu")
            throw std::invalid_argument("orb_extractor: backend must be auto, gpu or cpu, not '" + p.backend + "'");
        const orb_cpu::Config c = to_config(p);
        keep_pyramid = p.keep_pyramid;
        if (configured && c == cfg && p.backend == backend) return;
        auto fresh = std::make_unique<orb_cpu::Extractor>(c);   // validates the values (throws)
        cfg = c;
        backend = p.backend;
        configured = true;
        cpu.reset();
#ifdef ORB_HAVE_VULKAN
        gpu.reset();
        gpu_failed = false;
#endif
        if (backend == "cpu") cpu = std::move(fresh);
    }

    // Image size and scale factors; the pyramid (keep_pyramid) is set by the backend: no CPU copy either way.
    void set_levels(const cv::Mat& g, const std::vector<float>& scales, orb::Features& out) const {
        out.width = g.cols;
        out.height = g.rows;
        out.scale_factors = scales;
        out.pyramid.clear();
        out.gpu_pyramid.reset();
    }

#ifdef ORB_HAVE_VULKAN
    // GPU extraction; false when the CPU has to do this frame (auto after a GPU failure).
    bool extract_gpu(fw::Context& ctx, const cv::Mat& g, const ::gpu::Device& device, orb::Features& out) {
        if (gpu_failed) return false;
        try {
            if (!gpu) {
                gpu = std::make_unique<orbvk::ORBextractorVk>(to_gpu_config(cfg), device.context());
                gpu->Prepare(g.cols, g.rows);   // GPU set-up and a warm-up frame
            }
            (*gpu)(g, cv::Mat(), out.keypoints, out.descriptors, lap);
            set_levels(g, gpu->GetScaleFactors(), out);
            // copied on the GPU into a pooled buffer: the extractor's own is overwritten by the next frame
            if (keep_pyramid) out.gpu_pyramid = gpu->SnapshotPyramid();
            const orbvk::FrameInfo& fi = gpu->LastFrame();
            if (fi.aborted)
                report(ctx, fw::Health::degraded,
                       "abnormal frame: " + std::to_string(fi.corners) + " FAST corners, no keypoints");
            else if (fi.overflow)
                report(ctx, fw::Health::degraded, "a GPU list overflowed: keypoints of this frame are incomplete");
            else
                report(ctx, fw::Health::ok, "gpu: " + (gpu->gpu() ? gpu->gpu()->deviceName() : std::string()));
            return true;
        } catch (const std::exception& e) {
            gpu.reset();
            if (backend == "gpu") {
                report(ctx, fw::Health::error, std::string("GPU failed: ") + e.what());
                throw;
            }
            gpu_failed = true;
            report(ctx, fw::Health::degraded, std::string("GPU failed, using the CPU: ") + e.what());
            return false;
        }
    }
#endif

    void extract_cpu(fw::Context& ctx, const cv::Mat& g, orb::Features& out) {
        if (!cpu) cpu = std::make_unique<orb_cpu::Extractor>(cfg);
        cpu->extract(g, out.keypoints, out.descriptors, [&ctx](int begin, int end, const std::function<void(int)>& fn) {
            fw::parallel_for(ctx, (std::size_t)begin, (std::size_t)end, [&fn](std::size_t i) { fn((int)i); }, 1);
        });
        set_levels(g, cpu->scale_factors(), out);
        // shared, not copied: orb_cpu builds the next frame's in new images while this one is held
        if (keep_pyramid) out.pyramid = cpu->pyramid();
        const orb_cpu::FrameInfo& fi = cpu->last_frame();
        if (fi.aborted)
            report(ctx, fw::Health::degraded, "abnormal frame: " + std::to_string(fi.corners) + " FAST corners, no keypoints");
        else if (backend == "cpu")
            report(ctx, fw::Health::ok, "cpu");
#ifdef ORB_HAVE_VULKAN
        else if (!gpu_failed) report(ctx, fw::Health::ok, "cpu");
#else
        else
            report(ctx, fw::Health::degraded, "built without Vulkan: using the CPU");
#endif
    }

    void operator()(fw::Context& ctx, const fw::Image& image, const OrbExtractorParams& params,
                    fw::Read<::gpu::Device> device, orb::Features& features) {
        extract(ctx, image, params, *device, features);
    }

    // ctx: only for parallel_for unless direct_report (OrbStereoExtractor runs the eyes inside a parallel_for)
    void extract(fw::Context& ctx, const fw::Image& image, const OrbExtractorParams& params,
                 const ::gpu::Device& device, orb::Features& features) {
        features.extract_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch()).count();
        reconfigure(params);
        const cv::Mat& g = to_gray(image, gray);
#ifdef ORB_HAVE_VULKAN
        if (backend != "cpu" && extract_gpu(ctx, g, device, features)) return;
#else
        (void)device;
        if (backend == "gpu") {
            report(ctx, fw::Health::error, "backend gpu: built without Vulkan");
            throw std::runtime_error("orb_extractor: backend gpu, but built without Vulkan");
        }
#endif
        extract_cpu(ctx, g, features);
    }
};

// Both eyes in one block: the images come in as a sync pair, so a frame is dropped for both eyes or for none
// (two OrbExtractor blocks with their own queues dropped different frames, and orb_matcher's sync lost the
// pair). The eyes run at the same time (each has its own GPU objects): one eye's CPU work (gray conversion,
// keypoint selection, descriptors) overlaps the other's shaders. With the CPU backend the eyes run one after
// the other, each on the THREADS budget (nested parallel_for).
struct OrbStereoExtractor {
    OrbExtractor eye[2];
    fw::Health health = fw::Health::ok;
    std::string health_msg;

    OrbStereoExtractor() {
        eye[0].direct_report = eye[1].direct_report = false;
    }

    void on_pair(fw::Context& ctx, const fw::Image& left, const fw::Image& right, const OrbExtractorParams& params,
                 fw::Read<::gpu::Device> device, orb::Features& left_features, orb::Features& right_features) {
        const ::gpu::Device& dev = *device;
        const fw::Image* images[2] = {&left, &right};
        orb::Features* out[2] = {&left_features, &right_features};
        fw::parallel_for(ctx, 0, 2, [&](std::size_t e) { eye[e].extract(ctx, *images[e], params, dev, *out[e]); }, 1);
        // the worse eye's health; the left one's message when they are equal
        const int w = int(eye[1].health) > int(eye[0].health) ? 1 : 0;
        const fw::Health h = eye[w].health;
        const std::string msg = (eye[0].health == eye[1].health && eye[0].health_msg == eye[1].health_msg)
                                    ? eye[0].health_msg
                                    : std::string(w ? "right: " : "left: ") + eye[w].health_msg;
        if (h == health && msg == health_msg) return;
        health = h;
        health_msg = msg;
        ctx.report(h, msg);
    }
};

}  // namespace

FW_REGISTER_ALGORITHM_CLASS(OrbExtractor,
    INPUTS(("image", fw::Image, trigger), ("params", OrbExtractorParams, latest)),
    RESOURCES(("gpu", gpu::Device, read)),
    OUTPUTS(("features", orb::Features)),
    OPTIONS(THREADS(1)));

FW_REGISTER_ALGORITHM_CLASS(OrbStereoExtractor,
    INPUTS(("params", OrbExtractorParams, latest)),
    RESOURCES(("gpu", gpu::Device, read)),
    OUTPUTS(("left_features", orb::Features), ("right_features", orb::Features)),
    ON_SYNC((("left", fw::Image), ("right", fw::Image)), &OrbStereoExtractor::on_pair, USES("params", "gpu"),
            EMITS("left_features", "right_features")),
    OPTIONS(SYNC_SLOP(1ms), THREADS(2)));   // one thread per eye
