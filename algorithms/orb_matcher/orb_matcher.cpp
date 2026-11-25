// orb_matcher: stereo matching of a left / right orb::Features pair along the epipolar great circles of a
// Kannala-Brandt (KB8) rig, closed-form triangulation and sub-pixel refinement along the epipolar line
// (stereo_matcher.hpp, docs/ALGORITHM.md). Replaces ORB-SLAM3's fisheye stereo matching (brute force in
// the lapping area); the output has the fields of its fisheye stereo Frame.
//
//   left, right : orb::Features (sync), from orb_extractor. Sub-pixel refinement reads their pyramids
//                 (orb_extractor keep_pyramid, on by default); without them it is skipped (degraded).
//                 GPU pyramids (orb::Features::gpu_pyramid, the extractor on the GPU) are refined on the
//                 GPU (gpu/subpixel_vk.hpp: no pixel goes through the CPU), CPU ones on the CPU;
//                 params subpixel_device chooses (auto, gpu, cpu).
//   gpu         : gpu::Device (resource, read), the box's GPU (types/gpu/device.hpp), the one the
//                 extractors' GPU pyramids are on: the sub-pixel shader runs there. Taken on every pair; read,
//                 so the extractor goes on with the next pair meanwhile (the device locks its queue itself).
//   calib       : calib::StereoImuRig (latest), from euroc_mcap_driver. Each camera as KB8: a KB8
//                 (equidistant) calibration as is, radtan (plumb_bob) by a KB8 fit (rig_from_calib.hpp).
//                 Resized to the features' image size when that differs (same aspect ratio).
//   params      : OrbMatcherParams (latest; optional, every field has a default). A change rebuilds the
//                 matcher from the next pair on.
//   match       : orb::Match, stamp of the left features
//   stereo      : orb::StereoFrame, stamp of the left features: both views' keypoints, descriptors and bearings
//                 with the matches, for tracking (through imu_preintegration for stereo-inertial)
//
// At the end of the data it prints a summary: pairs, matches per pair, matching time, output rate (wall
// clock) and the latency from the earlier extraction start of a pair to its match (orb::MatchStats::latency_ms).
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "types/camera/rig_from_calib.hpp"
#include "algorithms/orb_matcher/stereo_matcher.hpp"
#ifdef ORB_HAVE_VULKAN
#include "algorithms/orb_matcher/gpu/subpixel_vk.hpp"
#endif
#include "fw/fw.hpp"
#include "params/orb_matcher_params.hpp"
#include "types/calib/stereo_imu_rig.hpp"
#include "types/gpu/device.hpp"
#include "types/orb/features.hpp"
#include "types/orb/match.hpp"
#include "types/orb/stereo_frame.hpp"

namespace {

using orb_matcher::StereoMatcherParams;
using orb_matcher::SubpixelFit;

StereoMatcherParams to_matcher_params(const OrbMatcherParams& p) {
    StereoMatcherParams m;
    m.th_dist = p.th_dist;
    m.ratio = float(p.ratio);
    m.octave_window = p.octave_window;
    m.sigma_px = float(p.sigma_px);
    m.gate_k = float(p.gate_k);
    m.min_depth = float(p.min_depth);
    m.max_depth = float(p.max_depth);
    m.min_epipole_deg = float(p.min_epipole_deg);
    m.min_parallax_deg = float(p.min_parallax_deg);
    m.chi2 = float(p.chi2);
    m.subpixel = p.subpixel;
    m.subpixel_half_window = p.subpixel_half_window;
    m.subpixel_range = p.subpixel_range;
    if (p.subpixel_fit == "ssd") m.subpixel_fit = SubpixelFit::SsdParabola;
    else if (p.subpixel_fit == "sad") m.subpixel_fit = SubpixelFit::SadParabola;
    else if (p.subpixel_fit == "sad_v") m.subpixel_fit = SubpixelFit::SadV;
    else throw std::invalid_argument("orb_matcher: subpixel_fit must be ssd, sad or sad_v, not '" + p.subpixel_fit + "'");
    return m;
}

bool same(const StereoMatcherParams& a, const StereoMatcherParams& b) {
    return a.th_dist == b.th_dist && a.ratio == b.ratio && a.octave_window == b.octave_window &&
           a.sigma_px == b.sigma_px && a.gate_k == b.gate_k && a.min_depth == b.min_depth &&
           a.max_depth == b.max_depth && a.min_epipole_deg == b.min_epipole_deg &&
           a.min_parallax_deg == b.min_parallax_deg && a.chi2 == b.chi2 && a.subpixel == b.subpixel &&
           a.subpixel_half_window == b.subpixel_half_window && a.subpixel_range == b.subpixel_range &&
           a.subpixel_fit == b.subpixel_fit;
}

std::string fmt(const char* f, double a, double b = 0) {
    char buf[96];
    std::snprintf(buf, sizeof buf, f, a, b);
    return buf;
}

struct OrbMatcher {
    // calibration
    bool have_rig = false;
    fw::Time calib_stamp{};
    camera::StereoRig rig_calib;    // at the calibration's image size
    std::string model_note;              // how the cameras became KB8
    bool fit_poor = false;
    std::string fit_poor_msg;
    // matcher, rebuilt when the rig, the image size or the parameters change
    std::unique_ptr<orb_matcher::StereoMatcher> matcher;
    StereoMatcherParams mp;
    int width = 0, height = 0;
    orb_matcher::StereoMatches sm;       // reused: match() does not allocate after warm-up
    // sub-pixel refinement
#ifdef ORB_HAVE_VULKAN
    std::unique_ptr<orb_matcher::SubpixelVk> gpu_sub;
    std::shared_ptr<gpu::Context> gpu_failed_ctx;   // the GPU refinement failed on this context: CPU
    std::string gpu_failed_msg;
#endif
    std::vector<cv::Mat> mapped[2];      // CPU refinement on GPU pyramids: views of their (slow) mapping
    long sub_pairs[3] = {0, 0, 0};       // pairs refined on: none, CPU, GPU
    long sub_jobs = 0, sub_rejected = 0; // sub-pixel searches, and matches they dropped
    orb::MatchStats totals;              // summed over the pairs
    // health
    fw::Health health = fw::Health::ok;
    std::string health_msg;
    // summary
    long pairs = 0, matched = 0;
    double ms_sum = 0, ms_max = 0, sub_ms_sum = 0;
    std::vector<double> latencies;       // ms, per pair with known extraction start
    std::int64_t first_ns = 0, last_ns = 0;   // steady clock of the first / last match

    static std::int64_t steady_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void report(fw::Context& ctx, fw::Health h, const std::string& msg) {
        if (h == health && msg == health_msg) return;
        health = h;
        health_msg = msg;
        ctx.report(h, msg);
    }

    void update_calib(fw::Context& ctx, const Msg<calib::StereoImuRig>& cal, double max_fit_px) {
        if (have_rig && cal.stamp() == calib_stamp) return;
        camera::ModelFit fl, fr;
        try {
            rig_calib = camera::stereoRigFromCalib(*cal, &fl, &fr);
        } catch (const std::exception& e) {
            report(ctx, fw::Health::error, std::string("calibration: ") + e.what());
            throw;
        }
        have_rig = true;
        calib_stamp = cal.stamp();
        matcher.reset();
        model_note = "kb8";
        if (fl.fitted || fr.fitted)
            model_note = "kb8 fitted to " + cal->left.distortion_model + " (rms " + fmt("%.2f / %.2f px", fl.rms_px, fr.rms_px) +
                         ", p99 " + fmt("%.2f / %.2f px)", fl.p99_px, fr.p99_px);
        model_note += ", baseline " + fmt("%.1f mm", rig_calib.baseline() * 1e3);
        fit_poor = std::max(fl.p99_px, fr.p99_px) > max_fit_px;
        fit_poor_msg = "the KB8 fit is off by " + fmt("%.2f px (p99) > max_fit_error_px %.2f", std::max(fl.p99_px, fr.p99_px), max_fit_px);
        std::printf("orb_matcher: %s\n", model_note.c_str());
        std::fflush(stdout);
    }

    void on_pair(fw::Context& ctx, const orb::Features& left, const orb::Features& right,
                 Msg<calib::StereoImuRig> cal, const OrbMatcherParams& params, fw::Read<::gpu::Device> device_res,
                 orb::Match& out, orb::StereoFrame& frame) {
        update_calib(ctx, cal, params.max_fit_error_px);

        // image size: the features' (the extractor may run on a resized image), else the calibration's
        int w = left.width, h = left.height;
        if (w == 0 || h == 0) {
            w = rig_calib.width();
            h = rig_calib.height();
        }
        if ((right.width || right.height) && (right.width != w || right.height != h))
            throw std::invalid_argument("orb_matcher: left image " + std::to_string(w) + "x" + std::to_string(h) +
                                        ", right " + std::to_string(right.width) + "x" + std::to_string(right.height));
        if (left.scale_factors.empty())
            throw std::invalid_argument("orb_matcher: the features have no scale_factors (orb_extractor fills them)");

        StereoMatcherParams want = to_matcher_params(params);
        const std::string& device = params.subpixel_device;
        if (device != "auto" && device != "gpu" && device != "cpu")
            throw std::invalid_argument("orb_matcher: subpixel_device must be auto, gpu or cpu, not '" + device + "'");

        // where the sub-pixel refinement runs: GPU (GPU pyramids), CPU (CPU pyramids, or GPU ones read
        // through their mapping), or not at all (no pyramids)
        enum Sub { sub_none, sub_cpu, sub_gpu } sub = sub_none;
        std::string sub_note;
        const bool mats = !left.pyramid.empty() && !right.pyramid.empty();
        bool gpus = false;
#ifdef ORB_HAVE_VULKAN
        gpus = left.gpu_pyramid && right.gpu_pyramid && left.gpu_pyramid->ctx == right.gpu_pyramid->ctx;
        // the shader runs on the box's device: the pyramids must be on it (the graph connects both to it)
        const bool on_device = gpus && device_res->available() && device_res->context() == left.gpu_pyramid->ctx;
#else
        (void)device_res;
#endif
        if (want.subpixel) {
            std::string why_not_gpu = device == "cpu" ? "subpixel_device cpu"
                                                      : "the features have no GPU pyramids (orb_extractor on the CPU)";
#ifdef ORB_HAVE_VULKAN
            if (gpus && !on_device)
                why_not_gpu = device_res->available() ? "the features' GPU pyramids are on another gpu/Device than this block's"
                                                      : "gpu/Device: " + device_res->error();
            if (device != "cpu" && on_device && ensure_gpu(left.gpu_pyramid->ctx, want, why_not_gpu)) sub = sub_gpu;
#else
            why_not_gpu = "built without Vulkan";
#endif
            if (sub == sub_none && device == "gpu") {
                report(ctx, fw::Health::error, "subpixel_device gpu: " + why_not_gpu);
                throw std::runtime_error("orb_matcher: subpixel_device gpu: " + why_not_gpu);
            }
            if (sub == sub_none && (mats || gpus)) {
                sub = sub_cpu;
                if (!mats) sub_note = "GPU pyramids read by the CPU (slow): " + why_not_gpu;
            }
        }
        const bool no_pyramids = want.subpixel && sub == sub_none;
        if (no_pyramids) want.subpixel = false;   // match without the refinement rather than not at all

        if (!matcher || w != width || h != height || !same(want, mp)) {
            const camera::StereoRig rig =
                (w == rig_calib.width() && h == rig_calib.height()) ? rig_calib : rig_calib.resized(w, h);
            matcher = std::make_unique<orb_matcher::StereoMatcher>(
                rig, want, int(std::max(left.keypoints.size(), right.keypoints.size())));
            mp = want;
            width = w;
            height = h;
        }

        orb_matcher::ViewFeatures vl, vr;
        auto view = [&](int side, const orb::Features& f, orb_matcher::ViewFeatures& v) {
            if (!f.keypoints.empty() &&
                (f.descriptors.type() != CV_8UC1 || f.descriptors.cols != 32 || f.descriptors.rows != int(f.keypoints.size())))
                throw std::invalid_argument("orb_matcher: descriptors must be N x 32 CV_8U, one row per keypoint");
            v.keypoints = f.keypoints.data();
            v.n = int(f.keypoints.size());
            v.descriptors = f.descriptors.empty() ? nullptr : f.descriptors.ptr();
            v.stride = f.descriptors.empty() ? 32 : f.descriptors.step[0];
            std::vector<cv::Mat>& m = mapped[side];
            m.clear();
#ifdef ORB_HAVE_VULKAN
            if (sub == sub_cpu && f.pyramid.empty() && f.gpu_pyramid)
                for (const gpu::Level& g : f.gpu_pyramid->levels)
                    m.emplace_back(g.h, g.w, CV_8UC1, const_cast<uint8_t*>(f.gpu_pyramid->map) + g.goff, size_t(g.stride));
#endif
            const std::vector<cv::Mat>& pyr = m.empty() ? f.pyramid : m;
            v.pyramid = pyr.empty() ? nullptr : pyr.data();
            v.pyramid_levels = int(pyr.size());
        };
        view(0, left, vl);
        view(1, right, vr);
        orb_matcher::SubpixelBackend* backend = nullptr;
#ifdef ORB_HAVE_VULKAN
        if (sub == sub_gpu) {
            gpu_sub->set_pyramids(left.gpu_pyramid, right.gpu_pyramid);
            backend = gpu_sub.get();
        }
        try {
            matcher->match(vl, vr, left.scale_factors, sm, backend);
        } catch (const std::exception& e) {
            if (gpu_sub) gpu_sub->clear_pyramids();
            if (sub != sub_gpu || device == "gpu") throw;
            // the GPU failed: this pair and the next ones on the CPU (GPU pyramids read through the mapping)
            gpu_failed_ctx = left.gpu_pyramid->ctx;
            gpu_failed_msg = e.what();
            gpu_sub.reset();
            sub = sub_cpu;
            view(0, left, vl);
            view(1, right, vr);
            matcher->match(vl, vr, left.scale_factors, sm);
        }
        if (gpu_sub) gpu_sub->clear_pyramids();   // the buffers go back to the extractor's pool
#else
        matcher->match(vl, vr, left.scale_factors, sm, backend);
#endif
        const orb_matcher::StereoMatchStats& s_ = sm.stats;
        for (cv::Mat& v : mapped[0]) v.release();
        for (cv::Mat& v : mapped[1]) v.release();
        ++sub_pairs[sub];
        totals.candidates += s_.candidates;
        totals.hamming_evals += s_.hamming_evals;
        totals.rejected_dist += s_.rejected_dist;
        totals.rejected_ratio += s_.rejected_ratio;
        totals.rejected_unique += s_.rejected_unique;
        totals.rejected_parallax += s_.rejected_parallax;
        totals.rejected_reproj += s_.rejected_reproj;
        sub_rejected += sm.stats.rejected_subpixel;
        if (sub != sub_none) sub_jobs += sm.stats.matched + sm.stats.rejected_subpixel;

        out.left_to_right = sm.left_to_right;
        out.right_to_left = sm.right_to_left;
        out.points = sm.points;
        out.depth = sm.depth;
        out.right_uv = sm.right_uv;
        out.bearing_left = sm.bearing[0];
        out.bearing_right = sm.bearing[1];
        out.matches.clear();
        out.matches.reserve(sm.stats.matched);
        for (int i = 0; i < int(sm.left_to_right.size()); ++i)
            if (sm.left_to_right[i] >= 0) out.matches.emplace_back(i, sm.left_to_right[i], float(sm.distance[i]));
        const orb_matcher::StereoMatchStats& s = sm.stats;
        out.stats = orb::MatchStats{s.left,           s.right,           s.candidates,      s.hamming_evals,
                                    s.matched,        s.rejected_dist,   s.rejected_ratio,  s.rejected_unique,
                                    s.rejected_parallax, s.rejected_reproj, s.rejected_subpixel, s.ms,
                                    s.subpixel_ms};
        frame.keypoints[0] = left.keypoints;
        frame.keypoints[1] = right.keypoints;
        frame.descriptors[0] = left.descriptors;   // shared, not copied: the features are not modified
        frame.descriptors[1] = right.descriptors;
        frame.bearings[0] = sm.bearing[0];
        frame.bearings[1] = sm.bearing[1];
        frame.width = w;
        frame.height = h;
        frame.scale_factors = left.scale_factors;
        frame.left_to_right = sm.left_to_right;
        frame.right_to_left = sm.right_to_left;
        frame.points = sm.points;
        frame.depth = sm.depth;

        const std::int64_t now = steady_ns();
        const std::int64_t start = (left.extract_start_ns && right.extract_start_ns)
                                       ? std::min(left.extract_start_ns, right.extract_start_ns)
                                       : std::max(left.extract_start_ns, right.extract_start_ns);
        if (start > 0) {
            out.stats.latency_ms = double(now - start) * 1e-6;
            latencies.push_back(out.stats.latency_ms);
        }
        if (!first_ns) first_ns = now;
        last_ns = now;

        ++pairs;
        matched += s.matched;
        ms_sum += s.ms;
        sub_ms_sum += s.subpixel_ms;
        ms_max = std::max(ms_max, s.ms);

        const std::string where = sub == sub_gpu ? ", sub-pixel on the GPU" : sub == sub_cpu ? ", sub-pixel on the CPU" : "";
        if (no_pyramids)
            report(ctx, fw::Health::degraded, "no pyramids in the features (orb_extractor keep_pyramid): sub-pixel refinement off");
#ifdef ORB_HAVE_VULKAN
        else if (sub == sub_cpu && gpu_failed_ctx && gpus && gpu_failed_ctx == left.gpu_pyramid->ctx)
            report(ctx, fw::Health::degraded, "GPU sub-pixel refinement failed, on the CPU: " + gpu_failed_msg);
#endif
        else if (fit_poor)
            report(ctx, fw::Health::degraded, fit_poor_msg);
        else
            report(ctx, fw::Health::ok, model_note + where + (sub_note.empty() ? "" : " (" + sub_note + ")"));
    }

#ifdef ORB_HAVE_VULKAN
    // The GPU refinement for this context and parameters; false (why) when it cannot run.
    bool ensure_gpu(const std::shared_ptr<gpu::Context>& gctx, const StereoMatcherParams& p, std::string& why) {
        if (gctx == gpu_failed_ctx) {
            why = "the GPU refinement failed: " + gpu_failed_msg;
            return false;
        }
        if (!orb_matcher::SubpixelVk::supports(p.subpixel_half_window, p.subpixel_range)) {
            why = "the GPU refinement supports subpixel_half_window <= 7 and subpixel_range <= 8";
            return false;
        }
        if (gpu_sub && gpu_sub->context() == gctx.get() && gpu_sub->half_window() == p.subpixel_half_window &&
            gpu_sub->range() == p.subpixel_range && gpu_sub->fit() == p.subpixel_fit)
            return true;
        try {
            gpu_sub.reset();
            gpu_sub = std::make_unique<orb_matcher::SubpixelVk>(gctx, p.subpixel_half_window, p.subpixel_range,
                                                                p.subpixel_fit);
            return true;
        } catch (const std::exception& e) {
            gpu_failed_ctx = gctx;
            gpu_failed_msg = e.what();
            why = "the GPU refinement failed: " + gpu_failed_msg;
            return false;
        }
    }
#endif

    void on_eos() {
        if (pairs == 0) {
            std::printf("orb_matcher: no stereo pairs\n");
        } else {
            std::printf("orb_matcher: %ld pairs, %.1f matches / pair, %.3f ms / pair (sub-pixel %.3f), max %.3f ms\n",
                        pairs, double(matched) / double(pairs), ms_sum / double(pairs), sub_ms_sum / double(pairs), ms_max);
            if (pairs > 1)
                std::printf("orb_matcher: output %.2f Hz (wall clock, first to last match over %.1f s)\n",
                            double(pairs - 1) / (double(last_ns - first_ns) * 1e-9), double(last_ns - first_ns) * 1e-9);
            const double np = double(pairs);
            std::printf("orb_matcher: per pair: %.1f hamming evaluations; rejected: distance %.1f, ratio %.1f, unique %.1f, "
                        "parallax %.1f, reprojection %.1f, sub-pixel %.1f\n",
                        totals.hamming_evals / np, totals.rejected_dist / np, totals.rejected_ratio / np,
                        totals.rejected_unique / np, totals.rejected_parallax / np, totals.rejected_reproj / np,
                        double(sub_rejected) / np);
            std::printf("orb_matcher: sub-pixel refinement on the GPU %ld pairs, on the CPU %ld, off %ld; dropped %.1f%% of "
                        "%ld matches\n",
                        sub_pairs[2], sub_pairs[1], sub_pairs[0], 100.0 * double(sub_rejected) / double(std::max(sub_jobs, 1L)),
                        sub_jobs);
            if (!latencies.empty()) {
                std::vector<double> v = latencies;
                std::sort(v.begin(), v.end());
                double sum = 0;
                for (double x : v) sum += x;
                auto q = [&](double f) { return v[size_t(f * double(v.size() - 1))]; };
                std::printf("orb_matcher: latency extraction start -> match: mean %.1f ms, median %.1f, p95 %.1f, max %.1f "
                            "(%zu pairs)\n",
                            sum / double(v.size()), q(0.5), q(0.95), v.back(), v.size());
            }
        }
        std::fflush(stdout);
    }
};

}  // namespace

FW_REGISTER_ALGORITHM_CLASS(OrbMatcher,
    INPUTS(("calib", calib::StereoImuRig, latest), ("params", OrbMatcherParams, latest)),
    OUTPUTS(("match", orb::Match), ("stereo", orb::StereoFrame)),
    RESOURCES(("gpu", gpu::Device, read)),
    ON_SYNC((("left", orb::Features), ("right", orb::Features)), &OrbMatcher::on_pair, USES("calib", "params", "gpu"),
            EMITS("match", "stereo")),
    ON_EOS(&OrbMatcher::on_eos),
    OPTIONS(SYNC_SLOP(1ms)));
