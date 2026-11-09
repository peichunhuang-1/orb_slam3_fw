// subpixel_gpu_test: orb_matcher's sub-pixel refinement on the GPU against the CPU version, and the GPU
// pyramid handover from orb_extractor.
//
//   orb_matcher_subpixel_gpu_test [--jobs N]
//
// 1. handover: ORBextractorVk::SnapshotPyramid equals the extractor's own pyramid byte for byte, stays the
//    same after the next frame overwrote the extractor's, and released buffers are reused
// 2. the same searches (random level, position, epipolar direction, shift; a textured stereo pair through
//    the GPU extractor) through SubpixelVk and SubpixelRefiner, for the three fits: the accept / reject
//    decision and the offset agree (up to float rounding; a near tie may pick the other position)
// 3. time of one pair's searches (337, the EuRoC mean) on both
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "algorithms/orb_extractor/orb_cpu.hpp"
#include "algorithms/orb_extractor/vk/orb_vk_slam.hpp"
#include "algorithms/orb_matcher/core/subpixel.hpp"
#include "algorithms/orb_matcher/gpu/subpixel_vk.hpp"
#include "types/gpu/context.hpp"

using namespace orb_matcher;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    std::printf("  %-78s %s\n", what.c_str(), ok ? "ok" : "FAIL");
    failures += !ok;
}

static cv::Mat texture(int w, int h, unsigned seed) {
    cv::Mat n(h, w, CV_32F);
    cv::RNG rng(seed);
    rng.fill(n, cv::RNG::NORMAL, 0, 1);
    cv::GaussianBlur(n, n, cv::Size(0, 0), 2.0);
    cv::Mat out;
    cv::normalize(n, n, 0, 1, cv::NORM_MINMAX);
    n.convertTo(out, CV_8U, 230, 12);
    return out;
}

// The snapshot's level l as a cv::Mat copy in normal memory
static cv::Mat level_of(const gpu::Pyramid& p, int l) {
    const gpu::Level& g = p.levels[l];
    return cv::Mat(g.h, g.w, CV_8UC1, const_cast<uint8_t*>(p.map) + g.goff, size_t(g.stride)).clone();
}

static bool same(const cv::Mat& a, const cv::Mat& b) {
    if (a.size() != b.size()) return false;
    for (int y = 0; y < a.rows; ++y)
        if (std::memcmp(a.ptr(y), b.ptr(y), size_t(a.cols))) return false;
    return true;
}

int main(int argc, char** argv) {
    int njobs = 20000;
    for (int i = 1; i + 1 < argc; ++i)
        if (std::string(argv[i]) == "--jobs") njobs = std::max(100, std::atoi(argv[++i]));
    const int W = 752, H = 480;
    const cv::Mat left = texture(W, H, 1);
    cv::Mat right;   // the left image 6.3 px to the left, plus noise: a disparity
    const cv::Mat shift = (cv::Mat_<double>(2, 3) << 1, 0, -6.3, 0, 1, 0);
    cv::warpAffine(left, right, shift, left.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
    {
        cv::Mat noise(H, W, CV_16S);
        cv::randn(noise, 0, 2);
        cv::Mat r16;
        right.convertTo(r16, CV_16S);
        r16 += noise;
        r16.convertTo(right, CV_8U);
    }

    orbvk::ExtractorConfig cfg;
    cfg.nFeatures = 1000;
    cfg.adaptiveThreshold = false;
    // one device for both, as the box's gpu/Device resource gives the blocks
    const std::shared_ptr<gpu::Context> ctx = gpu::Context::create();
    orbvk::ORBextractorVk exL(cfg, ctx), exR(cfg, ctx);
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    std::vector<int> lap{0, 0};

    std::printf("GPU pyramid handover\n");
    exL(left, cv::Mat(), kps, desc, lap);
    auto snapL = exL.SnapshotPyramid();
    exR(right, cv::Mat(), kps, desc, lap);
    auto snapR = exR.SnapshotPyramid();
    check(snapL->ctx == ctx && snapR->ctx == ctx, "both snapshots on the given device (" + ctx->deviceName() + ")");
    const int nl = int(snapL->levels.size());
    std::vector<cv::Mat> cpuL(nl), cpuR(nl);   // the pyramids in normal memory, for the CPU version
    bool eq = true;
    for (int l = 0; l < nl; ++l) {
        cpuL[l] = level_of(*snapL, l);
        cpuR[l] = level_of(*snapR, l);
        eq = eq && same(cpuL[l], exL.mvImagePyramid[l]) && same(cpuR[l], exR.mvImagePyramid[l]);
    }
    check(eq && nl == 8, "snapshot = the extractor's pyramid, byte for byte (" + std::to_string(nl) + " levels)");
    {
        // the CPU extractor (orb_cpu, same arithmetic as the shaders) builds the same pyramid
        orb_cpu::Config cc;
        orb_cpu::Extractor ce(cc);
        std::vector<cv::KeyPoint> ck;
        cv::Mat cd;
        ce.extract(left, ck, cd);
        const std::vector<cv::Mat>& cp = ce.pyramid();
        std::string diff;
        for (int l = 0; l < nl && l < int(cp.size()); ++l) {
            cv::Mat d;
            cv::absdiff(cpuL[l], cp[l], d);
            double mx;
            cv::minMaxLoc(d, nullptr, &mx);
            diff += " L" + std::to_string(l) + " " + std::to_string(int(mx));
        }
        check(int(cp.size()) == nl && same(cpuL[0], cp[0]) && same(cpuL[nl - 1], cp[nl - 1]),
              "GPU pyramid = orb_cpu pyramid (max |diff| per level:" + diff + ")");
    }
    {
        exL(right, cv::Mat(), kps, desc, lap);   // the extractor's pyramid now holds another image
        bool kept = true;
        for (int l = 0; l < nl; ++l) kept = kept && same(level_of(*snapL, l), cpuL[l]);
        check(kept && !same(exL.mvImagePyramid[0], cpuL[0]), "snapshot unchanged after the next frame");
        const size_t before = exL.gpu()->pyramidBuffers();
        for (int i = 0; i < 5; ++i) exL.SnapshotPyramid();   // each released at once
        check(exL.gpu()->pyramidBuffers() == before + 1, "released buffers are reused (" +
                                                             std::to_string(exL.gpu()->pyramidBuffers()) + " allocated)");
    }

    // Searches: random level, left position, epipolar direction (right one slightly turned), and the
    // right point near the true match (6.3 px at level 0, scaled) plus an error of up to 3 level px
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> uni(0.f, 1.f);
    std::vector<SubpixelJob> jobs(njobs);
    for (SubpixelJob& j : jobs) {
        j.level = int(uni(rng) * nl) % nl;
        const cv::Mat& im = cpuL[j.level];
        const float s = std::pow(1.2f, float(j.level));
        j.left = Eigen::Vector2f(uni(rng) * (im.cols - 1), uni(rng) * (im.rows - 1));   // some leave the image
        const float a = (uni(rng) - 0.5f) * 0.6f, b = a + (uni(rng) - 0.5f) * 0.05f;
        j.dirL = Eigen::Vector2f(-std::cos(a), -std::sin(a));   // towards smaller x (as the disparity)
        j.dirR = Eigen::Vector2f(-std::cos(b), -std::sin(b));
        j.right = j.left + Eigen::Vector2f(-6.3f / s, 0) + (uni(rng) - 0.5f) * 6.f * j.dirR;
    }
    std::vector<float> offCpu(njobs), offGpu(njobs);
    std::vector<uint8_t> okCpu(njobs), okGpu(njobs);

    for (SubpixelFit fit : {SubpixelFit::SsdParabola, SubpixelFit::SadParabola, SubpixelFit::SadV}) {
        const char* name = fit == SubpixelFit::SsdParabola ? "SSD parabola" : fit == SubpixelFit::SadParabola ? "SAD parabola" : "SAD V";
        std::printf("GPU vs CPU, %s, %d searches\n", name, njobs);
        SubpixelRefiner cpu;
        cpu.configure(5, 5, fit);
        SubpixelVk gpu(snapL->ctx, 5, 5, fit);
        gpu.set_pyramids(snapL, snapR);
        for (int k = 0; k < njobs; ++k) {
            const SubpixelJob& j = jobs[k];
            okCpu[k] = cpu.refine(cpuL[j.level], j.left, j.dirL, cpuR[j.level], j.right, j.dirR, offCpu[k]);
        }
        gpu.refine(jobs.data(), njobs, offGpu.data(), okGpu.data());
        int both = 0, disagree = 0, flips = 0, accepted = 0;
        std::vector<double> diffs;
        for (int k = 0; k < njobs; ++k) {
            accepted += okCpu[k];
            if (okCpu[k] != okGpu[k]) { ++disagree; continue; }
            if (!okCpu[k]) continue;
            ++both;
            const double d = std::abs(double(offCpu[k]) - offGpu[k]);
            if (d > 0.05) ++flips;   // another integer minimum: a near tie
            else diffs.push_back(d);
        }
        std::sort(diffs.begin(), diffs.end());
        const double p99 = diffs.empty() ? 0 : diffs[size_t(0.99 * (diffs.size() - 1))], mx = diffs.empty() ? 0 : diffs.back();
        char m[200];
        std::snprintf(m, sizeof m, "accept / reject agree: %d of %d differ (%.3f%%); CPU accepts %.1f%%", disagree, njobs,
                      100.0 * disagree / njobs, 100.0 * accepted / njobs);
        check(disagree <= njobs / 200, m);
        std::snprintf(m, sizeof m, "offset |GPU - CPU|: p99 %.2g px, max %.2g px (%d both accepted)", p99, mx, both);
        check(p99 < 1e-3 && mx < 1e-2, m);
        std::snprintf(m, sizeof m, "another minimum picked (near tie): %d (%.3f%%)", flips, 100.0 * flips / std::max(both, 1));
        check(flips <= both / 200, m);
    }

    std::printf("time of one pair's searches (337)\n");
    {
        const int n = 337, reps = 300;
        SubpixelRefiner cpu;
        cpu.configure(5, 5, SubpixelFit::SsdParabola);
        SubpixelVk gpu(snapL->ctx, 5, 5, SubpixelFit::SsdParabola);
        gpu.set_pyramids(snapL, snapR);
        auto t0 = std::chrono::steady_clock::now();
        for (int r = 0; r < reps; ++r)
            for (int k = 0; k < n; ++k) {
                const SubpixelJob& j = jobs[k];
                okCpu[k] = cpu.refine(cpuL[j.level], j.left, j.dirL, cpuR[j.level], j.right, j.dirR, offCpu[k]);
            }
        const double cpuMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / reps;
        t0 = std::chrono::steady_clock::now();
        for (int r = 0; r < reps; ++r) gpu.refine(jobs.data(), n, offGpu.data(), okGpu.data());
        const double gpuMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / reps;
        std::printf("  CPU (pyramids in normal memory) %.3f ms, GPU (submit + wait) %.3f ms\n", cpuMs, gpuMs);
    }

    std::printf(failures ? "%d FAILED\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
