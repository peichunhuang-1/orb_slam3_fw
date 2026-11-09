// orb_gpu_vs_cpu: the orb_extractor block's two backends must give the same keypoints and descriptors.
// Runs the GPU engine (orbvk::ORBextractorVk, configured as the block does) and the CPU one
// (orb_cpu::Extractor) on synthetic scenes and on the given 8-bit PGM images, over several settings,
// and compares the keypoints and descriptors (see kMaxAngleDeg / kMaxHamming). Also prints the time per image of both.
//
//   build/orb_slam3_fw/orb_gpu_vs_cpu [--allow-cpu] [image.pgm ...]      exit code 0 = all match
//
// --allow-cpu: accept a software Vulkan device (llvmpipe) when there is no GPU, to check the shaders.
//
// Built with the Vulkan backend only (CMakeLists.txt); not installed.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "algorithms/orb_extractor/orb_cpu.hpp"
#include "algorithms/orb_extractor/vk/orb_vk_slam.hpp"

namespace {

bool loadPgm(const char* path, cv::Mat& img) {
    std::ifstream f(path, std::ios::binary);
    std::string magic;
    if (!(f >> magic) || magic != "P5") return false;
    auto next = [&](int& v) {
        while (f >> std::ws && f.peek() == '#') f.ignore(1 << 20, '\n');
        f >> v;
    };
    int w, h, maxv;
    next(w);
    next(h);
    next(maxv);
    f.get();
    if (!f || maxv != 255) return false;
    img.create(h, w, CV_8UC1);
    f.read(reinterpret_cast<char*>(img.data), (std::streamsize)w * h);
    return (bool)f;
}

// Rectangles, discs and noise on a mid-grey background, softened by a box blur; the top third is
// almost flat so that the per-cell fallback threshold is used there. noise: uniform pixel noise.
cv::Mat scene(int w, int h, unsigned seed, bool noise) {
    std::mt19937 rng(seed);
    auto uni = [&](int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); };
    cv::Mat m(h, w, CV_8UC1, cv::Scalar(90));
    if (noise) {
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) m.at<uint8_t>(y, x) = (uint8_t)uni(0, 255);
        return m;
    }
    for (int i = 0; i < 220; ++i) {
        const uint8_t c = (uint8_t)uni(0, 255);
        const int cx = uni(-40, w), cy = uni(-40, h), a = uni(4, 90), b = uni(4, 90);
        const bool disc = i % 2;
        for (int y = std::max(0, cy - b); y < std::min(h, cy + b); ++y)
            for (int x = std::max(0, cx - a); x < std::min(w, cx + a); ++x) {
                const double dx = (x - cx) / (double)a, dy = (y - cy) / (double)b;
                if (!disc || dx * dx + dy * dy <= 1.0) m.at<uint8_t>(y, x) = c;
            }
    }
    cv::Mat out(h, w, CV_8UC1);
    std::normal_distribution<double> n(0, 3);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            int s = 0, k = 0;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    const int X = std::clamp(x + dx, 0, w - 1), Y = std::clamp(y + dy, 0, h - 1);
                    s += m.at<uint8_t>(Y, X);
                    ++k;
                }
            const double base = y < h / 3 ? 128.0 : s / (double)k;
            out.at<uint8_t>(y, x) = (uint8_t)std::clamp((int)std::lround(base + n(rng)), 0, 255);
        }
    return out;
}

// GLSL's atan / cos / sin are not correctly rounded (their precision is the driver's), so the angle
// can differ from std::atan2 in the last bits, and a BRIEF test whose rotated sample lies at a rounding
// boundary can flip. Everything else (corners, responses, selection, order) must be identical.
constexpr double kMaxAngleDeg = 0.01;
constexpr int kMaxHamming = 8;

// The block's GPU configuration (orb_extractor.cpp, to_gpu_config).
bool allowCpuDevice = false;

orbvk::ExtractorConfig gpuConfig(const orb_cpu::Config& c) {
    orbvk::ExtractorConfig g;
    g.allowCpuDevice = allowCpuDevice;
    g.nFeatures = c.nfeatures;
    g.scaleFactor = c.scale_factor;
    g.nLevels = c.nlevels;
    g.iniThFAST = c.ini_th_fast;
    g.minThFAST = c.min_th_fast;
    g.cellThreshold = c.per_cell_threshold;
    g.adaptiveThreshold = false;
    g.retryBelow = 0.f;
    g.edgeThreshold = c.edge_threshold;
    g.distribute = c.distribute;
    g.cellsPerFeature = c.cells_per_feature;
    g.abortCornerFraction = c.abort_corner_fraction;
    return g;
}

double ms(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

}  // namespace

int main(int argc, char** argv) {
    struct Input { std::string name; cv::Mat img; };
    std::vector<Input> inputs;
    const int sizes[][2] = {{752, 480}, {720, 540}, {640, 480}, {643, 479}, {120, 90}};
    for (auto& s : sizes)
        for (unsigned seed = 1; seed <= 2; ++seed)
            inputs.push_back({"scene " + std::to_string(s[0]) + "x" + std::to_string(s[1]) + " #" + std::to_string(seed),
                              scene(s[0], s[1], seed * 131 + s[0], false)});
    inputs.push_back({"noise 752x480", scene(752, 480, 7, true)});
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--allow-cpu") {
            allowCpuDevice = true;
            continue;
        }
        cv::Mat m;
        if (!loadPgm(argv[i], m)) {
            std::fprintf(stderr, "cannot read %s (8-bit binary PGM)\n", argv[i]);
            return 2;
        }
        inputs.push_back({argv[i], m});
    }

    struct Mode { const char* name; bool distribute, cell; int nf; float cpf, abort; };
    const Mode modes[] = {{"default", true, true, 1000, 1.f, 0.f},     {"strongest", false, false, 1000, 1.f, 0.f},
                          {"distribute", true, false, 500, 1.f, 0.f},  {"per-cell", false, true, 2000, 1.f, 0.f},
                          {"dense grid", true, true, 1200, 2.5f, 0.f}, {"abort 0.1", true, true, 1000, 1.f, 0.1f}};

    int runs = 0, fails = 0, skipped = 0;
    long kpsTotal = 0, angleOff = 0, descOff = 0;
    int maxHamming = 0;
    double maxAngle = 0;
    double gpuMs = 0, cpuMs = 0;
    int timed = 0;
    std::string device;
    for (const Mode& md : modes) {
        orb_cpu::Config c;
        c.nfeatures = md.nf;
        c.distribute = md.distribute;
        c.per_cell_threshold = md.cell;
        c.cells_per_feature = md.cpf;
        c.abort_corner_fraction = md.abort;
        orb_cpu::Extractor cpu(c);
        for (const Input& in : inputs) {
            orbvk::ORBextractorVk gpu(gpuConfig(c));
            gpu.Prepare(in.img.cols, in.img.rows);
            if (device.empty() && gpu.gpu()) device = gpu.gpu()->deviceName();
            std::vector<cv::KeyPoint> gk, ck;
            cv::Mat gd, cd;
            std::vector<int> lap = {0, 0};
            auto t0 = std::chrono::steady_clock::now();
            gpu(in.img, cv::Mat(), gk, gd, lap);
            auto t1 = std::chrono::steady_clock::now();
            cpu.extract(in.img, ck, cd);
            auto t2 = std::chrono::steady_clock::now();
            if (in.img.cols == 752 && in.name.rfind("scene", 0) == 0 && std::string(md.name) == "default") {
                gpuMs += ms(t0, t1);
                cpuMs += ms(t1, t2);
                ++timed;
            }
            if (gpu.LastFrame().overflow && !gpu.LastFrame().aborted) {
                // the GPU's corner / keypoint lists are bounded (pixel noise fills them); the CPU's are not
                ++skipped;
                continue;
            }
            ++runs;
            std::string why;
            if (gpu.LastFrame().aborted != cpu.last_frame().aborted) why = "abnormal-frame decision differs";
            else if (gk.size() != ck.size())
                why = std::to_string(gk.size()) + " GPU vs " + std::to_string(ck.size()) + " CPU keypoints";
            for (size_t i = 0; why.empty() && i < gk.size(); ++i) {
                const cv::KeyPoint &a = gk[i], &b = ck[i];
                ++kpsTotal;
                if (a.pt != b.pt || a.size != b.size || a.response != b.response || a.octave != b.octave) {
                    why = "keypoint " + std::to_string(i) + " differs";
                    break;
                }
                double da = std::fabs(a.angle - b.angle);
                da = std::min(da, 360.0 - da);
                if (da != 0) ++angleOff;
                maxAngle = std::max(maxAngle, da);
                if (da > kMaxAngleDeg) why = "angle of keypoint " + std::to_string(i) + " differs by " + std::to_string(da) + " deg";
                const int hd = (int)cv::norm(gd.row((int)i), cd.row((int)i), cv::NORM_HAMMING);
                if (hd) ++descOff;
                maxHamming = std::max(maxHamming, hd);
                if (hd > kMaxHamming) why = "descriptor " + std::to_string(i) + ": " + std::to_string(hd) + " bits differ";
            }
            if (!why.empty()) {
                ++fails;
                std::printf("FAIL  %-10s %-22s %s\n", md.name, in.name.c_str(), why.c_str());
            }
        }
    }
    std::printf("device: %s\n", device.c_str());
    std::printf("%d/%d image x setting runs match (%d skipped: GPU list overflow on pixel noise)\n", runs - fails, runs, skipped);
    std::printf("keypoints: %ld, positions / responses / octaves identical; angle differs on %ld (max %.2g deg), "
                "descriptor on %ld (max %d bits)\n", kpsTotal, angleOff, maxAngle, descOff, maxHamming);
    if (timed)
        std::printf("752x480, 1000 features: GPU %.1f ms, CPU (1 thread) %.1f ms per image\n", gpuMs / timed, cpuMs / timed);
    return fails ? 1 : 0;
}
