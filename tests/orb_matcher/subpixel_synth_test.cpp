// subpixel_synth_test: sub-pixel refinement along the epipolar line on rendered images.
//
//   subpixel_synth_test [--trials 40] [--supersample 3]
//
// Each trial renders a textured plane (0.5-5 m, tilted up to 25°) into both KB8 cameras (each pixel
// the mean of 3x3 samples over its area, as a sensor integrates) and builds
// 8-level pyramids (scale 1.2, INTER_LINEAR from level 0). Left keypoints sit on integer pixels
// of their level (pt = level pixel · scale, as ORB-SLAM3 reports them); the right keypoint is the
// true projection rounded to the same grid, i.e. only the detector's quantisation error. With
// identical descriptors every pair can match, so the test isolates the geometry and the SAD.
// Compared: StereoMatcher with subpixel off and on, against the plane's true depth.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <memory>
#include <new>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>

#include "algorithms/orb_matcher/stereo_matcher.hpp"
#include "tests/orb_matcher/synth_rigs.hpp"

using namespace orb_matcher;

// heap allocation counter (no allocation in match() after warm-up)
static std::atomic<long> gAllocs{0};
void* operator new(size_t n) {
    ++gAllocs;
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }
static long gAllocsInMatch = 0, gMatchCalls = 0;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    printf("  %-70s %s\n", what.c_str(), ok ? "ok" : "FAIL");
    failures += !ok;
}
static double pct(std::vector<float> v, double q) {
    if (v.empty()) return NAN;
    const size_t k = std::min(v.size() - 1, size_t(q * (v.size() - 1) + 0.5));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return v[k];
}

static const int kLevels = 8;

// Multi-scale noise texture (wraps around), 8-bit range.
static cv::Mat makeTexture(std::mt19937& rng, int size = 1024) {
    cv::Mat acc = cv::Mat::zeros(size, size, CV_32F);
    std::uniform_real_distribution<float> uni(-1, 1);
    float amp = 1;
    for (int cells = 8; cells <= size / 2; cells *= 2, amp *= 0.75f) {
        cv::Mat n(cells, cells, CV_32F);
        for (int i = 0; i < cells * cells; ++i) n.at<float>(i) = uni(rng);
        cv::Mat up;
        cv::resize(n, up, cv::Size(size, size), 0, 0, cv::INTER_CUBIC);
        acc += amp * up;
    }
    cv::normalize(acc, acc, 20, 235, cv::NORM_MINMAX);
    return acc;
}

static float sampleTexture(const cv::Mat& tex, float x, float y) {
    const int S = tex.cols;
    const float fx = std::floor(x), fy = std::floor(y), ax = x - fx, ay = y - fy;
    auto at = [&](int u, int v) { return tex.at<float>(((v % S) + S) % S, ((u % S) + S) % S); };
    const int u = int(fx), v = int(fy);
    return (1 - ay) * ((1 - ax) * at(u, v) + ax * at(u + 1, v)) + ay * ((1 - ax) * at(u, v + 1) + ax * at(u + 1, v + 1));
}

struct Plane {
    Eigen::Vector3f n;   // unit normal, left camera frame: n · X = d
    float d;
    Eigen::Vector3f e1, e2;   // in-plane axes
    float texelsPerMetre;
    bool intersect(const Eigen::Vector3f& origin, const Eigen::Vector3f& dir, Eigen::Vector3f& X) const {
        const float den = n.dot(dir);
        if (std::abs(den) < 1e-6f) return false;
        const float t = (d - n.dot(origin)) / den;
        if (t <= 0) return false;
        X = origin + t * dir;
        return true;
    }
};

// Each pixel averages ss x ss samples over its area (a sensor integrates; ss = 1 point-samples).
static cv::Mat render(const camera::StereoRig& rig, int cam, const Plane& pl, const cv::Mat& tex, int ss) {
    const camera::KannalaBrandt8& c = rig.camera(cam);
    cv::Mat img(c.height(), c.width(), CV_8U);
    const Eigen::Vector3f origin = cam ? rig.t_12() : Eigen::Vector3f::Zero();
    for (int v = 0; v < img.rows; ++v)
        for (int u = 0; u < img.cols; ++u) {
            float sum = 0;
            for (int sy = 0; sy < ss; ++sy)
                for (int sx = 0; sx < ss; ++sx) {
                    const Eigen::Vector2f uv(u + (sx + 0.5f) / ss - 0.5f, v + (sy + 0.5f) / ss - 0.5f);
                    Eigen::Vector3f f, X;
                    if (c.unproject(uv, f) && pl.intersect(origin, cam ? rig.R_12() * f : f, X))
                        sum += sampleTexture(tex, X.dot(pl.e1) * pl.texelsPerMetre, X.dot(pl.e2) * pl.texelsPerMetre);
                }
            img.at<uint8_t>(v, u) = cv::saturate_cast<uint8_t>(sum / (ss * ss));
        }
    return img;
}

static std::vector<cv::Mat> pyramid(const cv::Mat& img, const std::vector<float>& scale) {
    std::vector<cv::Mat> p(kLevels);
    p[0] = img;
    for (int l = 1; l < kLevels; ++l)
        cv::resize(img, p[l], cv::Size(int(std::lround(img.cols / scale[l])), int(std::lround(img.rows / scale[l]))), 0, 0,
                   cv::INTER_LINEAR);
    return p;
}

struct Tally {
    std::vector<float> relErr, dispErr;   // depth error; disparity-equivalent error f·B·|1/z − 1/z_true| (px)
    std::vector<float> dispSigned;        // f·B·(1/z − 1/z_true): > 0 = too close
    std::vector<float> dispErrLevel[kLevels];
    long pairs = 0, matched = 0, rejectedSub = 0;
    double ms = 0, subMs = 0;
    int frames = 0;
};

struct Mode { const char* name; bool subpixel; SubpixelFit fit; };
static const Mode kModes[] = {{"off", false, SubpixelFit::SadParabola},
                              {"SAD parabola", true, SubpixelFit::SadParabola},
                              {"SAD V-fit", true, SubpixelFit::SadV},
                              {"SSD parabola", true, SubpixelFit::SsdParabola}};
static const int kNumModes = 4;

// Returns the tallies of every mode.
static std::vector<Tally> runRig(const char* name, const camera::StereoRig& rig, int trials, unsigned seed, int ss) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> uni(0, 1);
    std::vector<float> scale(kLevels);
    for (int l = 0; l < kLevels; ++l) scale[l] = std::pow(1.2f, float(l));
    std::vector<float> w(kLevels);
    for (int l = 0; l < kLevels; ++l) w[l] = std::pow(1.2f, -2.f * l);
    std::discrete_distribution<int> levelDist(w.begin(), w.end());
    const camera::KannalaBrandt8& c1 = rig.camera(0);
    const camera::KannalaBrandt8& c2 = rig.camera(1);
    const Eigen::Matrix3f R21 = rig.R_12().transpose();
    const float fB = c1.fx() * rig.baseline();

    std::vector<Tally> tally(kNumModes);
    std::vector<std::unique_ptr<StereoMatcher>> matchers;
    for (const Mode& m : kModes) {
        StereoMatcherParams p;
        p.subpixel = m.subpixel;
        p.subpixel_fit = m.fit;
        p.min_parallax_deg = 0;   // measure the refinement over the whole 0.5-5 m range
        matchers.push_back(std::make_unique<StereoMatcher>(rig, p));
    }
    StereoMatches out;
    int maxN = 0;
    const cv::Mat tex = makeTexture(rng);
    for (int t = 0; t < trials; ++t) {
        // plane: depth 0.5-5 m (log-uniform) at the image centre, tilted up to 25°
        const float Z = 0.5f * std::pow(10.f, uni(rng));
        const Eigen::Vector3f axis = Eigen::Vector3f(uni(rng) - 0.5f, uni(rng) - 0.5f, 0).normalized();
        Plane pl;
        pl.n = Eigen::AngleAxisf(25 * float(M_PI) / 180 * uni(rng), axis) * Eigen::Vector3f::UnitZ();
        pl.d = Z * pl.n.z();
        pl.e1 = pl.n.cross(Eigen::Vector3f::UnitY()).normalized();
        pl.e2 = pl.n.cross(pl.e1);
        pl.texelsPerMetre = c1.fx() / Z * 0.7f;   // ~1.4 px per texel at the centre
        const cv::Mat imgL = render(rig, 0, pl, tex, ss), imgR = render(rig, 1, pl, tex, ss);
        const std::vector<cv::Mat> pyrL = pyramid(imgL, scale), pyrR = pyramid(imgR, scale);

        std::vector<cv::KeyPoint> kl, kr;
        std::vector<float> truthZ;
        const int border = 30;
        for (int k = 0; k < 300 && int(kl.size()) < 200; ++k) {
            const int lvl = levelDist(rng);
            const float s = scale[lvl];
            const int wl = pyrL[lvl].cols, hl = pyrL[lvl].rows;
            const float xl = std::floor(border / s + uni(rng) * (wl - 2 * border / s));
            const float yl = std::floor(border / s + uni(rng) * (hl - 2 * border / s));
            const Eigen::Vector2f uvL(xl * s, yl * s);
            Eigen::Vector3f f, X;
            Eigen::Vector2f uvR;
            if (!c1.unproject(uvL, f) || !pl.intersect(Eigen::Vector3f::Zero(), f, X)) continue;
            const Eigen::Vector3f X2 = R21 * (X - rig.t_12());
            if (!c2.project(X2, uvR) || uvR.x() < border || uvR.y() < border || uvR.x() >= c2.width() - border ||
                uvR.y() >= c2.height() - border)
                continue;
            const Eigen::Vector2f det = (uvR / s).array().round().matrix() * s;   // detector grid
            kl.emplace_back(uvL.x(), uvL.y(), 31.f * s, -1, 0, lvl);
            kr.emplace_back(det.x(), det.y(), 31.f * s, -1, 0, lvl);
            truthZ.push_back(X.z());
        }
        // identical random descriptors per pair; the right view in reverse order
        const int n = int(kl.size());
        std::vector<uint8_t> dl(size_t(n) * 32), dr(size_t(n) * 32);
        for (auto& b : dl) b = uint8_t(rng());
        std::vector<cv::KeyPoint> krRev(kr.rbegin(), kr.rend());
        for (int i = 0; i < n; ++i) std::copy(&dl[size_t(i) * 32], &dl[size_t(i) * 32] + 32, &dr[size_t(n - 1 - i) * 32]);
        ViewFeatures L{kl.data(), n, dl.data(), 32, pyrL.data(), kLevels};
        ViewFeatures R{krRev.data(), n, dr.data(), 32, pyrR.data(), kLevels};
        if (t == 0) maxN = n;

        for (int mode = 0; mode < kNumModes; ++mode) {   // `out` is warm once it has held the largest n
            Tally& tl = tally[mode];
            const long before = gAllocs.load();
            matchers[mode]->match(L, R, scale, out);
            if (t >= 1 && n <= maxN) { gAllocsInMatch += gAllocs.load() - before; ++gMatchCalls; }
            tl.ms += out.stats.ms;
            tl.subMs += out.stats.subpixel_ms;
            tl.rejectedSub += out.stats.rejected_subpixel;
            ++tl.frames;
            tl.pairs += n;
            for (int i = 0; i < n; ++i) {
                if (out.left_to_right[i] != n - 1 - i) continue;   // unmatched (wrong cannot happen)
                ++tl.matched;
                tl.relErr.push_back(std::abs(out.depth[i] - truthZ[i]) / truthZ[i]);
                const float es = fB * (1 / out.depth[i] - 1 / truthZ[i]);
                const float e = std::abs(es);
                tl.dispSigned.push_back(es);
                tl.dispErr.push_back(e);
                tl.dispErrLevel[kl[i].octave].push_back(e);
            }
        }
    }

    printf("%s, %d planes, %dx%d samples per pixel (disparity error = f·B·|1/z − 1/z_true|, level-0 px)\n", name, trials, ss, ss);
    printf("    %-13s matched  depth err median    p90   disparity err median    p90     bias   time/frame (subpixel)\n", "");
    for (int mode = 0; mode < kNumModes; ++mode) {
        const Tally& t = tally[mode];
        printf("    %-13s %5.1f%%        %6.3f%%  %6.3f%%            %6.3f px  %6.3f px  %+6.3f   %.3f ms (%.3f ms)\n",
               kModes[mode].name, 100.0 * t.matched / t.pairs, 100 * pct(t.relErr, 0.5), 100 * pct(t.relErr, 0.9),
               pct(t.dispErr, 0.5), pct(t.dispErr, 0.9), pct(t.dispSigned, 0.5), t.ms / t.frames, t.subMs / t.frames);
    }
    printf("    disparity err median by level (px):\n");
    for (int mode = 0; mode < kNumModes; ++mode) {
        printf("      %-13s", kModes[mode].name);
        for (int l = 0; l < kLevels; ++l)
            if (!tally[mode].dispErrLevel[l].empty()) printf("  L%d %.3f", l, pct(tally[mode].dispErrLevel[l], 0.5));
        printf("\n");
    }
    return tally;
}

int main(int argc, char** argv) {
    int trials = 40, ss = 3;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--trials") trials = std::max(1, atoi(argv[++i]));
        else if (std::string(argv[i]) == "--supersample") ss = std::max(1, atoi(argv[++i]));
    }
    const camera::StereoRig rigs[2] = {targetRig(), eurocRig()};
    const char* names[2] = {"target rig (720x540, B 5 cm)", "EuRoC rig (752x480, B 11 cm)"};
    const int def = int(StereoMatcherParams().subpixel_fit) + 1;   // the default fit's mode
    for (int r = 0; r < 2; ++r) {
        const std::vector<Tally> t = runRig(names[r], rigs[r], trials, 1 + r, ss);
        const Tally& off = t[0];
        const Tally& on = t[def];
        printf("  default fit: %s\n", kModes[def].name);
        check(pct(on.dispErr, 0.5) < 0.6 * pct(off.dispErr, 0.5), "disparity error median at most 60% of unrefined");
        check(pct(on.dispErrLevel[0], 0.5) < 0.15, "level 0: disparity error median < 0.15 px");
        check(double(on.rejectedSub) / on.pairs < 0.1, "fewer than 10% dropped by the refinement");
        printf("\n");
    }
    check(gMatchCalls > 0 && gAllocsInMatch == 0,
          "no heap allocation in match() after warm-up (" + std::to_string(gMatchCalls) + " calls)");
    printf(failures ? "%d FAILED\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
