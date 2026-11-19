// epipolar_synth_test: StereoMatcher on synthetic scenes with known correspondences.
//
//   epipolar_synth_test [--trials 200]
//
// Each trial: 3D points 0.3-8 m in front of the rig, projected into both KB8 cameras with
// Gaussian noise sigma_px · 1.2^octave per axis; descriptors random, the right copy with 0-40
// flipped bits. Distractors: keypoints seen by one camera only, keypoints elsewhere in the right
// image whose descriptor is as close as the true one (fools descriptor-only matching), and in one
// scenario keypoints on the same epipolar line at another depth (repeated texture). Keypoints are
// shuffled. The same scenes go through ORB-SLAM3's fisheye stereo matching (baseline).
//
// Checks: precision > 99% (no repeated texture), noise-free exactness, recall, depth error,
// no heap allocation in match() after warm-up, argument checks.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <new>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "algorithms/orb_matcher/stereo_matcher.hpp"
#include "tests/orb_matcher/orbslam3_fisheye_baseline.hpp"
#include "tests/orb_matcher/synth_rigs.hpp"

using namespace orb_matcher;

// ---- heap allocation counter (for the no-allocation check) -----------------------------------
static std::atomic<long> gAllocs{0};
void* operator new(size_t n) {
    ++gAllocs;
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }

static int failures = 0;
static void check(bool ok, const std::string& what) {
    printf("  %-70s %s\n", what.c_str(), ok ? "ok" : "FAIL");
    failures += !ok;
}
template <class E>
static bool throws(const std::function<void()>& f) {
    try { f(); } catch (const E&) { return true; } catch (...) { return false; }
    return false;
}

// ---- scenes ---------------------------------------------------------------------------------
struct Scene {
    std::vector<cv::KeyPoint> kp[2];
    std::vector<uint8_t> desc[2];        // 32 bytes per keypoint
    std::vector<int> truthLR;            // left index -> true right index, -1 none
    std::vector<float> truthDepth;       // left index -> true z
};

struct SceneConfig {
    int common = 250, leftOnly = 150, rightOnly = 150;
    int offLine = 60;     // right-only keypoints anywhere, descriptor as close as the true match
    int onLine = 0;       // right-only keypoints on the true epipolar line at another depth
    float noise = 1.0f;   // sigma_px at level 0 (per axis)
    int maxFlips = 40;
};

static const int kLevels = 8;
static std::vector<float> scaleFactors() {
    std::vector<float> s(kLevels);
    for (int l = 0; l < kLevels; ++l) s[l] = std::pow(1.2f, float(l));
    return s;
}

static Scene makeScene(const camera::StereoRig& rig, const SceneConfig& sc, std::mt19937& rng) {
    const std::vector<float> scale = scaleFactors();
    std::uniform_real_distribution<float> uni(0, 1);
    std::normal_distribution<float> gauss(0, 1);
    const int W = rig.width(), H = rig.height(), border = 19;
    const camera::KannalaBrandt8& c1 = rig.camera(0);
    const camera::KannalaBrandt8& c2 = rig.camera(1);
    const Eigen::Matrix3f R21 = rig.R_12().transpose();
    // ORB-like level distribution: weight 1.2^(-2l)
    std::vector<float> w(kLevels);
    for (int l = 0; l < kLevels; ++l) w[l] = std::pow(1.2f, -2.f * l);
    std::discrete_distribution<int> levelDist(w.begin(), w.end());

    auto inside = [&](const Eigen::Vector2f& uv) {
        return uv.x() >= border && uv.x() < W - border && uv.y() >= border && uv.y() < H - border;
    };
    auto randomDesc = [&](uint8_t* d) { for (int b = 0; b < 32; ++b) d[b] = uint8_t(rng()); };
    auto flipped = [&](const uint8_t* src, uint8_t* d, int flips) {
        std::copy(src, src + 32, d);
        std::vector<int> bits(256);
        std::iota(bits.begin(), bits.end(), 0);
        std::shuffle(bits.begin(), bits.end(), rng);
        for (int f = 0; f < flips; ++f) d[bits[f] / 8] ^= uint8_t(1u << (bits[f] % 8));
    };
    auto noisy = [&](const Eigen::Vector2f& uv, int level) {
        const float s = sc.noise * scale[level];
        return cv::KeyPoint(uv.x() + s * gauss(rng), uv.y() + s * gauss(rng), 31.f * scale[level], -1, 0, level);
    };
    auto randomPixel = [&] {
        return Eigen::Vector2f(border + uni(rng) * (W - 2 * border - 1), border + uni(rng) * (H - 2 * border - 1));
    };
    auto rightPixel = [&](const Eigen::Vector3f& P, Eigen::Vector2f& uv) {
        const Eigen::Vector3f P2 = R21 * (P - rig.t_12());
        return P2.z() > 0.05f && c2.project(P2, uv) && inside(uv);
    };

    struct Raw { cv::KeyPoint kp; uint8_t d[32]; int partner; float depth; };
    std::vector<Raw> L, R;
    for (int n = 0; n < sc.common; ++n) {
        Eigen::Vector3f f;
        const Eigen::Vector2f uvL = randomPixel();
        c1.unproject(uvL, f);
        const float z = 0.3f * std::pow(8.f / 0.3f, uni(rng));   // log-uniform depth
        const Eigen::Vector3f P = f / f.z() * z;
        Eigen::Vector2f uvR;
        const int lvl = levelDist(rng);
        Raw l{noisy(uvL, lvl), {}, -1, z};
        randomDesc(l.d);
        if (!rightPixel(P, uvR)) { L.push_back(l); continue; }   // left only
        const float u = uni(rng);
        const int lvlR = std::clamp(u < 0.8f ? lvl : u < 0.9f ? lvl - 1 : lvl + 1, 0, kLevels - 1);
        Raw r{noisy(uvR, lvlR), {}, int(L.size()), z};
        flipped(l.d, r.d, int(uni(rng) * (sc.maxFlips + 1)));
        l.partner = int(R.size());
        L.push_back(l);
        R.push_back(r);
        // repeated texture on the epipolar line: the same ray at another depth
        if (n < sc.onLine) {
            Eigen::Vector2f uvR2;
            const float z2 = z * (uni(rng) < 0.5f ? 0.5f + 0.4f * uni(rng) : 1.25f + uni(rng));
            if (rightPixel(f / f.z() * z2, uvR2)) {
                Raw r2{noisy(uvR2, lvlR), {}, -1, z2};
                flipped(l.d, r2.d, int(uni(rng) * (sc.maxFlips + 1)));
                R.push_back(r2);
            }
        }
        // look-alike anywhere in the image (other points than the ones with a twin on the line)
        if (n >= sc.onLine && n < sc.onLine + sc.offLine) {
            Raw r2{noisy(randomPixel(), lvlR), {}, -1, 0};
            flipped(l.d, r2.d, int(uni(rng) * (sc.maxFlips + 1)));
            R.push_back(r2);
        }
    }
    for (int n = 0; n < sc.leftOnly; ++n) {
        Raw l{noisy(randomPixel(), levelDist(rng)), {}, -1, 0};
        randomDesc(l.d);
        L.push_back(l);
    }
    for (int n = 0; n < sc.rightOnly; ++n) {
        Raw r{noisy(randomPixel(), levelDist(rng)), {}, -1, 0};
        randomDesc(r.d);
        R.push_back(r);
    }

    // shuffle both views; translate the correspondences
    std::vector<int> pl(L.size()), pr(R.size());
    std::iota(pl.begin(), pl.end(), 0);
    std::iota(pr.begin(), pr.end(), 0);
    std::shuffle(pl.begin(), pl.end(), rng);
    std::shuffle(pr.begin(), pr.end(), rng);
    std::vector<int> whereR(R.size());
    for (size_t k = 0; k < pr.size(); ++k) whereR[pr[k]] = int(k);
    Scene s;
    s.truthLR.assign(L.size(), -1);
    s.truthDepth.assign(L.size(), 0);
    for (int v = 0; v < 2; ++v) {
        const std::vector<Raw>& src = v ? R : L;
        const std::vector<int>& perm = v ? pr : pl;
        s.kp[v].resize(src.size());
        s.desc[v].resize(src.size() * 32);
        for (size_t k = 0; k < perm.size(); ++k) {
            const Raw& x = src[perm[k]];
            s.kp[v][k] = x.kp;
            std::copy(x.d, x.d + 32, &s.desc[v][k * 32]);
            if (v == 0) {
                s.truthDepth[k] = x.depth;
                if (x.partner >= 0) s.truthLR[k] = whereR[x.partner];
            }
        }
    }
    return s;
}

static ViewFeatures viewOf(const Scene& s, int v) {
    ViewFeatures f;
    f.keypoints = s.kp[v].data();
    f.n = int(s.kp[v].size());
    f.descriptors = s.desc[v].data();
    f.stride = 32;
    return f;
}

// ---- evaluation ---------------------------------------------------------------------------
struct Tally {
    long matchable = 0, matched = 0, correct = 0, wrong = 0;
    long nearMatched = 0, nearWrong = 0;   // left keypoints whose true depth < 2 m
    std::vector<float> relErr;          // correct matches
    std::vector<float> relErrBin[3];    // depth < 1 m, 1-3 m, > 3 m
    std::vector<double> ms;
    double precision() const { return matched ? double(correct) / matched : 1; }
    double recall() const { return matchable ? double(correct) / matchable : 0; }
    double nearPrecision() const { return nearMatched ? 1 - double(nearWrong) / nearMatched : 1; }
};

static void score(const Scene& s, const std::vector<int>& lr, const std::vector<float>& depth, Tally& t) {
    for (size_t i = 0; i < lr.size(); ++i) {
        t.matchable += s.truthLR[i] >= 0;
        if (lr[i] < 0) continue;
        ++t.matched;
        const bool near = s.truthDepth[i] > 0 && s.truthDepth[i] < 2;
        t.nearMatched += near;
        if (lr[i] != s.truthLR[i]) { ++t.wrong; t.nearWrong += near; continue; }
        ++t.correct;
        const float z = s.truthDepth[i], e = std::abs(depth[i] - z) / z;
        t.relErr.push_back(e);
        t.relErrBin[z < 1 ? 0 : z < 3 ? 1 : 2].push_back(e);
    }
}

static double median(std::vector<float> v) {
    if (v.empty()) return NAN;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}
static double mean(const std::vector<double>& v) {
    return v.empty() ? 0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}
static double p99(std::vector<double> v) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[size_t(0.99 * (v.size() - 1))];
}

static void printTally(const char* name, const Tally& t) {
    printf("    %-20s precision %7.3f%%  recall %5.1f%%  wrong %4ld / %6ld  depth err median %.2f%% (<1 m %.2f%%, 1-3 m %.2f%%, >3 m %.2f%%)",
           name, 100 * t.precision(), 100 * t.recall(), t.wrong, t.matched, 100 * median(t.relErr),
           100 * median(t.relErrBin[0]), 100 * median(t.relErrBin[1]), 100 * median(t.relErrBin[2]));
    if (!t.ms.empty()) printf("  %.3f ms (p99 %.3f)", mean(t.ms), p99(t.ms));
    printf("\n    %-20s depth < 2 m: precision %7.3f%%, %ld matches\n", "", 100 * t.nearPrecision(), t.nearMatched);
}

struct Result { Tally ours, base; };

// Keypoints and descriptors only: no images, so no sub-pixel refinement (subpixel_synth_test).
// No parallax limit either: the scenarios measure matching over the whole 0.3-8 m range.
static StereoMatcherParams noImages() {
    StereoMatcherParams p;
    p.subpixel = false;
    p.min_parallax_deg = 0;
    return p;
}

static Result runScenario(const char* name, const camera::StereoRig& rig, const SceneConfig& sc, int trials, unsigned seed,
                          const StereoMatcherParams& params = noImages(), bool quiet = false) {
    std::mt19937 rng(seed);
    StereoMatcher matcher(rig, params, 1000);
    StereoMatches out;
    BaselineMatches base;
    const std::vector<float> scale = scaleFactors();
    Result r;
    for (int t = 0; t < trials; ++t) {
        const Scene s = makeScene(rig, sc, rng);
        matcher.match(viewOf(s, 0), viewOf(s, 1), scale, out);
        r.ours.ms.push_back(out.stats.ms);
        score(s, out.left_to_right, out.depth, r.ours);
        const auto t0 = std::chrono::steady_clock::now();
        orbslam3FisheyeMatches(rig, viewOf(s, 0), viewOf(s, 1), scale, base);
        r.base.ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        score(s, base.left_to_right, base.depth, r.base);
    }
    if (quiet) return r;
    printf("  %s\n", name);
    printTally("orb_matcher", r.ours);
    printTally("ORB-SLAM3 fisheye", r.base);
    return r;
}

int main(int argc, char** argv) {
    int trials = 200;
    for (int i = 1; i + 1 < argc; ++i)
        if (std::string(argv[i]) == "--trials") trials = std::max(1, atoi(argv[++i]));

    struct NamedRig { const char* name; camera::StereoRig rig; };
    const NamedRig rigs[2] = {{"target rig (720x540, B 5 cm)", targetRig()}, {"EuRoC rig (752x480, B 11 cm)", eurocRig()}};
    for (const NamedRig& nr : rigs) {
        printf("%s, %d trials per scenario\n", nr.name, trials);
        SceneConfig exact;
        exact.noise = 0;
        exact.offLine = 0;
        const Result e = runScenario("noise-free, no look-alikes", nr.rig, exact, trials, 1);
        check(e.ours.precision() == 1.0 && e.ours.recall() > 0.99, "noise-free: precision 100%, recall > 99%");
        check(median(e.ours.relErr) < 1e-4, "noise-free: depth exact (median rel. error < 1e-4)");

        const Result n = runScenario("noise 1 px + 60 look-alikes off the line", nr.rig, SceneConfig(), trials, 2);
        check(n.ours.precision() > 0.99, "precision > 99%");
        check(n.ours.recall() > 0.80, "recall > 80%");
        check(1 - n.ours.precision() < 0.5 * (1 - n.base.precision()), "wrong-match rate below half the baseline's");

        SceneConfig rep;
        rep.onLine = 60;
        const Result r = runScenario("noise 1 px + 60 look-alikes off + 60 on the line", nr.rig, rep, trials, 3);
        check(r.ours.precision() >= r.base.precision(), "repeated texture on the line: precision >= baseline");

        printf("  ratio sweep (precision / recall): off the line | + on the line\n");
        for (float ratio : {0.6f, 0.7f, 0.8f, 0.9f}) {
            StereoMatcherParams p = noImages();
            p.ratio = ratio;
            const Result a = runScenario("", nr.rig, SceneConfig(), trials, 2, p, true);
            const Result b = runScenario("", nr.rig, rep, trials, 3, p, true);
            printf("    ratio %.1f   %7.3f%% / %5.1f%%   |   %7.3f%% / %5.1f%%\n", ratio, 100 * a.ours.precision(),
                   100 * a.ours.recall(), 100 * b.ours.precision(), 100 * b.ours.recall());
        }
        printf("\n");
    }

    printf("allocation, arguments\n");
    {
        const camera::StereoRig rig = targetRig();
        std::mt19937 rng(9);
        StereoMatcher m(rig, noImages(), 1000);
        StereoMatches out;
        const std::vector<float> scale = scaleFactors();
        std::vector<Scene> scenes;
        for (int i = 0; i < 20; ++i) scenes.push_back(makeScene(rig, SceneConfig(), rng));
        // warm-up: one pass sizes `out` for the largest scene
        for (const Scene& s : scenes) m.match(viewOf(s, 0), viewOf(s, 1), scale, out);
        const long before = gAllocs.load();
        for (const Scene& s : scenes) m.match(viewOf(s, 0), viewOf(s, 1), scale, out);
        check(gAllocs.load() == before, "no heap allocation in match() after warm-up");

        check(throws<std::invalid_argument>([&] { m.check_image_size(1440, 1080); }), "check_image_size: 1440x1080 on a 720x540 rig");
        check(!throws<std::invalid_argument>([&] { m.check_image_size(720, 540); }), "check_image_size: 720x540 accepted");
        StereoMatcherParams bad = noImages();
        bad.ratio = 0;
        check(throws<std::invalid_argument>([&] { StereoMatcher(rig, bad); }), "ratio = 0 rejected");
        bad = noImages();
        bad.max_depth = 0.1f;
        check(throws<std::invalid_argument>([&] { StereoMatcher(rig, bad); }), "max_depth <= min_depth rejected");
        StereoMatches empty;
        m.match(ViewFeatures(), ViewFeatures(), scale, empty);
        check(empty.stats.matched == 0 && empty.left_to_right.empty(), "empty views");
        // default parallax limit: every stereo point keeps an angle >= 1.146° between its two rays
        {
            StereoMatcherParams p = noImages();
            p.min_parallax_deg = StereoMatcherParams().min_parallax_deg;
            StereoMatcher mp(rig, p, 1000);
            double minAngle = 180;
            long rejected = 0, matched = 0;
            for (const Scene& sc : scenes) {
                mp.match(viewOf(sc, 0), viewOf(sc, 1), scale, out);
                rejected += out.stats.rejected_parallax;
                for (size_t i = 0; i < out.left_to_right.size(); ++i) {
                    if (out.left_to_right[i] < 0) continue;
                    ++matched;
                    const Eigen::Vector3f P = out.points[i], Q = P - rig.t_12();
                    minAngle = std::min(minAngle, std::acos(std::clamp(double(P.normalized().dot(Q.normalized())), -1.0, 1.0)) * 180 / M_PI);
                }
            }
            char msg[160];
            snprintf(msg, sizeof msg, "min_parallax_deg default: smallest ray angle %.3f deg (%ld matched, %ld rejected)",
                     minAngle, matched, rejected);
            check(matched > 0 && rejected > 0 && minAngle >= p.min_parallax_deg - 0.01, msg);
            bad = noImages();
            bad.min_parallax_deg = 90;
            check(throws<std::invalid_argument>([&] { StereoMatcher(rig, bad); }), "min_parallax_deg = 90 rejected");
        }
        StereoMatcher withSub(rig, StereoMatcherParams(), 1000);   // subpixel on by default
        check(throws<std::invalid_argument>([&] { withSub.match(viewOf(scenes[0], 0), viewOf(scenes[0], 1), scale, out); }),
              "subpixel without pyramids rejected");
    }

    printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
    return failures ? 1 : 0;
}
