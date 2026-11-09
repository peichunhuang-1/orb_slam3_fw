// Compiled only with Vulkan (ORB_HAVE_VULKAN), see orb_vk.cpp.
#ifdef ORB_HAVE_VULKAN
#include "orb_vk_slam.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include "orb_vk_log.hpp"

namespace orbvk {

ExtractorConfig& ORBextractorVk::defaultConfig() {
    static ExtractorConfig c;
    return c;
}

ORBextractorVk::ORBextractorVk(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST)
    : ORBextractorVk([&] {
          ExtractorConfig c = defaultConfig();
          c.nFeatures = nfeatures;
          c.scaleFactor = scaleFactor;
          c.nLevels = nlevels;
          c.iniThFAST = iniThFAST;
          c.minThFAST = minThFAST;
          return c;
      }()) {}

ORBextractorVk::ORBextractorVk(const ExtractorConfig& cfg) : cfg_(cfg), thr_(cfg.iniThFAST) {
    if (cfg_.nFeatures < 1 || cfg_.nLevels < 1 || cfg_.nLevels > 16 || cfg_.scaleFactor <= 1.f)
        throw std::runtime_error("ORBextractorVk: need nFeatures >= 1, 1 <= nLevels <= 16, scaleFactor > 1");
    if (cfg_.minThFAST < 1 || cfg_.minThFAST > cfg_.iniThFAST)
        throw std::runtime_error("ORBextractorVk: need 1 <= minThFAST <= iniThFAST");
    // Same scales as OrbVk's pyramid (and as ORB-SLAM3: scaleFactor^l, sigma^2 = scale^2).
    for (int l = 0; l < cfg_.nLevels; ++l) {
        float s = std::pow(cfg_.scaleFactor, (float)l);
        mvScaleFactor.push_back(s);
        mvInvScaleFactor.push_back(1.f / s);
        mvLevelSigma2.push_back(s * s);
        mvInvLevelSigma2.push_back(1.f / (s * s));
    }
    mvImagePyramid.resize(cfg_.nLevels);
    if (cfg_.imageWidth > 0 && cfg_.imageHeight > 0) Prepare(cfg_.imageWidth, cfg_.imageHeight);
}

void ORBextractorVk::Prepare(int width, int height) {
    ensureGpu(width, height);
    // Warm-up frame: a checkerboard (corners in every level) runs every kernel once, so the first
    // real frame does not pay for the driver's first-submission work. The adaptive state is reset.
    cv::Mat img(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) img.at<uint8_t>(y, x) = (((x / 24) ^ (y / 24)) & 1) ? 200 : 50;
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    std::vector<int> lap = {0, 0};
    (*this)(img, cv::Mat(), kps, desc, lap);
    thr_ = cfg_.iniThFAST;
    info_ = FrameInfo{};
}

ORBextractorVk::~ORBextractorVk() = default;

void ORBextractorVk::ensureGpu(int w, int h) {
    if (orb_ && orb_->levels()[0].w == w && orb_->levels()[0].h == h) return;
    OrbParams p;
    p.nfeatures = cfg_.nFeatures;
    p.maxFeatures = cfg_.maxFeatures > 0 ? std::max(cfg_.maxFeatures, cfg_.nFeatures) : 2 * cfg_.nFeatures;
    p.nlevels = cfg_.nLevels;
    p.scaleFactor = cfg_.scaleFactor;
    p.fastThreshold = thr_;
    p.edgeThreshold = cfg_.edgeThreshold;
    p.distribute = cfg_.distribute;
    p.minFastThreshold = cfg_.cellThreshold ? cfg_.minThFAST : 0;
    p.cellsPerFeature = cfg_.cellsPerFeature;
    p.abortCornerFraction = cfg_.abortCornerFraction;
    p.profile = cfg_.profile;
    p.perfCounters = cfg_.perfCounters;
    p.perfEach = cfg_.perfEach;
    p.listPipelines = cfg_.listPipelines;
    orb_.reset();  // release the old GPU resources first
    orb_ = std::make_unique<OrbVk>(w, h, p, cfg_.allowCpuDevice);
    pyramidPixels_ = 0;
    for (const LevelInfo& L : orb_->levels()) pyramidPixels_ += (long)L.w * L.h;
    if ((int)orb_->levels().size() < cfg_.nLevels) {
        char m[160];
        snprintf(m, sizeof m, "[orb_vk] %dx%d image: only %zu of %d pyramid levels fit (edgeThreshold %d)", w, h,
                 orb_->levels().size(), cfg_.nLevels, cfg_.edgeThreshold);
        orb_extractor::log(orb_extractor::log_level::warn, m);
    }
}

void ORBextractorVk::SetFastThreshold(int t) {
    if (t < 1 || t > 254) throw std::runtime_error("SetFastThreshold: threshold must be 1..254");
    thr_ = t;
}

void ORBextractorVk::SetNumFeatures(int n) {
    if (n < 1) throw std::runtime_error("SetNumFeatures: n must be >= 1");
    cfg_.nFeatures = n;
    if (orb_ && n <= orb_->maxFeatures()) orb_->setNumFeatures(n);
    else orb_.reset();  // rebuilt with more capacity on the next frame
}

// Next frame's threshold from this frame's counts. Lower it quickly when a level fell short of
// its quota; raise it slowly while there are many more candidates than needed (those extra
// corners cost Harris/NMS time but are never selected).
void ORBextractorVk::adapt() {
    if (!cfg_.adaptiveThreshold || cfg_.cellThreshold) return;
    if (info_.aborted) {   // abnormal frame: never lower the threshold for it
        thr_ = std::min(254, std::max(thr_, info_.fastThreshold) + std::max(2, thr_ / 4));
        return;
    }
    const int lo = cfg_.minThFAST;
    const int hi = std::max(lo, cfg_.maxThFAST > 0 ? cfg_.maxThFAST : cfg_.iniThFAST);
    int t = info_.fastThreshold;
    const float surplus = (float)info_.candidates / cfg_.nFeatures;
    if (info_.keypoints < cfg_.nFeatures) t -= std::max(1, t / 5);
    else if (surplus > 2 * cfg_.raiseRatio) t += std::max(1, t / 8);   // far too many: climb faster
    else if (surplus > cfg_.raiseRatio) t += 1;
    thr_ = std::clamp(t, lo, hi);
}

int ORBextractorVk::operator()(cv::InputArray image, cv::InputArray /*mask*/, std::vector<cv::KeyPoint>& keypoints,
                               cv::OutputArray descriptors, std::vector<int>& vLappingArea) {
    if (image.empty()) return -1;
    cv::Mat img = image.getMat();
    CV_Assert(img.type() == CV_8UC1);
    ensureGpu(img.cols, img.rows);

    auto run = [&](int t) {
        orb_->setFastThreshold(t);
        orb_->detectAndCompute(img.data, (int)img.step, kps_, desc_);
        const OrbFrameStats& s = orb_->lastStats();
        info_.fastThreshold = t;
        info_.keypoints = (int)kps_.size();
        info_.candidates = (int)s.candidates;
        info_.corners = (int)s.corners;
        info_.overflow = s.overflow;
        info_.aborted = s.aborted;
        info_.weakCells = (int)s.weakCells;
        info_.weakCorners = (int)s.weakCorners;
    };
    info_ = FrameInfo{};
    run(thr_);
    double gpu = orb_->lastGpuMs();
    if (!cfg_.cellThreshold && cfg_.retryBelow > 0 && !info_.aborted && !info_.overflow &&
        info_.keypoints < cfg_.retryBelow * cfg_.nFeatures && info_.fastThreshold > cfg_.minThFAST) {
        run(cfg_.minThFAST);  // sudden loss of texture: do not wait for the adaptation
        info_.retried = true;
        gpu += orb_->lastGpuMs();
    }
    info_.gpuMs = cfg_.profile ? gpu : -1.0;
    adapt();

    // Views of the GPU pyramid for stereo matching (no copy).
    const auto& L = orb_->levels();
    for (int l = 0; l < cfg_.nLevels; ++l)
        mvImagePyramid[l] = l < (int)L.size()
                                ? cv::Mat(L[l].h, L[l].w, CV_8UC1, const_cast<uint8_t*>(orb_->pyramidLevel(l)), L[l].stride)
                                : cv::Mat();

    // Output in ORB-SLAM3's layout: keypoints inside the lapping area go to the end.
    const int n = (int)kps_.size();
    keypoints.assign(n, cv::KeyPoint());
    if (n == 0) {
        descriptors.release();
        return 0;
    }
    descriptors.create(n, 32, CV_8U);
    cv::Mat D = descriptors.getMat();
    const float lapLo = vLappingArea.size() >= 2 ? (float)vLappingArea[0] : 0.f;
    const float lapHi = vLappingArea.size() >= 2 ? (float)vLappingArea[1] : -1.f;
    int mono = 0, stereo = n - 1;
    for (int i = 0; i < n; ++i) {
        const OrbKeypoint& k = kps_[i];
        int dst = (k.x >= lapLo && k.x <= lapHi) ? stereo-- : mono++;
        keypoints[dst] = cv::KeyPoint(k.x, k.y, k.size, k.angle, k.response, k.octave);
        memcpy(D.ptr(dst), &desc_[(size_t)i * 32], 32);
    }
    return mono;
}

}  // namespace orbvk

#endif  // ORB_HAVE_VULKAN
