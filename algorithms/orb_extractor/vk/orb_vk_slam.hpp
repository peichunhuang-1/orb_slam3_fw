#pragma once
// Per-image ORB extraction engine (GPU, orb_vk) with ORB-SLAM3's conventions: constructor
// arguments, operator() with the lapping area, scale getters, mvImagePyramid, and the FAST
// threshold policies (per-cell threshold, distribution, abnormal frames). Used by
// the orb_extractor block (algorithms/orb_extractor/orb_extractor.cpp).
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "orb_vk.hpp"

namespace orbvk {

struct ExtractorConfig {
    // ORB-SLAM3 settings, read from "ORBextractor.*" in the settings yaml.
    int   nFeatures   = 1000;
    float scaleFactor = 1.2f;
    int   nLevels     = 8;
    int   iniThFAST   = 20;    // FAST threshold of the first frame
    int   minThFAST   = 7;     // lowest threshold the adaptation may use

    // orb_vk options, read from "ORBextractor.vk.*" (all optional).
    bool  cellThreshold = true;       // per-cell FAST threshold as ORB-SLAM3: iniThFAST in every
                                      // grid cell, minThFAST in cells without a corner, decided
                                      // within the frame (OrbParams::minFastThreshold). When on,
                                      // the adaptive threshold and the retry below are not used.
    bool  adaptiveThreshold = true;   // adjust the FAST threshold from the previous frame
    int   maxThFAST   = 0;     // highest threshold the adaptation may use; 0 = iniThFAST.
                               // Raising it (e.g. 40) makes textured scenes faster.
    float raiseRatio  = 3.0f;  // raise the threshold while candidates > raiseRatio * nFeatures
    float retryBelow  = 0.5f;  // fewer keypoints than retryBelow * nFeatures: run the frame
                               // again at minThFAST (0 = never)
    int   maxFeatures = 0;     // capacity for SetNumFeatures() without rebuilding; 0 = 2 * nFeatures
    int   edgeThreshold = 19;  // border without keypoints; 19 matches ORB-SLAM3
    bool  distribute  = true;  // spread keypoints over a grid (OrbParams::distribute), the role
                               // of ORB-SLAM3's quadtree; off = strongest per level
    float cellsPerFeature = 1.0f;  // grid cells per keypoint of a level's quota
    float abortCornerFraction = 0.f;    // abnormal frame above this fraction of the pyramid's pixels
                               // as FAST corners: no keypoints, the GPU stops after FAST
                               // (OrbParams::abortCornerFraction); 0 = off (default, see
                               // orb_extractor::Options::abort_corner_fraction)
    int   imageWidth  = 0;     // > 0 (with imageHeight): set up and warm up the GPU in the
    int   imageHeight = 0;     // constructor instead of on the first frame (~90 ms per extractor);
                               // read from Camera.width / Camera.height
    bool  profile     = false; // record GPU timestamps (FrameInfo::gpuMs)
    bool  allowCpuDevice = false;
    // Debug, as OrbParams (were environment variables)
    std::string perfCounters;
    bool  perfEach = false;
    bool  listPipelines = false;
    // From ORB-SLAM3's settings file: extractorConfigFromSettings (extractor_settings.hpp, target
    // orb_extractor_settings, for the tests and tools).
};

// Information about the last extracted frame.
struct FrameInfo {
    int    fastThreshold = 0;   // threshold of the final run of this frame
    int    keypoints = 0;
    int    candidates = 0;      // after NMS, before per-level selection
    int    corners = 0;         // FAST corners
    bool   retried = false;     // the frame was run a second time at minThFAST
    int    weakCells = 0;       // cellThreshold: grid cells that used minThFAST
    int    weakCorners = 0;     // cellThreshold: FAST corners found there
    bool   overflow = false;
    bool   aborted = false;     // abnormal frame (abortCornerFraction): no keypoints
    double gpuMs = -1.0;        // GPU time, profile only (sum of both runs if retried)
};

class ORBextractorVk {
public:
    // Same arguments as ORB_SLAM3::ORBextractor; other options come from defaultConfig().
    ORBextractorVk(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);
    explicit ORBextractorVk(const ExtractorConfig& cfg);
    ~ORBextractorVk();

    // Options used by the 5-argument constructor (which ORB-SLAM3's Tracking calls). Set them
    // once at start-up, e.g. defaultConfig() = extractorConfigFromSettings(settingsFile).
    static ExtractorConfig& defaultConfig();

    // Sets up the GPU for this image size and runs one warm-up frame, so the first real frame
    // is not slower (otherwise done on the first frame). Called by the constructor when the
    // configuration has imageWidth / imageHeight.
    void Prepare(int width, int height);

    // Same contract as ORB_SLAM3::ORBextractor::operator(): 8-bit grayscale image, mask
    // ignored. Keypoints whose x lies in [vLappingArea[0], vLappingArea[1]] are placed at the
    // end (in reverse order); the return value is the number of keypoints before them.
    // Returns -1 for an empty image.
    int operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                   cv::OutputArray descriptors, std::vector<int>& vLappingArea);


    int GetLevels() const { return cfg_.nLevels; }
    float GetScaleFactor() const { return cfg_.scaleFactor; }
    std::vector<float> GetScaleFactors() const { return mvScaleFactor; }
    std::vector<float> GetInverseScaleFactors() const { return mvInvScaleFactor; }
    std::vector<float> GetScaleSigmaSquares() const { return mvLevelSigma2; }
    std::vector<float> GetInverseScaleSigmaSquares() const { return mvInvLevelSigma2; }

    // Runtime tuning.
    void SetFastThreshold(int t);   // used from the next frame; adaptation continues from it
    int  GetFastThreshold() const { return thr_; }
    void SetNumFeatures(int n);     // above maxFeatures the GPU side is rebuilt (slow, ~0.1 s)
    int  GetNumFeatures() const { return cfg_.nFeatures; }
    const ExtractorConfig& config() const { return cfg_; }
    const FrameInfo& LastFrame() const { return info_; }
    const OrbVk* gpu() const { return orb_.get(); }   // null before the first frame
    long PyramidPixels() const { return pyramidPixels_; }   // 0 before the first frame

    // Pyramid of the last frame (views of GPU-visible memory, valid until the next call),
    // used by ORB-SLAM3's stereo matching.
    std::vector<cv::Mat> mvImagePyramid;

private:
    void ensureGpu(int w, int h);
    void adapt();

    ExtractorConfig cfg_;
    std::unique_ptr<OrbVk> orb_;
    int thr_ = 20;
    long pyramidPixels_ = 0;
    FrameInfo info_;
    std::vector<OrbKeypoint> kps_;
    std::vector<uint8_t> desc_;
    std::vector<float> mvScaleFactor, mvInvScaleFactor, mvLevelSigma2, mvInvLevelSigma2;
};

}  // namespace orbvk
