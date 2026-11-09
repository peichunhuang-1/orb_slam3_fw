#pragma once
// ORB extraction on the CPU (pure algorithm, no GPU, no fw dependency).
//
// Ported from the GPU (Vulkan) orb_extractor: every step uses the same integer arithmetic as its
// shaders, so for the same image and settings the keypoints and descriptors are the ones the GPU
// version produces:
//
//   1. pyramid: level l = level l-1 resized to round(w0 / s^l) with fixed-point bilinear filtering
//   2. 7x7 Gaussian (integer kernel, clamp-to-edge) per level, for the BRIEF tests only
//   3. FAST-9 inside [edge, size - edge); per-cell threshold (iniThFAST, minThFAST in grid cells
//      without an iniThFAST corner) as ORB-SLAM3
//   4. Harris response of the kept corners, 3x3 non-maximum suppression
//   5. per-level quota (geometric, as ORB-SLAM3); with `distribute` the level keeps the strongest
//      keypoint of every grid cell first, then every cell's second, ... (ORB-SLAM3's quadtree role)
//   6. intensity-centroid orientation and rotated BRIEF (OpenCV's learned pattern, cv::ORB bit layout)
//
// orb_cpu.cpp is compiled with -ffp-contract=off (CMakeLists.txt): no fused multiply-add, as the
// shaders' `precise`, so the Harris responses and BRIEF rotations round the same way.
#include <cstdint>
#include <functional>
#include <vector>

#include <opencv2/core.hpp>

namespace orb_cpu {

struct Config {
    int   nfeatures = 1000;        // keypoints per image (ORB-SLAM3 nFeatures)
    float scale_factor = 1.2f;     // scale between levels (ORB-SLAM3 scaleFactor)
    int   nlevels = 8;             // pyramid levels (ORB-SLAM3 nLevels), 1..16
    int   ini_th_fast = 20;        // FAST threshold (ORB-SLAM3 iniThFAST)
    int   min_th_fast = 7;         // per-cell fallback threshold (ORB-SLAM3 minThFAST)
    bool  per_cell_threshold = true;   // false: ini_th_fast everywhere
    bool  distribute = true;           // spread keypoints over a grid; false: strongest per level
    float cells_per_feature = 1.0f;    // grid cells per keypoint of a level's quota
    float abort_corner_fraction = 0.f; // > 0: no keypoints when FAST corners exceed this fraction
                                       // of the pyramid's pixels (noise, gravel); 0 = off
    int   edge_threshold = 19;         // border without keypoints, >= 19 (BRIEF patch radius)
    float harris_k = 0.04f;

    bool operator==(const Config& o) const;
    bool operator!=(const Config& o) const { return !(*this == o); }
};

// Geometry of one pyramid level and its grid (distribute / per-cell threshold).
struct Level {
    int w = 0, h = 0;
    float scale = 1.f;
    int cell = 0, gw = 1, gh = 1;   // cell size in pixels, cells across / down inside the border
};

// Counts of the last frame.
struct FrameInfo {
    int  corners = 0;       // FAST corners (all levels)
    int  weak_cells = 0;    // per-cell threshold: cells that used min_th_fast
    int  candidates = 0;    // keypoints after NMS
    int  keypoints = 0;     // keypoints returned
    bool aborted = false;   // abnormal frame (abort_corner_fraction): no keypoints
};

// Runs fn(i) for i in [begin, end), possibly in parallel. fn must be safe to run concurrently for
// different i. Empty: sequential.
using ParallelFor = std::function<void(int begin, int end, const std::function<void(int)>& fn)>;

class Extractor {
public:
    // Throws std::invalid_argument for invalid settings.
    explicit Extractor(const Config& cfg);

    // Full ORB on an 8-bit single-channel image. Keypoints in full-resolution coordinates
    // (octave = level, size = 31 * scale, angle in degrees [0, 360), response = Harris);
    // descriptors N x 32 CV_8U in keypoint order, level by level. Throws std::invalid_argument
    // for an image that is not CV_8UC1 or too small for one level.
    void extract(const cv::Mat& gray, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                 const ParallelFor& parallel = {});

    const Config& config() const { return cfg_; }
    const FrameInfo& last_frame() const { return info_; }
    // Pyramid of the last frame, one CV_8UC1 per level that fits (level 0 = the image copy).
    // Copies of these cv::Mat (shared data) stay valid: the next extract() writes a level held by
    // someone else into a new image. ORB-SLAM3's stereo matching reads it.
    const std::vector<cv::Mat>& pyramid() const { return pyr_; }
    const std::vector<Level>& levels() const { return levels_; }

    // Scale pyramid as ORB-SLAM3's ORBextractor getters (all cfg.nlevels levels).
    const std::vector<float>& scale_factors() const { return scale_; }
    const std::vector<float>& inv_scale_factors() const { return inv_scale_; }
    const std::vector<float>& level_sigma2() const { return sigma2_; }
    const std::vector<float>& inv_level_sigma2() const { return inv_sigma2_; }

    // Keypoint quota per level (geometric distribution, as OpenCV / ORB-SLAM3).
    static std::vector<int> level_quotas(int nfeatures, float scale_factor, int nlevels);

private:
    void setup(int w, int h);   // level geometry and grid for this image size

    Config cfg_;
    int w0_ = 0, h0_ = 0;
    long pyramid_pixels_ = 0;
    std::vector<Level> levels_;
    std::vector<int> quotas_;
    std::vector<cv::Mat> pyr_, blur_;
    FrameInfo info_;
    std::vector<float> scale_, inv_scale_, sigma2_, inv_sigma2_;
};

}  // namespace orb_cpu
