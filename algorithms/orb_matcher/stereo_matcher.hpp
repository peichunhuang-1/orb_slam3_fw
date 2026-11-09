#pragma once
// Stereo matching along epipolar great circles (docs/ALGORITHM.md sections 1-2).
//
// Every keypoint is unprojected once to a unit bearing and rotated into the rig's rectification
// frame (e_x along the baseline). There a match must have the same plane angle θ around the
// baseline (= the same rectified row) and a positive disparity angle γ = φ_R − φ_L inside the
// depth range. Right keypoints are sorted by θ, so each left keypoint only looks at the few
// right keypoints in its θ window; Hamming distances are computed only for those. Matches are
// one to one and triangulated in closed form, r_L = B · sin φ_R / sin γ, then checked by their
// reprojection error as ORB-SLAM3's KannalaBrandt8::TriangulateMatches does.
//
// Single threaded. All buffers are sized in the constructor for max_features per view; match()
// does not allocate unless a view has more keypoints, or `out` is a new object.
// θ is measured from the left optical axis around the baseline, in (−π, π]; it does not wrap
// for cameras with FOV < 180° side by side (the epipoles are outside the images).
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

#include "algorithms/orb_matcher/stereo_rig.hpp"

namespace orb_matcher {

// Sub-pixel cost and interpolation of its minimum:
//   SadParabola  ORB-SLAM3's pinhole stereo (parabola through the three SAD values)
//   SadV         equiangular line fit, the matching model for SAD's V-shaped minimum
//   SsdParabola  sum of squared differences, quadratic near the minimum
enum class SubpixelFit { SadParabola, SadV, SsdParabola };

struct StereoMatcherParams {
    int   th_dist = 75;            // Hamming upper limit ((TH_HIGH + TH_LOW) / 2 in ORB-SLAM3)
    float ratio = 0.7f;            // best < ratio · second best (repeated texture along the line);
                                   // 0.7 = ORB-SLAM3 fisheye; 0.9 gains no recall (epipolar_synth_test)
    int   octave_window = 1;       // |octave_L − octave_R| <= this
    float sigma_px = 1.0f;         // keypoint noise at level 0, pixels; level l: sigma_px · scale^l
    float gate_k = 3.0f;           // epipolar and disparity gates at gate_k · σ
    float min_depth = 0.15f;       // range from the left camera, metres (sets the largest γ)
    float max_depth = 0;           // 0 = unlimited (far points are matched; the caller decides)
    float min_epipole_deg = 5.f;   // bearings closer than this to the baseline stay monocular
    float min_parallax_deg = 1.146f;   // disparity angle γ below this: no stereo point (stays monocular);
                                   // 1.146° = ORB-SLAM3's TriangulateMatches (cos > 0.9998). Range limit
                                   // ≈ baseline / γ: 2.5 m at 5 cm, 5.5 m at 11 cm. 0 = off
    float chi2 = 5.991f;           // reprojection test: |e|² < chi2 · σ² in both images

    // Sub-pixel refinement along the epipolar line (docs/ALGORITHM.md section 3; needs the pyramids). After
    // the reprojection test, the right keypoint is moved onto the left keypoint's epipolar curve;
    // (2w+1)² patches on the left keypoint's pyramid level, aligned with each image's epipolar
    // direction and minus their centre value, are compared at 2L+1 positions along it, and the
    // minimum is interpolated. Matches whose minimum is at the end of the range or whose
    // interpolation is off by more than one step are dropped (as ORB-SLAM3's pinhole stereo does).
    // EuRoC V1_02 against the Leica scan: median depth error 4.49% -> 2.74%, errors > 20%
    // 8.7% -> 4.5% (README.md).
    bool  subpixel = true;
    int   subpixel_half_window = 5;   // w: 11x11 patches
    int   subpixel_range = 5;         // L: ±5 steps of one level pixel
    SubpixelFit subpixel_fit = SubpixelFit::SsdParabola;   // best of the three (subpixel_synth_test, EuRoC)
};

struct StereoMatchStats {
    int left = 0, right = 0;       // keypoints with a usable bearing
    int candidates = 0;            // (left, right) pairs inside the θ window
    int hamming_evals = 0;         // pairs that passed every geometric test
    int matched = 0;
    int rejected_dist = 0;         // best Hamming > th_dist (or no candidate passed the gates)
    int rejected_ratio = 0;
    int rejected_unique = 0;       // the right keypoint went to a closer left one
    int rejected_parallax = 0;     // γ <= 0, or below min_parallax_deg after triangulation
    int rejected_reproj = 0;
    int rejected_subpixel = 0;     // no clear SAD minimum, or the patch leaves the image
    double ms = 0;                 // wall time of match()
    double subpixel_ms = 0;        // part of ms spent in the refinement
};

struct StereoMatches {
    std::vector<int> left_to_right, right_to_left;   // -1 = no match
    std::vector<Eigen::Vector3f> points;             // left camera frame; valid where matched
    std::vector<float> depth;                        // z in the left camera, -1 = no match
    std::vector<int> distance;                       // per left: Hamming distance of the match, -1 = no match
    std::vector<Eigen::Vector2f> right_uv;           // per left: right pixel used (refined with subpixel)
    std::vector<Eigen::Vector3f> bearing[2];         // unit bearings ((0, 0, 0): outside the model)
    StereoMatchStats stats;
};

// One view's keypoints (full-resolution coordinates, octave = pyramid level) and descriptors
// (n rows of 32 bytes, `stride` bytes apart). The pyramid (8-bit images, level l scaled by
// 1 / scale_factors[l], keypoint at pt / scale_factors[l] on it) is needed for subpixel only.
struct ViewFeatures {
    const cv::KeyPoint* keypoints = nullptr;
    int n = 0;
    const uint8_t* descriptors = nullptr;
    size_t stride = 32;
    const cv::Mat* pyramid = nullptr;
    int pyramid_levels = 0;
};

class StereoMatcher {
public:
    // Throws std::invalid_argument for invalid parameters.
    StereoMatcher(const StereoRig& rig, const StereoMatcherParams& params = StereoMatcherParams(),
                  int max_features = 1000);
    ~StereoMatcher();
    StereoMatcher(const StereoMatcher&) = delete;
    StereoMatcher& operator=(const StereoMatcher&) = delete;

    // scale_factors[l] = scale of pyramid level l (orb::Features::scale_factors).
    void match(const ViewFeatures& left, const ViewFeatures& right, const std::vector<float>& scale_factors,
               StereoMatches& out);

    // Throws std::invalid_argument unless the images are the rig's size (a 1440 calibration
    // with 720 keypoints would match nothing and fail silently).
    void check_image_size(int width, int height) const;

    const StereoRig& rig() const;
    const StereoMatcherParams& params() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace orb_matcher
