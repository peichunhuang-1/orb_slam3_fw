#pragma once
// Sub-pixel position along the epipolar line (docs/ALGORITHM.md section 3).
//
// Both patches are sampled on a grid aligned with the local epipolar direction of their image
// (columns along it, rows across it): a local rectification. The right image is sampled once as a
// strip (2w+1) x (2w+1+2L) long, and the 2L+1 candidate patches are windows of it.
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

#include "algorithms/orb_matcher/stereo_matcher.hpp"   // SubpixelFit

namespace orb_matcher {

class SubpixelRefiner {
public:
    void configure(int halfWindow, int range, SubpixelFit fit = SubpixelFit::SsdParabola);

    // On one pyramid level (8-bit images, level pixel coordinates): the offset (level pixels,
    // along the unit direction dirR) from `right` that best matches the patch at `left`. dirL /
    // dirR: the local epipolar directions, pointing the same way (towards the epipole of the
    // baseline). False when the minimum is at the end of the range, the interpolation is off by
    // more than one step, or a patch leaves the image.
    bool refine(const cv::Mat& leftImg, const Eigen::Vector2f& left, const Eigen::Vector2f& dirL,
                const cv::Mat& rightImg, const Eigen::Vector2f& right, const Eigen::Vector2f& dirR, float& offset);

private:
    int w_ = 5, L_ = 5;
    int strideL_ = 12, strideR_ = 24;   // row strides, padded for 4-wide loads past the window
    SubpixelFit fit_ = SubpixelFit::SsdParabola;
    std::vector<float> patchL_, strip_, cost_;
};

}  // namespace orb_matcher
