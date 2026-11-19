#pragma once
// A stereo frame's features as tracking and its keyframes use them, shared (not copied) between the tracking
// frame and the keyframe made from it, as ORB-SLAM3's KeyFrame copies them from the Frame and never changes them.
//
// Feature index i (ORB-SLAM3's N = Nleft + Nright): i < n_left is left keypoint i, else right keypoint i - n_left.
// The descriptors are those of orb::StereoFrame, one matrix per view (ORB-SLAM3 stacks them).
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "types/orb/feature_grid.hpp"
#include "types/orb/stereo_frame.hpp"

namespace orb {

struct FrameFeatures {
    std::shared_ptr<const StereoFrame> stereo;
    FeatureGrid grid[2];  // left, right
    int n_left = 0, n_right = 0;

    // Scale levels (ORB-SLAM3's mvScaleFactors, mvInvScaleFactors, mvLevelSigma2, mvInvLevelSigma2)
    int levels = 0;
    float scale_factor = 1, log_scale_factor = 0;
    std::vector<float> scale_factors, inv_scale_factors, level_sigma2, inv_level_sigma2;

    explicit FrameFeatures(std::shared_ptr<const StereoFrame> s) : stereo(std::move(s)) {
        n_left = int(stereo->keypoints[0].size());
        n_right = int(stereo->keypoints[1].size());
        grid[0] = FeatureGrid(stereo->keypoints[0], stereo->width, stereo->height);
        grid[1] = FeatureGrid(stereo->keypoints[1], stereo->width, stereo->height);
        scale_factors = stereo->scale_factors;
        levels = int(scale_factors.size());
        scale_factor = levels > 1 ? scale_factors[1] : 1.0f;
        log_scale_factor = std::log(scale_factor);
        for (float s : scale_factors) {
            inv_scale_factors.push_back(1.0f / s);
            level_sigma2.push_back(s * s);
            inv_level_sigma2.push_back(1.0f / (s * s));
        }
    }

    int size() const { return n_left + n_right; }  // N
    bool isRight(int i) const { return i >= n_left; }
    const cv::KeyPoint& keypoint(int i) const {
        return i < n_left ? stereo->keypoints[0][std::size_t(i)] : stereo->keypoints[1][std::size_t(i - n_left)];
    }
    const std::uint8_t* descriptor(int i) const {
        return i < n_left ? stereo->descriptors[0].ptr<std::uint8_t>(i) : stereo->descriptors[1].ptr<std::uint8_t>(i - n_left);
    }
    const std::vector<cv::KeyPoint>& keys(int view) const { return stereo->keypoints[view]; }
    // Per left keypoint: its right match (-1 none), depth (-1 none) and point in the left camera
    int rightOf(int i) const { return stereo->left_to_right[std::size_t(i)]; }
    float depth(int i) const { return stereo->depth[std::size_t(i)]; }
    const Eigen::Vector3f& stereoPoint(int i) const { return stereo->points[std::size_t(i)]; }
    // Features of one view in a window (view 1: indices in the right view, add n_left for a feature index)
    std::vector<std::size_t> inArea(int view, float x, float y, float r, int min_level = -1, int max_level = -1) const {
        return grid[view].inArea(stereo->keypoints[view], x, y, r, min_level, max_level);
    }
    bool inImage(float x, float y) const { return x >= 0 && x < float(stereo->width) && y >= 0 && y < float(stereo->height); }
};

}  // namespace orb
