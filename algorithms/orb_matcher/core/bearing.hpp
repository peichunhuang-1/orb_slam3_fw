#pragma once
// Per-keypoint bearings in the rectification frame (docs/ALGORITHM.md section 1), structure of arrays.
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

#include "algorithms/orb_matcher/kannala_brandt.hpp"

namespace orb_matcher {

struct BearingSet {
    std::vector<Eigen::Vector3f> f;   // unit bearing in the camera frame; (0, 0, 0) when invalid
    std::vector<float> theta;         // plane angle around the baseline
    std::vector<float> phi;           // angle to the baseline
    std::vector<float> sigma;         // angular keypoint noise (radians)
    std::vector<float> sigma_theta;   // the same noise along θ: sigma / sin φ
    std::vector<int> octave;
    std::vector<uint8_t> valid;
    int count = 0;                    // valid entries

    void reserve(int n);
};

// R_rc: camera -> rectification frame. Keypoints outside the model's range, or closer than
// asin(sin_phi_min) to the baseline, are invalid.
void computeBearings(const KannalaBrandt8& cam, const Eigen::Matrix3f& R_rc, const cv::KeyPoint* kp, int n,
                     const std::vector<float>& scale_factors, float sigma_px, float sin_phi_min, BearingSet& out);

}  // namespace orb_matcher
