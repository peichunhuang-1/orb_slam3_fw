#pragma once
// The matcher orb_matcher replaces, for comparisons: ORB-SLAM3 v1.0 Frame::ComputeStereoFishEyeMatches
// with KannalaBrandt8::TriangulateMatches, on the same rig and keypoints.
//
//   BFMatcher(NORM_HAMMING).knnMatch(left, right, k = 2) over the lapping area (here: every
//   keypoint), Lowe ratio best < 0.7 · second, no Hamming limit, no one-to-one constraint;
//   accepted when TriangulateMatches returns depth > 0.0001: parallax cos < 0.9998, linear (SVD)
//   triangulation from z = 1 rays, z > 0 in both cameras, |e|² < 5.991 · σ²(octave) in both images.
//
// Transcribed from the ORB-SLAM3 v1.0 sources (not vendored here); re-check against
// src/Frame.cc and src/CameraModels/KannalaBrandt8.cc when the ORB-SLAM3 tree is available.
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

#include "types/orb/hamming.hpp"
#include "algorithms/orb_matcher/stereo_matcher.hpp"

namespace orb_matcher {

struct BaselineMatches {
    std::vector<int> left_to_right;   // -1 = none
    std::vector<float> depth;         // z1, -1 = none
    int desc_matches = 0, matched = 0;
};

inline void orbslam3FisheyeMatches(const camera::StereoRig& rig, const ViewFeatures& L, const ViewFeatures& R,
                                   const std::vector<float>& scale_factors, BaselineMatches& out) {
    out.left_to_right.assign(L.n, -1);
    out.depth.assign(L.n, -1.f);
    out.desc_matches = out.matched = 0;
    const camera::KannalaBrandt8& c1 = rig.camera(0);
    const camera::KannalaBrandt8& c2 = rig.camera(1);
    const Eigen::Matrix3f R12 = rig.R_12(), R21 = R12.transpose();
    const Eigen::Vector3f t12 = rig.t_12(), t21 = -R21 * t12;
    Eigen::Matrix<float, 3, 4> T1, T2;
    T1 << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();
    T2 << R21, t21;
    auto zRay = [](const camera::KannalaBrandt8& c, const cv::KeyPoint& k, Eigen::Vector3f& r) {
        Eigen::Vector3f f;
        if (!c.unproject(Eigen::Vector2f(k.pt.x, k.pt.y), f) || f.z() <= 1e-6f) return false;
        r = f / f.z();   // ORB-SLAM3's unprojectEig: z = 1
        return true;
    };
    for (int i = 0; i < L.n; ++i) {
        const uint8_t* dl = L.descriptors + size_t(i) * L.stride;
        int b1 = 1 << 30, b2 = 1 << 30, j1 = -1;
        for (int j = 0; j < R.n; ++j) {
            const int d = orb::hamming256(dl, R.descriptors + size_t(j) * R.stride);
            if (d < b1) { b2 = b1; b1 = d; j1 = j; }
            else if (d < b2) b2 = d;
        }
        if (j1 < 0 || R.n < 2 || !(float(b1) < float(b2) * 0.7f)) continue;
        ++out.desc_matches;
        const cv::KeyPoint& k1 = L.keypoints[i];
        const cv::KeyPoint& k2 = R.keypoints[j1];
        Eigen::Vector3f r1, r2;
        if (!zRay(c1, k1, r1) || !zRay(c2, k2, r2)) continue;
        const Eigen::Vector3f r21 = R12 * r2;
        if (r1.dot(r21) / (r1.norm() * r21.norm()) > 0.9998f) continue;
        Eigen::Matrix4f A;
        A.row(0) = r1.x() * T1.row(2) - T1.row(0);
        A.row(1) = r1.y() * T1.row(2) - T1.row(1);
        A.row(2) = r2.x() * T2.row(2) - T2.row(0);
        A.row(3) = r2.y() * T2.row(2) - T2.row(1);
        const Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
        const Eigen::Vector4f h = svd.matrixV().col(3);
        const Eigen::Vector3f x3D = h.head<3>() / h(3);
        const float z1 = x3D.z();
        if (z1 <= 0) continue;
        const Eigen::Vector3f x2 = R21 * x3D + t21;
        if (x2.z() <= 0) continue;
        const float s1 = scale_factors[k1.octave] * scale_factors[k1.octave];
        const float s2 = scale_factors[k2.octave] * scale_factors[k2.octave];
        Eigen::Vector2f uv1, uv2;
        if (!c1.project(x3D, uv1) || (uv1 - Eigen::Vector2f(k1.pt.x, k1.pt.y)).squaredNorm() > 5.991f * s1) continue;
        if (!c2.project(x2, uv2) || (uv2 - Eigen::Vector2f(k2.pt.x, k2.pt.y)).squaredNorm() > 5.991f * s2) continue;
        if (z1 > 0.0001f) {
            out.left_to_right[i] = j1;
            out.depth[i] = z1;
            ++out.matched;
        }
    }
}

}  // namespace orb_matcher
