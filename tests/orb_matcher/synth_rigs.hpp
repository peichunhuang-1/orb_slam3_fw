#pragma once
// Rigs shared by the synthetic tests.
#include <cmath>

#include <Eigen/Geometry>

#include "algorithms/orb_matcher/stereo_rig.hpp"

namespace orb_matcher {

// docs/ALGORITHM.md: 1440x1080 wide lens, 2x downscaled to 720x540, 5 cm baseline
inline StereoRig targetRig() {
    const KannalaBrandt8 l(640.f, 641.5f, 719.5f, 539.5f, -0.012f, 0.004f, -0.0008f, 0.00005f, 1440, 1080);
    const KannalaBrandt8 r(645.f, 644.f, 721.f, 541.f, -0.011f, 0.0035f, -0.0007f, 0.00004f, 1440, 1080);
    const Eigen::Matrix3f R =
        Eigen::AngleAxisf(0.5f * float(M_PI) / 180, Eigen::Vector3f(0.3f, 1.f, -0.2f).normalized()).matrix();
    return StereoRig(l, r, R, Eigen::Vector3f(0.05f, 0.0003f, -0.0004f)).resized(720, 540);
}

// KB8 fitted to EuRoC cam0 / cam1 (euroc_epipolar_test), 752x480, 11 cm baseline
inline StereoRig eurocRig() {
    const KannalaBrandt8 l(458.654f, 457.296f, 367.215f, 248.375f, 0.0280538f, 0.122174f, -0.593399f, 0.565113f, 752, 480);
    const KannalaBrandt8 r(457.587f, 456.134f, 379.999f, 255.238f, 0.0256064f, 0.140625f, -0.635884f, 0.598362f, 752, 480);
    Eigen::Matrix3f R;
    R << 0.999997258f, -0.00231713569f, -0.000343393127f, 0.0023120672f, 0.999898076f, -0.0140906684f,
        0.000376008102f, 0.0140898358f, 0.999900639f;
    return StereoRig(l, r, R, Eigen::Vector3f(0.11007414f, -0.000156612048f, 0.000889382791f));
}

}  // namespace orb_matcher
