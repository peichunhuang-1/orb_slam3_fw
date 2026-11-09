#pragma once
// calib::StereoImuRig (euroc_mcap_driver's "calib", sensor_msgs/CameraInfo conventions) -> StereoRig.
//
// The matcher works on the KB8 model (kannala_brandt.hpp). Per camera, by distortion_model:
//   equidistant / kannala_brandt / fisheye   KB8 as is (d = k1 k2 k3 k4)
//   plumb_bob / radtan (d = k1 k2 p1 p2 [k3]), or none / "" (no distortion)
//                                            KB8 fitted to the model over the whole image: fx fy cx cy
//                                            kept, k1..k4 by linear least squares on the pixel residuals
//                                            (EuRoC: rms ~0.1 px, the fit the reference results used)
// T_c1_c2 = T_body_left⁻¹ · T_body_right (the right camera's pose in the left one).
#include <string>

#include "algorithms/orb_matcher/kannala_brandt.hpp"
#include "algorithms/orb_matcher/stereo_rig.hpp"
#include "types/calib/stereo_imu_rig.hpp"

namespace orb_matcher {

// How well the KB8 model reproduces the camera's own model (pixels, over a 4 px grid of the image).
// All zero for a KB8 calibration taken as is.
struct ModelFit {
    bool fitted = false;
    double rms_px = 0, p99_px = 0, max_px = 0;
    double inner_max_px = 0;   // inside the extractor's 19 px border
};

// Radial-tangential (OpenCV / ROS plumb_bob) model on normalized coordinates.
struct Radtan {
    double fx = 1, fy = 1, cx = 0, cy = 0;
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;

    void distort(double x, double y, double& xd, double& yd) const;
    // Fixed-point iteration; false when it does not converge (outside the model's invertible range)
    bool undistort(double xd, double yd, double& x, double& y) const;
};

// KB8 with the same fx fy cx cy, k1..k4 fitted on a 4 px grid of the width x height image.
KannalaBrandt8 fitKannalaBrandt8(const Radtan& rt, int width, int height, ModelFit* fit = nullptr);

// One camera of the rig as KB8. Throws std::invalid_argument for an unknown distortion model,
// missing coefficients or a bad size.
KannalaBrandt8 cameraFromCalib(const calib::PinholeCamera& cam, ModelFit* fit = nullptr);

// Throws std::invalid_argument as cameraFromCalib and the StereoRig constructor.
StereoRig stereoRigFromCalib(const calib::StereoImuRig& rig, ModelFit* left_fit = nullptr, ModelFit* right_fit = nullptr);

}  // namespace orb_matcher
