#pragma once
// Calibrated stereo pair: two KB8 cameras and the pose of the right camera in the left one.
// From the framework's calibration (calib::StereoImuRig): stereoRigFromCalib (rig_from_calib.hpp).
#include <string>

#include <Eigen/Core>

#include "algorithms/orb_matcher/kannala_brandt.hpp"

namespace orb_matcher {

class StereoRig {
public:
    StereoRig() = default;
    // Throws std::invalid_argument if the cameras differ in size, R_12 is not a rotation
    // (|R·Rᵀ − I| > 1e-3 or det < 0) or the baseline is shorter than 0.1 mm. R_12 is
    // re-orthonormalised (float noise from the settings file).
    StereoRig(const KannalaBrandt8& left, const KannalaBrandt8& right, const Eigen::Matrix3f& R_12,
              const Eigen::Vector3f& t_12);

    // The rig for images resampled to width x height (KannalaBrandt8::resized; extrinsics unchanged).
    StereoRig resized(int width, int height) const;

    const KannalaBrandt8& camera(int i) const { return cam_[i]; }   // 0 left (Camera1), 1 right
    const Eigen::Matrix3f& R_12() const { return R_12_; }           // rotation right -> left
    const Eigen::Vector3f& t_12() const { return t_12_; }           // right camera centre in left
    float baseline() const { return t_12_.norm(); }
    int width() const { return cam_[0].width(); }
    int height() const { return cam_[0].height(); }

private:
    KannalaBrandt8 cam_[2];
    Eigen::Matrix3f R_12_ = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t_12_ = Eigen::Vector3f::Zero();
};

}  // namespace orb_matcher
