#pragma once
// Calibration of a stereo camera + IMU rig (in-process type "calib/StereoImuRig"): the intrinsics of both cameras,
// the extrinsics of every sensor in the body frame, and the IMU noise model. Sent once by euroc_mcap_driver
// (port "calib") before the first image; connect it to a `latest` port.
//
// Transform convention: T_a_b maps a point from frame b into frame a, p_a = T_a_b * p_b (the pose of b in a).
// EuRoC's sensor.yaml T_BS is T_body_sensor; /tf_static's body -> cam0 is the same transform.
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "fw/fw.hpp"

namespace calib {

// Pinhole intrinsics with lens distortion, as sensor_msgs/CameraInfo has them for the raw (distorted) image.
// EuRoC: distortion_model "plumb_bob" (radial-tangential), d = k1 k2 p1 p2 k3.
struct PinholeCamera {
    std::string frame_id;  // e.g. "cam0", the frame_id of the images
    int width = 0;
    int height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    std::string distortion_model;
    std::vector<double> d;

    Eigen::Matrix3d K() const {
        Eigen::Matrix3d k;
        k << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        return k;
    }
};

// Continuous-time IMU noise (Kalibr / ORB-SLAM3 convention). No rate: the preintegration discretizes it at
// each step's dt, from the sample timestamps (types/imu/preintegrated.hpp).
struct ImuNoise {
    double gyro_noise = 0;   // rad / s / sqrt(Hz)
    double gyro_walk = 0;    // rad / s^2 / sqrt(Hz)
    double accel_noise = 0;  // m / s^2 / sqrt(Hz)
    double accel_walk = 0;   // m / s^3 / sqrt(Hz)
};

struct StereoImuRig {
    std::string body_frame;  // parent frame of the extrinsics, e.g. "body"
    std::string imu_frame;   // e.g. "imu0"
    PinholeCamera left, right;
    Eigen::Isometry3d T_body_left = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_body_right = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_body_imu = Eigen::Isometry3d::Identity();
    ImuNoise imu;

    // Derived extrinsics
    Eigen::Isometry3d T_right_left() const { return T_body_right.inverse() * T_body_left; }  // stereo extrinsics
    Eigen::Isometry3d T_imu_left() const { return T_body_imu.inverse() * T_body_left; }      // ORB-SLAM3 IMU.T_b_c1
    Eigen::Isometry3d T_imu_right() const { return T_body_imu.inverse() * T_body_right; }
    double baseline() const { return T_right_left().translation().norm(); }  // metres
};

}  // namespace calib

FW_REGISTER_TYPE(calib::StereoImuRig, "calib/StereoImuRig")
