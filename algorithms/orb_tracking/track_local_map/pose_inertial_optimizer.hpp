#pragma once
// ORB-SLAM3's Optimizer::PoseInertialOptimizationLastKeyFrame and PoseInertialOptimizationLastFrame (only
// TrackLocalMap uses them): the frame's body pose, velocity and biases from its map points (left and right as
// separate monocular edges) and the IMU.
//
//   LastKeyFrame  the keyframe's state fixed; inertial edge = the frame's preintegration from the keyframe
//                 (frame.preintegrated); bias random-walk edges from its covariance
//   LastFrame     the previous frame's state free with its prior (prev.prior, Huber 5); inertial edge = this
//                 frame's own preintegration (frame.preintegrated_frame); the random-walk information still from
//                 frame.preintegrated, as the original; the previous frame is marginalized into the new prior and
//                 its prior is dropped
//
// The solver is g2o's Gauss-Newton (10 iterations per round, no stop rule) written out, with the original's
// details: the outlier test of round k (chi2 12, 7.5, 5.991, 5.991 for LastKeyFrame, 5.991 for LastFrame; 1.5x
// for points closer than 10 m, a point behind the camera is an outlier) reads the errors g2o computed before the
// round's last update; the Huber kernel goes after round 3; with fewer than 30 inliers (and !rec_init) points
// below chi2 18 are taken back. The prior left on the frame is H from the final linearization, symmetrized as
// written ((H + H) / 2) with eigenvalues below 1e-12 set to 0.
//
// par: the observations' errors and Jacobians of each iteration are computed on it (shared/parallel.hpp); their
// sums go chunk by chunk, so the result does not depend on the number of threads (it differs from a single sum
// over all edges by rounding only).
#include <vector>

#include <Eigen/Core>

#include "algorithms/orb_tracking/shared/parallel.hpp"
#include "types/camera/stereo_rig.hpp"
#include "types/imu/preintegrated.hpp"
#include "types/tracking/frame.hpp"

namespace orb_tracking {

// A fixed IMU state (the last keyframe's GetImuRotation, GetImuPosition, GetVelocity, bias)
struct ImuState {
    Eigen::Matrix3f Rwb = Eigen::Matrix3f::Identity();
    Eigen::Vector3f twb = Eigen::Vector3f::Zero(), vwb = Eigen::Vector3f::Zero();
    imu::Bias bias;
};

// world[i]: feature i's map point position (nullptr: none); track_depth[i]: its depth in the camera when the
// local map was projected (MapPoint::mTrackDepth; < 10 m is "close"). Both of size frame.size(). Sets the frame's
// pose, velocity, bias, outlier flags and prior; returns the inlier count.
int optimizePoseInertialLastKeyFrame(tracking::Frame& frame, const ImuState& keyframe, const camera::StereoRig& rig,
                                     const std::vector<const Eigen::Vector3f*>& world,
                                     const std::vector<float>& track_depth, bool rec_init = false,
                                     const ParallelFor& par = {});

// prev must carry its prior (prev.prior), which is reset
int optimizePoseInertialLastFrame(tracking::Frame& frame, tracking::Frame& prev, const camera::StereoRig& rig,
                                  const std::vector<const Eigen::Vector3f*>& world,
                                  const std::vector<float>& track_depth, bool rec_init = false,
                                  const ParallelFor& par = {});

}  // namespace orb_tracking
