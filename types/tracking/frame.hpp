#pragma once
// Tracking's frame (ORB-SLAM3's Frame for the KB8 stereo constructor): the features (shared with the
// keyframe made from it, orb::FrameFeatures), the map point and outlier flag per feature, the pose and
// velocity, and the IMU state. Map points and keyframes are referred to by id.
//
// Feature index i: i < nLeft() left keypoint i, else right keypoint i - nLeft() (N = Nleft + Nright).
// ORB-SLAM3's per-frame copies of the calibration (fx, mK, mbf, the grid bounds) are in the rig and the
// features; its mpPrevFrame is the tracker's last frame.
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "types/geometry/se3.hpp"
#include "types/imu/preintegrated.hpp"
#include "types/map/ids.hpp"
#include "types/map/vocabulary.hpp"
#include "types/orb/frame_features.hpp"

namespace tracking {

// The prior a pose-inertial optimization leaves on the frame for the next one (ORB-SLAM3's
// ConstraintPoseImu, mpcpi): the state and its 15x15 information (rotation, translation, velocity, gyro bias,
// acc bias), H as the optimizer made it (symmetrized, small eigenvalues zeroed)
struct PoseImuPrior {
    Eigen::Matrix3d Rwb = Eigen::Matrix3d::Identity();
    Eigen::Vector3d twb = Eigen::Vector3d::Zero(), vwb = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero(), ba = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 15, 15> H = Eigen::Matrix<double, 15, 15>::Zero();
};

struct Frame {
    long id = -1;          // mnId; -1: no frame (ORB-SLAM3's Frame())
    double timestamp = 0;  // s

    // Features, the stereo rig (atlas rigs[rig]; Tlr right camera in the left, Trl its inverse) and depths
    std::shared_ptr<const orb::FrameFeatures> features;
    int rig = -1;
    geometry::SE3f Tlr, Trl;
    float baseline = 0;  // mb (m)
    float th_depth = 0;  // mThDepth (m): close / far points
    map::BowVector bow;
    map::FeatureVector feat;

    // Per feature
    std::vector<map::MapPointId> points;  // mvpMapPoints (map::kNone: none)
    std::vector<bool> outlier;            // mvbOutlier

    map::KeyFrameId reference_kf = map::kNone;   // mpReferenceKF
    map::KeyFrameId last_keyframe = map::kNone;  // mpLastKeyFrame (set by the IMU preintegration step)

    // IMU: T_cb (body in the left camera) when the rig has an IMU
    bool has_imu_calib = false;
    geometry::SE3f T_cb;
    imu::Bias bias;  // mImuBias
    // mpImuPreintegrated: from the last keyframe (the tracker's, shared); mpImuPreintegratedFrame: from the last
    // frame (this frame's own, from imu_preintegration at zero bias)
    std::shared_ptr<imu::Preintegrated> preintegrated;
    std::shared_ptr<imu::Preintegrated> preintegrated_frame;
    bool imu_preintegrated = false;     // mbImuPreintegrated
    std::optional<PoseImuPrior> prior;  // mpcpi

    bool valid() const { return id >= 0 && features != nullptr; }
    int size() const { return features ? features->size() : 0; }  // N
    int nLeft() const { return features ? features->n_left : 0; }

    // Pose
    void setPose(const geometry::SE3f& Tcw) {
        Tcw_ = Tcw;
        updatePoseMatrices();
        has_pose_ = true;
    }
    // From the body pose and velocity (SetImuPoseVelocity): Tcw = Tcb · Twb⁻¹
    void setImuPoseVelocity(const Eigen::Matrix3f& Rwb, const Eigen::Vector3f& twb, const Eigen::Vector3f& vwb) {
        Vw_ = vwb;
        has_velocity_ = true;
        Tcw_ = T_cb * geometry::SE3f(Rwb, twb).inverse();
        updatePoseMatrices();
        has_pose_ = true;
    }
    bool hasPose() const { return has_pose_; }
    const geometry::SE3f& pose() const { return Tcw_; }
    const Eigen::Matrix3f& Rwc() const { return Rwc_; }
    const Eigen::Matrix3f& Rcw() const { return Rcw_; }
    const Eigen::Vector3f& tcw() const { return tcw_; }
    const Eigen::Vector3f& cameraCenter() const { return Ow_; }  // mOw
    Eigen::Vector3f imuPosition() const { return Rwc_ * T_cb.translation() + Ow_; }
    Eigen::Matrix3f imuRotation() const { return Rwc_ * T_cb.rotationMatrix(); }
    geometry::SE3f imuPose() const { return Tcw_.inverse() * T_cb; }  // Twb
    void setVelocity(const Eigen::Vector3f& v) {
        Vw_ = v;
        has_velocity_ = true;
    }
    const Eigen::Vector3f& velocity() const { return Vw_; }
    bool hasVelocity() const { return has_velocity_; }
    // SetNewBias: the bias and that of the preintegration from the last keyframe
    void setNewBias(const imu::Bias& b) {
        bias = b;
        if (preintegrated) preintegrated->setNewBias(b);
    }
    // World point of a left feature's stereo point (UnprojectStereoFishEye)
    Eigen::Vector3f unprojectStereo(int i) const { return Rwc_ * features->stereoPoint(i) + Ow_; }
    void computeBoW(const map::Vocabulary& voc);  // if not done yet

private:
    void updatePoseMatrices() {
        const geometry::SE3f Twc = Tcw_.inverse();
        Rwc_ = Twc.rotationMatrix();
        Ow_ = Twc.translation();
        Rcw_ = Tcw_.rotationMatrix();
        tcw_ = Tcw_.translation();
    }

    geometry::SE3f Tcw_;
    Eigen::Matrix3f Rwc_ = Eigen::Matrix3f::Identity(), Rcw_ = Eigen::Matrix3f::Identity();
    Eigen::Vector3f Ow_ = Eigen::Vector3f::Zero(), tcw_ = Eigen::Vector3f::Zero();
    bool has_pose_ = false;
    Eigen::Vector3f Vw_ = Eigen::Vector3f::Zero();
    bool has_velocity_ = false;
};

// The frame of a stereo message (Frame's KB8 stereo constructor): no map points, no outliers, no pose
struct FrameSetup {
    long id = 0;
    double timestamp = 0;
    int rig = -1;
    geometry::SE3f Tlr;
    float baseline = 0, th_depth = 0;
    bool has_imu_calib = false;
    geometry::SE3f T_cb;
};
Frame makeFrame(std::shared_ptr<const orb::StereoFrame> stereo, const FrameSetup& setup);

}  // namespace tracking
