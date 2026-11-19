#pragma once
// Tracking's state across frames (resource type "tracking/Tracker"): ORB-SLAM3's Tracking members other than
// mState, which is on the tree's blackboard (algorithms/orb_tracking/docs/DESIGN.md, "資料放在哪裡"). Keyframes
// and points are referred to by id. Settings (fps, ThDepth, ...) are in the params, not here.
#include <cstdint>
#include <memory>
#include <vector>

#include "fw/fw.hpp"
#include "types/calib/stereo_imu_rig.hpp"
#include "types/geometry/se3.hpp"
#include "types/imu/preintegrated.hpp"
#include "types/map/ids.hpp"
#include "types/tracking/frame.hpp"
#include "types/tracking/state.hpp"

namespace tracking {

enum class PendingReset { none, active_map, all };  // System's mbResetActiveMap / mbReset

struct Tracker {
    Frame current, last;  // mCurrentFrame, mLastFrame
    State last_processed_state = State::NO_IMAGES_YET;
    long next_frame_id = 0;  // Frame::nNextId

    // Trajectory: per frame, its pose relative to its reference keyframe (mlRelativeFramePoses, mlpReferences,
    // mlFrameTimes, mlbLost)
    std::vector<geometry::SE3f> relative_poses;
    std::vector<map::KeyFrameId> references;
    std::vector<double> frame_times;
    std::vector<bool> lost;

    // Motion model
    bool has_velocity = false;  // mbVelocity
    geometry::SE3f velocity;    // mVelocity: Tcl of the last two frames

    map::KeyFrameId reference_kf = map::kNone;   // mpReferenceKF
    map::KeyFrameId last_keyframe = map::kNone;  // mpLastKeyFrame
    long last_keyframe_frame_id = 0;             // mnLastKeyFrameId
    long last_reloc_frame_id = 0;                // mnLastRelocFrameId
    std::vector<map::KeyFrameId> local_keyframes;  // mvpLocalKeyFrames
    std::vector<map::MapPointId> local_points;     // mvpLocalMapPoints
    std::vector<map::MapPointId> temporal_points;  // mlpTemporalPoints
    int matches_inliers = 0;                       // mnMatchesInliers

    // IMU
    std::shared_ptr<imu::Preintegrated> preintegrated_from_kf;  // mpImuPreintegratedFromLastKF
    imu::Noise imu_noise;                                       // mpImuCalib's
    imu::Bias last_bias;                                        // mLastBias
    std::uint64_t applied_imu_update = 0;  // the last MapperStatus::imu_update_seq applied (UpdateFrameIMU)
    long first_imu_frame_id = 0;           // mnFirstImuFrameId

    bool map_updated = false;  // mbMapUpdated
    bool created_map = false;  // mbCreatedMap
    bool vo = false;           // mbVO
    double timestamp_lost = 0;  // mTimeStampLost
    long first_frame_id = 0;    // mnFirstFrameId
    long initial_frame_id = 0;  // mnInitialFrameId
    long last_init_frame_id = 0;  // mnLastInitFrameId

    // The rig the frames are made with (newParameterLoader): rebuilt when the calib changes
    bool has_calib = false;
    calib::StereoImuRig calib;
    int rig = -1;  // atlas rigs index
    geometry::SE3f Tlr;
    float baseline = 0, th_depth = 0;
    geometry::SE3f T_cb;

    PendingReset pending_reset = PendingReset::none;
};

}  // namespace tracking

FW_REGISTER_TYPE(tracking::Tracker, "tracking/Tracker")
