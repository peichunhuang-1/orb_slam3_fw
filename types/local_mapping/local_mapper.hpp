#pragma once
// Local mapping's state across ticks (resource type "local_mapping/LocalMapper"): ORB-SLAM3's LocalMapping
// members, by id (algorithms/orb_local_mapping/docs/DESIGN.md, "資料放在哪裡"). Only the local_mapping tree uses
// it. No mutex: fw's resource lock replaces mMutexNewKFs, mMutexReset, mMutexStop, mMutexAccept.
//
// The fields the tree's logic needs; the query scratch is filled in with the leaves.
#include <cstdint>
#include <deque>
#include <list>
#include <vector>

#include <Eigen/Core>

#include "fw/fw.hpp"
#include "types/ba/problem.hpp"
#include "types/geometry/se3.hpp"
#include "types/imu/preintegrated.hpp"
#include "types/map/ids.hpp"

namespace local_mapping {

enum class PendingReset { none, active_map, all };  // mbResetRequestedActiveMap / mbResetRequested

// A keyframe's scratch (KeyFrame::mnBAGlobalForKF and the GBA results). mnBALocalForKF and mnBAFixedForKF only
// matter within one LocalBundleAdjustment: sets in that call instead.
struct KeyFrameScratch {
    map::KeyFrameId ba_global_for = map::kNone;
    geometry::SE3f Tcw_gba, Tcw_before_gba;
    Eigen::Vector3f velocity_gba = Eigen::Vector3f::Zero(), velocity_before_gba = Eigen::Vector3f::Zero();
    imu::Bias bias_gba;
};

// A map point's scratch (MapPoint::mnBAGlobalForKF, mPosGBA)
struct PointScratch {
    map::KeyFrameId ba_global_for = map::kNone;
    Eigen::Vector3f pos_gba = Eigen::Vector3f::Zero();
};

// The optimization ba_solver is working on, and what writing it back needs
enum class Waiting { none, local_ba, imu_init_inertial, imu_init_full };
struct PendingOptimization {
    Waiting waiting = Waiting::none;
    std::uint64_t seq = 0;  // the problem's; a result with another one is dropped
    map::MapId map = map::kNone;  // the problem's map (the write-back skips what left it)
    // initialize_imu: the keyframes in temporal order (vpKF), and the first one's timestamp (mFirstTs)
    std::vector<map::KeyFrameId> imu_keyframes;
    double first_ts = 0;
};

struct LocalMapper {
    bool inertial = false;  // mbInertial (params.sensor)

    std::deque<map::KeyFrameId> queue;           // mlNewKeyFrames
    map::KeyFrameId current = map::kNone;        // mpCurrentKeyFrame
    std::list<map::MapPointId> recent_points;    // mlpRecentAddedMapPoints
    bool processing = false;                     // !mbAcceptKeyFrames
    bool abort_ba = false;                       // mbAbortBA
    PendingReset reset = PendingReset::none;
    map::MapId reset_map = map::kNone;           // mpMapToReset
    bool stop_requested = false, stopped = false, not_stop = false;  // loop closing (later)

    // IMU initialization
    float t_init = 0;       // mTinit
    bool bad_imu = false;   // mbBadImu
    bool initializing = false;  // bInitializing
    Eigen::Matrix3d Rwg = Eigen::Matrix3d::Identity();  // mRwg
    double scale = 1;                                   // mScale
    Eigen::Vector3d bg = Eigen::Vector3d::Zero(), ba = Eigen::Vector3d::Zero();  // mbg, mba
    int idx_init = 0;  // mIdxInit
    int n_kfs = 0;     // mnKFs

    // What tracking applies (tracking/MapperStatus): UpdateFrameIMU, and mState = OK after an initialization
    std::uint64_t imu_update_seq = 0;
    float imu_scale = 1;
    imu::Bias imu_bias;
    map::KeyFrameId imu_keyframe = map::kNone;
    std::uint64_t tracking_ok_seq = 0;

    bool far_points = false;  // mbFarPoints
    float th_far_points = 0;  // mThFarPoints

    std::uint64_t next_seq = 1;
    PendingOptimization pending;

    std::vector<KeyFrameScratch> keyframe_scratch;  // by keyframe id
    std::vector<PointScratch> point_scratch;        // by point id

    std::uint64_t processed = 0;  // keyframes processed (statistics)
};

}  // namespace local_mapping

FW_REGISTER_TYPE(local_mapping::LocalMapper, "local_mapping/LocalMapper")
