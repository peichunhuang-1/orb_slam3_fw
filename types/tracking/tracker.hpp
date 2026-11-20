#pragma once
// Tracking's state across frames (resource type "tracking/Tracker"): ORB-SLAM3's Tracking members other than
// mState, which is on the tree's blackboard (algorithms/orb_tracking/docs/DESIGN.md, "資料放在哪裡"). Keyframes
// and points are referred to by id. Settings (fps, ThDepth, ...) are in the params, not here.
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "fw/fw.hpp"
#include "types/calib/stereo_imu_rig.hpp"
#include "types/geometry/se3.hpp"
#include "types/imu/preintegrated.hpp"
#include "types/map/atlas.hpp"
#include "types/map/ids.hpp"
#include "types/tracking/atlas_edits.hpp"
#include "types/tracking/frame.hpp"
#include "types/tracking/state.hpp"

namespace tracking {

enum class PendingReset { none, active_map, all };  // System's mbResetActiveMap / mbReset

// A keyframe's relocalization query scratch (KeyFrame::mnRelocQuery, mnRelocWords, mRelocScore). Kept across
// queries as there: a candidate's neighbour may carry the score of an earlier query.
struct RelocScratch {
    long query = 0;
    int words = 0;
    float score = 0;
};

// A map point's projection in the current frame (MapPoint::mbTrackInView(R), mTrackProjX(R), mTrackProjY(R),
// mnTrackScaleLevel(R), mTrackViewCos(R), mTrackDepth) and mnLastFrameSeen
struct PointScratch {
    long last_frame_seen = 0;
    bool in_view = false, in_view_r = false;
    float proj_x = 0, proj_y = 0, proj_xr = 0, proj_yr = 0;
    int level = -1, level_r = -1;
    float view_cos = 0, view_cos_r = 0;
    float depth = 0;  // left camera, distance (norm)
};

struct Tracker {
    // The atlas as tracking sees it (types/map/docs/DESIGN.md, "tracking 改成只讀"): copied from the resource at
    // the start of each frame (sync_atlas), with the edits it does not have yet applied again. Read it as
    // const map::Atlas&; change it only through orb_tracking::addEdit, which also sends the change.
    map::Atlas view;
    std::vector<std::shared_ptr<const AtlasEdit>> pending;  // made, not in the resource yet (in order)
    std::uint64_t next_edit_seq = 1;
    std::map<map::MapPointId, PointCounts> counts;  // since the last keyframe: sent with the next one
    std::map<map::MapId, int> change_notified;      // Map::mnMapChangeNotified (only tracking uses it)

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

    // Settings of the current run (make_frame, from the params and the sensor): System::IMU_STEREO, mMaxFrames,
    // mnFramesToResetIMU
    bool inertial = false;
    int max_frames = 20;
    int frames_to_reset_imu = 20;

    // Per map point, the projection scratch ORB-SLAM3 keeps on MapPoint (by point id; missing entries are its
    // initial values). It persists across frames as there: a pose-estimation outlier is marked seen
    // (mnLastFrameSeen) and not projected again in the local map, but keeps the view flags of the projection
    // before, and the pose-inertial optimization reads mTrackDepth from the last projection.
    std::vector<PointScratch> point_scratch;
    std::vector<RelocScratch> reloc;  // by keyframe id

    bool new_keyframe = false;  // a keyframe was made this frame (the status)
};

}  // namespace tracking

FW_REGISTER_TYPE(tracking::Tracker, "tracking/Tracker")
