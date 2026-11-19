// track_with_motion_model: Tracking::UpdateLastFrame and TrackWithMotionModel. The last frame's pose again from
// its reference keyframe; with the IMU initialized (and no recent relocalization) the pose is the IMU prediction
// and nothing else is done. Else the constant-velocity pose, the last frame's points projected (th 7 stereo, 15
// inertial; twice that with fewer than 20 matches), optimized, outliers out. Stereo: at least 10 matches to
// points with observations; inertial: success even with few matches.
#include <algorithm>

#include "algorithms/orb_tracking/shared/pose_optimizer.hpp"
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "algorithms/orb_tracking/track_with_motion_model/search_by_projection.hpp"

namespace {

using namespace orb_tracking;

fw::bt::Status track_with_motion_model(fw::Write<map::Atlas> atlas, fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    const map::Atlas& a = *atlas;
    tracking::Frame& f = t.current;

    // UpdateLastFrame (the visual odometry points are for localization mode only)
    if (const map::KeyFrame* ref = a.keyframe(t.last.reference_kf); ref && !t.relative_poses.empty())
        t.last.setPose(t.relative_poses.back() * ref->pose());

    if (imuInitialized(a) && f.id > t.last_reloc_frame_id + t.frames_to_reset_imu) {
        predictStateImu(a, t);
        return fw::bt::Status::SUCCESS;
    }
    f.setPose(t.velocity * t.last.pose());

    std::fill(f.points.begin(), f.points.end(), map::kNone);
    const float th = t.inertial ? 15.0f : 7.0f;
    int nmatches = searchByProjection(f, t.last, a, th);
    if (nmatches < 20) {
        std::fill(f.points.begin(), f.points.end(), map::kNone);
        nmatches = searchByProjection(f, t.last, a, 2 * th);
    }
    if (nmatches < 20) return t.inertial ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;

    optimizePose(f, a);
    const int nmatches_map = discardPoseOutliers(a, t, nmatches, OutlierRule::motion_model);
    if (t.inertial) return fw::bt::Status::SUCCESS;
    return nmatches_map >= 10 ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_ACTION(track_with_motion_model,
                      RESOURCES(("atlas", map::Atlas, write), ("tracker", tracking::Tracker, write)));
