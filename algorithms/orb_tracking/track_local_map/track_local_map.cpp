// track_local_map: Tracking::TrackLocalMap. The local map (keyframes sharing points with the frame and their
// neighbours, their points), its points projected and matched, then the pose:
//   - PoseOptimization before the IMU initialization and up to mnFramesToResetIMU frames after a relocalization;
//   - else the pose-inertial optimization from the last frame when the map did not change (its prior: the last
//     frame's), from the last keyframe otherwise.
// Where the original would dereference a missing prior or preintegration (it crashes), the next one down is used:
// LastKeyFrame without the last frame's prior, PoseOptimization without the frame's preintegration.
// Then the points' found counts, the inliers (matches to points with observations; stereo: outliers leave the
// frame), and success: at least 50 inliers within mMaxFrames of a relocalization, more than 10 when RECENTLY_LOST,
// else at least 15 (inertial) or 30 (stereo).
#include <vector>

#include "algorithms/orb_tracking/shared/pose_optimizer.hpp"
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "algorithms/orb_tracking/track_local_map/local_map.hpp"
#include "algorithms/orb_tracking/track_local_map/pose_inertial_optimizer.hpp"

namespace {

using tracking::State;
using namespace orb_tracking;

void optimize(const map::Atlas& a, tracking::Tracker& t) {
    tracking::Frame& f = t.current;
    if (!imuInitialized(a) || f.id <= t.last_reloc_frame_id + t.frames_to_reset_imu) {
        optimizePose(f, a);
        return;
    }
    const std::vector<const Eigen::Vector3f*> world = worldPositions(a, f.points, false);
    std::vector<float> depth(f.points.size(), 0.f);
    for (std::size_t i = 0; i < f.points.size(); ++i)
        if (f.points[i] != map::kNone) depth[i] = pointScratch(t, f.points[i]).depth;
    const camera::StereoRig& rig = rigOf(a, f);

    if (!t.map_updated && t.last.prior && f.preintegrated_frame && f.preintegrated) {
        optimizePoseInertialLastFrame(f, t.last, rig, world, depth);
        return;
    }
    const map::KeyFrame* kf = a.keyframe(f.last_keyframe);
    if (kf && f.preintegrated) {
        ImuState s;
        s.Rwb = kf->imuRotation();
        s.twb = kf->imuPosition();
        s.vwb = kf->velocity();
        s.bias = kf->bias;
        optimizePoseInertialLastKeyFrame(f, s, rig, world, depth);
        return;
    }
    optimizePose(f, a);
}

fw::bt::Status track_local_map(const State& state, fw::Optional<const tracking::MapperStatus&> mapper,
                               fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    const map::Atlas& a = tracker->view;
    tracking::Frame& f = t.current;

    updateLocalMap(a, t);
    searchLocalPoints(a, t, state, mapperOf(mapper));
    optimize(a, t);

    // The points' statistics
    t.matches_inliers = 0;
    for (std::size_t i = 0; i < f.points.size(); ++i) {
        if (f.points[i] == map::kNone) continue;
        const map::MapPoint* p = a.point(f.points[i]);
        if (!f.outlier[i]) {
            if (p) {
                ++t.counts[f.points[i]].found;  // IncreaseFound
                if (p->n_obs > 0) ++t.matches_inliers;
            }
        } else if (!t.inertial) {
            f.points[i] = map::kNone;
        }
    }

    const int n = t.matches_inliers;
    if (f.id < t.last_reloc_frame_id + t.max_frames && n < 50) return fw::bt::Status::FAILURE;
    if (n > 10 && state == State::RECENTLY_LOST) return fw::bt::Status::SUCCESS;
    return n >= (t.inertial ? 15 : 30) ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_ACTION(track_local_map,
                      INPUTS(("state", tracking::State, latest), ("mapper", tracking::MapperStatus, latest)),
                      RESOURCES(("tracker", tracking::Tracker, write)));
