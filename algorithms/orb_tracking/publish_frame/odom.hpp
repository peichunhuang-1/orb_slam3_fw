#pragma once
// The continuous odom frame next to the atlas's map frame (not in ORB-SLAM3; algorithms/orb_tracking/docs/
// DESIGN.md, "輸出的座標系"). ORB-SLAM3 re-anchors its world (IMU initialization and VIBA rotate the whole map,
// BAs move keyframes and points, later loop closing and merges), and the pose it tracks is in whatever the world
// is then. Here the change of the map between two published frames is measured where the frame is localized:
//
//   the last published frame is localized twice with the same estimator (PoseOptimization) and the same
//   observations (its inlier features), against its points' positions when it was published and against their
//   positions now: D = Twc(now) · Twc(then)⁻¹ (map_now ← map_then), and T_map_odom = D · T_map_odom.
//
// A rigid change G of the map moves the estimate by exactly G (the cost is invariant to it), so D = G; a
// non-rigid one gives the rigid motion that best explains the change as these observations see it, which is how
// the current frame, tracked against the same points, sees it. Both solves use the same estimator, so its own
// bias cancels. Then the published odom motion between the two frames is Twc_last(now)⁻¹ · Twc_current(now):
// both in one version of the map.
//
// The second solve starts from the last frame moved with its reference keyframe (the anchor): a rotation of the
// whole map (IMU initialization: up to ~100°) is far outside the optimizer's reach from the old pose. With too
// few inliers in either solve D is the anchor's motion alone; without an anchor, I. A new map (after LOST, a
// reset) has no relation to the last one: odom goes on from the last published pose.
#include <Eigen/Geometry>

#include "types/map/atlas.hpp"
#include "types/tracking/frame.hpp"
#include "types/tracking/tracker.hpp"

namespace orb_tracking {

enum class OdomSource {
    first,        // the first published frame: odom = map
    unchanged,    // none of the last frame's points moved: D = I
    relocalized,  // D from the last frame localized again
    anchor,       // D from the reference keyframe's motion (too few inliers)
    none,         // points moved but neither works: D = I
    new_map,      // another map: odom goes on from the last pose
};

struct OdomUpdate {
    Eigen::Isometry3d T_map_odom;
    Eigen::Isometry3d T_odom_cam;  // the frame's left camera in odom
    OdomSource source = OdomSource::first;
    int inliers = 0;  // of the second solve when relocalized
};

constexpr int kOdomMinInliers = 30;  // as TrackLocalMap's for a tracked frame

// For frame f (with a pose, about to be published) in atlas a (the view it was tracked in): updates s and
// returns the transforms. a.current is f's map.
OdomUpdate updateOdom(tracking::OdomState& s, const tracking::Frame& f, const map::Atlas& a);

}  // namespace orb_tracking
