#pragma once
// Optimizer::LocalBundleAdjustment(pKF, …) without the optimization itself (ba_solver's `local` kind): the
// window and the copy of the problem under the lock, and the write-back by id.
//
// Window, as the original:
//   - local keyframes: pKF, then its covisibles (GetVectorCovisibleKeyFrames, best first) that are good and in
//     its map; the map's init keyframe among them is held fixed;
//   - local points: the good points in that map seen by the local keyframes, in their feature order;
//   - fixed keyframes: the other good keyframes of the map that see a local point (a covisible left out for
//     being bad or in another map is not taken as fixed either, the original's mnBALocalForKF);
//   - no fixed keyframe at all (no fixed camera and the init keyframe not local): no problem, the original
//     returns before optimizing.
// mnBALocalForKF / mnBAFixedForKF only serve within this call, so they are sets here.
// Every observation of a local point by a local or fixed keyframe is a reprojection: the left feature in the
// left camera, the right one in the right camera (the KB8 rig has no uRight: no stereo edge). The initial
// lambda is 100 when the map is inertial (setUserLambdaInit), else g2o's.
//
// Write-back (under the lock, after the solve; tracking may have changed the atlas meanwhile):
//   - outlier observations erased (EraseMapPointMatch, EraseObservation) unless the point is bad by now;
//   - poses of the optimized keyframes still good and in the map; positions of the points still in the map
//     (bad ones too, as the original), then UpdateNormalAndDepth; the map's change index.
#include <optional>

#include "types/ba/problem.hpp"
#include "types/map/atlas.hpp"

namespace orb_local_mapping {

std::optional<ba::Problem> localBAProblem(const map::Atlas& a, map::KeyFrameId kf);

void applyLocalBA(map::Atlas& a, map::MapId map, const ba::Result& r);

}  // namespace orb_local_mapping
