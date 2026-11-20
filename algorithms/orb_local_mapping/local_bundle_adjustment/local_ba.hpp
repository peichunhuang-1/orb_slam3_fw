#pragma once
// Optimizer::LocalBundleAdjustment(pKF, …) and Optimizer::LocalInertialBA(pKF, …, bLarge, bRecInit) without the
// optimization itself (ba_solver's `local` and `local_inertial` kinds): the window and the copy of the problem
// under the lock, and the write-back by id.
//
// LocalBundleAdjustment's window, as the original:
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
//   - poses of the optimized keyframes still good and in the map, and for an inertial one its velocity and
//     bias (SetNewBias); positions of the points still in the map (bad ones too, as the original), then
//     UpdateNormalAndDepth; the map's change index.
//
// LocalInertialBA's window, as the original:
//   - optimized: pKF, then its previous keyframes (mPrevKF, good or not), Nd in all, Nd = min(keyframes in pKF's
//     map - 2, 25 when bLarge else 10);
//   - fixed: the previous keyframe of the oldest one; with none, the oldest one itself leaves the window and is
//     fixed (after its points were taken);
//   - local points: the good points seen by the optimized keyframes, in their feature order (in any map);
//   - more fixed keyframes: per local point, the first of its observers (by id; the original's KeyFrame* order)
//     neither optimized nor fixed yet is marked fixed, and the point's loop stops at the first good one, which
//     is taken; at most 200 fixed in all (counted with any taken from another map, which go into no edge and so
//     are left out of the problem). maxCovKF = 0: no covisible keyframe is optimized;
//   - a keyframe with bImu has a velocity and a bias in the problem, held fixed with a fixed pose;
//   - observations as LocalBundleAdjustment's, by optimized and fixed keyframes that are good and in pKF's map; a
//     right one's information is that of the left feature's level when the point has one in that keyframe, else
//     level 0's (the original reads the left keypoint's octave, a default KeyPoint without one);
//   - mTrackDepth (tracking's depth of the point when it last projected it, "close" below 10 m) is not seen by
//     local mapping: a point is close when within 10 m of pKF's camera centre;
//   - the original's SetNewBias(the previous keyframe's bias) on each inertial edge's preintegration is done on
//     the atlas here (it stays when the BA fails the convergence check);
//   - 4 iterations from lambda 1e-2 when bLarge, else 10 from 1.
// The write-back is applyLocalBA's (nothing when the result is not ok: "FAIL LOCAL-INERTIAL BA").
#include <optional>

#include "types/ba/problem.hpp"
#include "types/map/atlas.hpp"

namespace orb_local_mapping {

std::optional<ba::Problem> localBAProblem(const map::Atlas& a, map::KeyFrameId kf);

std::optional<ba::Problem> localInertialBAProblem(map::Atlas& a, map::KeyFrameId kf, bool large, bool rec_init);

void applyLocalBA(map::Atlas& a, map::MapId map, const ba::Result& r);

}  // namespace orb_local_mapping
