#pragma once
// LoopClosing::MergeLocal2 (stereo-inertial map merging), run by local mapping (the atlas's only writer) once it
// has stopped and emptied its queue (algorithms/orb_loop_closing/docs/DESIGN.md, "local mapping 的樹"), in
// steps around the two optimizations ba_solver computes. m.merge holds the merge (types/local_mapping).
//
// alignMergeInertial  (the original up to InertialOptimization)
//   The current map, by T_old_new (mSold_new, scale 1), into the merge map's frame (ApplyScaledRotation, the
//   velocities rotated); UpdateFrameIMU(1, the current keyframe's bias, the tracker's last keyframe) through the
//   status (imu_update_seq). numKFnew: the current map's keyframes. When the current map has no BA2 yet: the
//   problem of InertialOptimization(pCurrentMap, bg, ba) (inertial_init with Rwg and the scale fixed: biases and
//   velocities; 200 iterations, lambda 1e3, priors 1e2 and 1e6, the biases starting at its first keyframe's, by
//   id: the original's GetAllKeyFrames goes by address).
// applyMergeBias  its write-back (velocities; the bias, the preintegration re-integrated when the gyro bias moved
//   by more than 0.01), UpdateFrameIMU(1, the bias, the last keyframe), BA2, BA1, SetImuInitialized.
// joinMaps  every keyframe and point of the merge map moves to the current map (which stays current); the merge
//   map's origin loses its first connection; the merge map set bad (SetMapBad: CountMaps drops; the original's
//   MergeLocal2 leaves it in the atlas, empty). The spanning tree from the matched keyframe up to its root
//   reversed, the matched keyframe a child of the current one. mvpMergeConnectedKFs (m.merge_connected_kfs,
//   cleared first: the original never clears it, appending to the first merge's) gets the matched
//   keyframe and its covisibles, the first 6 kept; the current keyframe's connections updated, it and its
//   covisibles (the first 6); SearchAndFuse of the points of the former (stopping once over 1000, after a whole
//   keyframe) into the latter by their poses (fuseWithPose, th 4), each found point replaced; the connections of
//   both updated. Fewer than 10 keyframes in the current map before: done (nullopt). Else the problem of
//   MergeInertialBA(the last keyframe, the matched keyframe):
//     - optimized: the last keyframe and its previous ones (mPrevKF), 6 in all; the previous one of the oldest
//       is a covisible one (pose, velocity and biases free; with none, the oldest becomes it); then the matched
//       keyframe and 2 previous ones, the previous one of the oldest fixed (with none, the oldest is fixed
//       itself), then its next ones up to 12 optimized in all. A keyframe the walk reaches twice (the matched
//       keyframe the last of its map) is taken once: the original adds it twice;
//     - local points: the good points of the optimized keyframes (per feature, left and right), each counted
//       once per feature that sees it. The original leaves out (only counts) the ones the last local BA of the
//       last keyframe took (mnBALocalForKF: m.point_scratch, by the local BA's problems), i.e. most of the
//       current side's: the current keyframes then move onto the fused points of the merge map and leave their
//       own behind (algorithms/orb_loop_closing/docs/DESIGN.md, 階段 4). Taken here; original_points (the
//       fixture test) keeps the original's;
//     - more covisible keyframes: the 30 points seen least (std::sort of the counts in id order, the
//       original's pointer order), each the first of its observers (by id) neither optimized nor fixed yet,
//       marked, and taken when good;
//     - an EdgeInertial from the previous keyframe to each optimized one (both bImu, with a preintegration), the
//       preintegration's SetNewBias(that keyframe's bias) on the atlas; one EdgeMono per left observation of a
//       local point by a good keyframe of the problem. A right-only observation is left out: the original reads
//       the left feature -1 there (undefined);
//     - 8 iterations from lambda 1e3. Points it took get mnBALocalForKF = the last keyframe.
// applyMergeInertialBA  the outlier observations erased (unless the point went bad); a result with a non-finite
//   estimate stops there (the reference's local change); poses, velocities and biases (SetNewBias); points
//   (UpdateNormalAndDepth); the map's change index.
#include <optional>

#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace orb_loop_closing {

std::optional<ba::Problem> alignMergeInertial(map::Atlas& a, local_mapping::LocalMapper& m);
void applyMergeBias(map::Atlas& a, local_mapping::LocalMapper& m, const ba::Result& r);
std::optional<ba::Problem> joinMaps(map::Atlas& a, local_mapping::LocalMapper& m, bool original_points = false);
ba::Problem mergeInertialBAProblem(map::Atlas& a, local_mapping::LocalMapper& m, map::KeyFrameId last_kf,
                                   map::KeyFrameId merge_kf, bool original_points = false);
void applyMergeInertialBA(map::Atlas& a, map::MapId map, const ba::Result& r);

}  // namespace orb_loop_closing
