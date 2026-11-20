#pragma once
// LocalMapping::SearchInNeighbors and what only it uses: ORBmatcher::Fuse(pKF, vpMapPoints, th, bRight) (the
// Sim3 overload is loop closing's).
//
// Reproduced as written:
// - the "right camera" calls are Fuse(pKF, points, true), which binds true to th, not to bRight: a second fuse
//   in the left camera with th = 1. The right camera is never fused into, so fuse has no right-camera case;
// - the point lists are taken once (the current keyframe's before the forward fuse, the targets' before the
//   backward one), so a point replaced on the way is skipped by its bad flag;
// - a match whose feature holds a bad point counts as fused and changes nothing.
// ORB-SLAM3 marks the targets and candidates on the keyframes and points (mnFuseTargetForKF,
// mnFuseCandidateForKF); here they are sets of the call, the same since a keyframe is current once.
#include <vector>

#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace orb_local_mapping {

// ORBmatcher::Fuse with bRight = false: each good point not already in keyframe kf is projected into its left
// camera; in the image, in the scale range, at most 60° off its normal, the best descriptor (Hamming <= TH_LOW,
// 50) among the left features within th · scale of the predicted level, at that level or one below and inside
// the χ² 5.99 error, gets it: the feature's point is replaced by the one with more observations, or the point is
// added there. Returns how many points were matched.
int fuse(map::Atlas& a, map::KeyFrameId kf, const std::vector<map::MapPointId>& points, float th);

// The keyframes SearchInNeighbors fuses with, in order: the 10 best covisibles, then the 20 best covisibles of
// each of those (not the current keyframe; stops after the first whose turn sees abort_ba), then when inertial
// the previous keyframes while there are fewer than 20. Each once, bad ones left out.
std::vector<map::KeyFrameId> fuseTargets(const map::Atlas& a, map::KeyFrameId cur, bool inertial, bool abort_ba);

// SearchInNeighbors for local_mapper.current: its points fused into each target (th 3, then 1), then (unless
// abort_ba) the targets' points into it the same way; its points' descriptors, normals and depths and its
// covisibility updated.
void searchInNeighbors(map::Atlas& a, const local_mapping::LocalMapper& m);

}  // namespace orb_local_mapping
