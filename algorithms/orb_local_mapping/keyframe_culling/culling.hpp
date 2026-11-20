#pragma once
// LocalMapping::KeyFrameCulling for local_mapper.current.
//
// Reproduced as written:
// - the covisibles are listed once (after UpdateBestCovisibles), so one culled on the way is still counted;
// - inertial: the keyframe is unlinked from the prev/next chain (its preintegration merged into the next one's)
//   before SetBadFlag, which a not_erase keyframe ignores: it stays good, out of the chain.
// Differs: ORB-SLAM3 reads mvDepth[i] for right features too, past its end (mvDepth has Nleft entries); here a
// right feature has no depth, so it is skipped as a far one.
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace orb_local_mapping {

// Each good covisible of the current keyframe but its map's first is culled (setKeyFrameBad) when more than 90%
// (inertial: 50%) of its close points are seen by more than 3 other keyframes at its level + 1 or finer.
// Inertial, it is culled only when the map has more than 21 keyframes, it is 2 or more before the current one, it
// has a previous and a next keyframe and the time between those is under 0.5 s, or under 3 s when the IMU is
// initialized and it is older than the last 21 keyframes, or under 3 s and its IMU within 2 cm of the previous
// one's before the second inertial BA. Stops after 20 covisibles when the BA was aborted, or after 100.
// Returns how many were culled (a not_erase one is only marked to_be_erased).
int keyFrameCulling(map::Atlas& a, const local_mapping::LocalMapper& m);

}  // namespace orb_local_mapping
