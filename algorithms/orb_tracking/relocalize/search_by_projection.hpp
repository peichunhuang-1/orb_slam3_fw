#pragma once
// ORB-SLAM3's ORBmatcher::SearchByProjection(Frame&, KeyFrame*, sAlreadyFound, th, ORBdist) (only Relocalization
// uses it, with the orientation check): the keyframe's good points not found yet, projected into the frame's left
// view at its pose ([0, w] x [0, h], no depth check, as the original), within their scale-invariance distance, the
// best free left feature within th · scale at the predicted level ±1, at most orb_dist; then the rotation check.
#include <set>

#include "types/map/atlas.hpp"
#include "types/tracking/frame.hpp"

namespace orb_tracking {

// Sets frame.points; returns the number of new matches
int searchByProjection(tracking::Frame& frame, const map::KeyFrame& kf, const map::Atlas& atlas,
                       const std::set<map::MapPointId>& already_found, float th, int orb_dist);

}  // namespace orb_tracking
