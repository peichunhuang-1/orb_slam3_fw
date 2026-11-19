#pragma once
// ORB-SLAM3's ORBmatcher::SearchByProjection(Frame& current, const Frame& last, th) for KB8 stereo (only
// TrackWithMotionModel uses it, nn_ratio 0.9 unused, with the orientation check): the last frame's points (not
// outliers, bad ones too) projected into the current frame at its pose, the best descriptor within th · scale of
// the last keypoint's octave, at most TH_HIGH (100). As the original:
//   - the octave window follows the motion along the optical axis (forward: >= the octave, backward: <=, else ±1);
//   - the left projection must be in [0, w] x [0, h] in front of the camera, else the right one is not tried;
//   - the right view is tried after the left whatever the left found, projected with the left camera's model
//     (mpCamera) at Trl · Pc, without bounds check;
//   - a current feature already holding a point with observations is skipped;
//   - then the rotation consistency check.
#include "types/map/atlas.hpp"
#include "types/tracking/frame.hpp"

namespace orb_tracking {

// Sets current.points; returns the number of matches
int searchByProjection(tracking::Frame& current, const tracking::Frame& last, const map::Atlas& atlas, float th);

}  // namespace orb_tracking
