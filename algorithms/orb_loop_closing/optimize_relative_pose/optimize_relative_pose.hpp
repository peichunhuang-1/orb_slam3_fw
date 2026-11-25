#pragma once
// optimizeRelativePose: ORB-SLAM3's Optimizer::OptimizeSim3 with bFixScale = true (stereo, stereo-inertial), so
// the vertex is a rigid transform T12 (kf2's camera into kf1's); algorithms/orb_loop_closing/docs/DESIGN.md,
// "以 SE3 取代 Sim3".
//
// Per kf1 feature i with a match: kf1's own point and the matched one, both fixed, taken into kf1's / kf2's
// camera with the keyframes' poses. Two edges per correspondence:
//   e12  kf1's left keypoint i  vs  KB8(T12 · X2)
//   e21  kf2's left keypoint of the matched point  vs  KB8(T12⁻¹ · X1)
// Skipped as the original: no point of kf1's own (but the match stays), a bad point, a right-view feature
// (i >= n_left), a matched point not in kf2's left view when !all_points, X2 behind kf2.
// When the matched point is not in kf2's left view (all_points), e21 measures the point's own projection into
// kf2 at the level predicted from its distance to kf2 (the original takes mnTrackScaleLevel, tracking's scratch
// of the last frame that saw it; the level only sets the edge's information).
//
// The solver is g2o's Levenberg-Marquardt written out as in orb_tracking/shared/pose_optimizer (the original's
// 7-dof vertex with the scale update zeroed is this 6-dof one), Huber δ = √th2: 5 iterations; the pairs with either chi2 over th2 removed from matches and from the
// problem, the others without the kernel; fewer than 10 left: returns 0; else 10 more iterations (5 if none was
// removed), and the pairs over th2 again removed from matches. Returns the inliers.
// Unlike the original, the error and an analytic Jacobian come from the KB8 model in double (project_jacobian_d).
// The original differentiates its float projection numerically with δ = 1e-9, below float rounding: its Jacobian
// is ~0, the LM steps barely move T12 off the RANSAC guess, and wrong loop corrections follow.
#include <vector>

#include "types/geometry/se3.hpp"
#include "types/map/atlas.hpp"

namespace orb_loop_closing {

// matches: per kf1 feature (size kf1.size()), kf2's matched point, in and out. T12: the initial estimate, then
// the optimized one. unseen_level: the level of e21 for a point not in kf2's left view (-1: predicted from its
// distance to kf2; the tests give the fixture's mnTrackScaleLevel, 0).
int optimizeRelativePose(const map::Atlas& atlas, const map::KeyFrame& kf1, const map::KeyFrame& kf2,
                         std::vector<map::MapPointId>& matches, geometry::SE3d& T12, float th2, bool all_points,
                         int unseen_level = -1);

}  // namespace orb_loop_closing
