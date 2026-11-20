#pragma once
// The `local_inertial` problem (Optimizer::LocalInertialBA's optimization, after local mapping chose the window):
//   - per keyframe a body pose (ImuPoseManifold), and for one with imu set (bImu) a velocity and gyro / acc
//     biases; all of a fixed keyframe's held fixed;
//   - an EdgeInertial from each keyframe's previous one when both have imu set, with gyro and acc random-walk
//     edges (information C(9:12)⁻¹, C(12:15)⁻¹); the one from a fixed keyframe (the window's oldest) has its
//     information scaled by 1e-2, and it, or every one with p.rec_init, a Huber kernel (δ = √16.92);
//   - points eliminated, one EdgeMono per observation (Huber √5.991, information as the problem gives it);
//   - p.iterations LM iterations from p.initial_lambda.
// Not ok ("FAIL LOCAL-INERTIAL BA", nothing written back) when, without p.large, the robust cost after is more
// than twice the cost before, or either is NaN; the costs are g2o's activeRobustChi2 (the errors last computed,
// in float). Outliers: chi2 > 5.991 (8.9865 for a close point) or the point behind that camera.
// Result: the optimized keyframes' Tcw, and for those with imu set velocity and bias; every point's position.
#include "types/ba/problem.hpp"

namespace ba_solver {

ba::Result solveLocalInertial(const ba::Problem& p);

}  // namespace ba_solver
