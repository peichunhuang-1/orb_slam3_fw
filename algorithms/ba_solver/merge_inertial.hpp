#pragma once
// The `merge_inertial` problem (Optimizer::MergeInertialBA's optimization, after loop closing chose the window,
// algorithms/orb_loop_closing/merge_maps/merge.hpp):
//   - per keyframe a body pose (ImuPoseManifold), and for one with imu set (bImu) a velocity and gyro / acc
//     biases; all of a fixed keyframe's held fixed;
//   - an EdgeInertial from keyframes[prev] to each keyframe with prev >= 0 when both have imu set and it has a
//     preintegration, Huber δ = √16.92, with gyro and acc random-walk edges (information C(9:12)⁻¹, C(12:15)⁻¹);
//   - points eliminated, one EdgeMono per observation (Huber √5.991, information as the problem gives it);
//   - p.iterations LM iterations from p.initial_lambda.
// A block no edge uses is held as it is (g2o leaves a vertex without edges out of the optimization): the
// velocity and biases of a keyframe without an inertial edge, a pose no edge sees.
// Outliers: chi2 > 5.991. Not ok (nothing but the outliers written back, the local modification of the reference
// for a diverged BA) when any estimate is not finite.
// Result: the optimized keyframes' Tcw, and for those with imu set velocity and bias; every point's position.
#include "types/ba/problem.hpp"

namespace ba_solver {

ba::Result solveMergeInertial(const ba::Problem& p);

}  // namespace ba_solver
