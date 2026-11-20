#pragma once
// The inertial problems built at the IMU initialization (initialize_imu):
//
//   inertial_init  Optimizer::InertialOptimization(map, Rwg, scale, bg, ba, ..., bFixedVel = false, priorG, priorA):
//                  every keyframe's body pose fixed and velocity free; one gyro and one accelerometer bias
//                  (starting at p.bg, p.ba), Rwg (2-dof, from p.Rwg), the scale (fixed: stereo); an EdgeInertialGS
//                  per keyframe with a previous one in the problem; bias priors (information prior_a, prior_g, both
//                  added even when 0). p.iterations LM iterations from p.initial_lambda (1e3 when prior_g != 0).
//                  Result: Rwg, scale, bg, ba, and each keyframe's velocity and (the shared) bias.
//   full_inertial  Optimizer::FullInertialBA(map, its, bFixLocal = false, ...): every keyframe's body pose; for
//                  those with imu set (bImu) a velocity, and either a bias each with random-walk edges or, with
//                  p.shared_bias (bInit), one shared bias (starting at the last keyframe's) with the priors; an
//                  EdgeInertial (Huber √16.92) between consecutive keyframes both with imu; points eliminated with
//                  one EdgeMono per observation (Huber √5.991). A point with no observation is left out.
//                  Result: every keyframe's Tcw, and for those with imu set velocity and bias; the points'
//                  positions.
#include "types/ba/problem.hpp"

namespace ba_solver {

ba::Result solveInertialInit(const ba::Problem& p);
ba::Result solveFullInertial(const ba::Problem& p);

}  // namespace ba_solver
