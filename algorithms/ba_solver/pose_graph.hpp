#pragma once
// The pose graph problems of loop closing (the original's essential graph optimizations, after loop closing
// chose the vertices and computed the edges' measurements: algorithms/orb_loop_closing/correct_loop/).
//
// pose_graph       Optimizer::OptimizeEssentialGraph with bFixScale = true: one vertex per keyframe, its Tcw
//                  (VertexSim3Expmap with the scale update zeroed: the scale stays 1, and g2o's numeric Jacobian
//                  of the scale is zero, as the perturbation goes through the same zeroed update, so the problem
//                  is 6-dof), δ applied as exp(δ)·Tcw (g2o's Sim3(update) with σ = 0); EdgeSim3's error
//                  log(Sji · Siw · Sjw⁻¹) (Sim3::log with σ = 0), information I; LM from λ = 1e-16, 20 iterations.
// pose_graph_4dof  Optimizer::OptimizeEssentialGraph4DoF: VertexPose4DoF (ImuCamPose::UpdateW: the body's yaw
//                  about the world z and its world translation, the rotation's accumulated change DR re-made a
//                  pure yaw and orthonormalized every 5 updates, as the original), Edge4DoF's error
//                  [LogSO3(Ri·Rjᵀ·dRijᵀ); Ri·(-Rjᵀ·tj) + ti - dtij] with information diag(1e3, 1e3, 1, 1, 1, 1)
//                  (the original sets (0, 0) twice); LM from g2o's default λ, 20 iterations.
// Both: g2o's numeric Jacobians (central differences, δ = 1e-9), as the original's edges define none.
// The result's poses: each problem keyframe's optimized Tcw, fixed ones included.
#include "types/ba/problem.hpp"

namespace ba_solver {

ba::Result solvePoseGraph(const ba::Problem& p);
ba::Result solvePoseGraph4DoF(const ba::Problem& p);

}  // namespace ba_solver
