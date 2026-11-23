#pragma once
// The `global` problem: Optimizer::BundleAdjustment as GlobalBundleAdjustemnt(map, 10, ..., bRobust = false)
// calls it for loop closing's visual global BA. Every keyframe pose (the fixed ones held: the map's init
// keyframe), every point, eliminated; one KB8 reprojection per observation with information
// 1/σ² of the feature's level. As the original: the left observations have no robust kernel (bRobust), the right
// ones always a Huber (δ = √5.99, not 5.991); LM from g2o's default λ, p.iterations iterations, no outlier pass.
// Every point of the problem is in the result (the original's nEdges counts observing keyframes, so a point whose
// observations were all dropped is kept, unchanged). Result: every keyframe's Tcw (fixed ones included: the
// write-back marks them), every point's position.
#include <memory>

#include "algorithms/ba_solver/stepped.hpp"
#include "types/ba/problem.hpp"

namespace ba_solver {

std::unique_ptr<SteppedSolve> startGlobal(ba::Problem p);
ba::Result solveGlobal(const ba::Problem& p);

}  // namespace ba_solver
