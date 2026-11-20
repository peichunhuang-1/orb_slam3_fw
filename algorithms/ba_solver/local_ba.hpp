#pragma once
// The `local` problem (Optimizer::LocalBundleAdjustment's optimization, after local mapping chose the window):
// keyframe poses (fixed ones held), points eliminated by the Schur complement, one KB8 reprojection per
// observation (left and right separately: the KB8 rig has no uRight) with information 1/σ² of the feature's
// level and a Huber kernel (δ = √5.991), up to p.iterations LM iterations from p.initial_lambda.
// Outliers: chi2 > 5.991 (the error last computed, g2o's) or the point behind that camera.
#include "types/ba/problem.hpp"

namespace ba_solver {

ba::Result solveLocal(const ba::Problem& p);

}  // namespace ba_solver
