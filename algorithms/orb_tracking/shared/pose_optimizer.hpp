#pragma once
// ORB-SLAM3's Optimizer::PoseOptimization: the frame's pose from its map points, left and right observations
// as separate monocular edges (KB8 stereo), Huber kernel, 4 rounds of 10 Levenberg-Marquardt iterations with
// outliers re-classified after each round. Used by track_reference_keyframe, track_with_motion_model,
// relocalize and track_local_map.
//
// The solver is g2o's, written out (g2o is not used): OptimizationAlgorithmLevenberg with its stop rules,
// VertexSE3Expmap's left update exp(δ)·T, a dense LDLT, and the original's details:
//   - every round starts again from the frame's pose as it was on entry (the original resets the vertex from
//     the frame, which is only written at the end);
//   - an edge's chi2 for the outlier test is the error last computed, which g2o leaves at the last trial step
//     even when that step was rejected;
//   - the Huber kernel goes after round 3; the chi2 tests are in float (5.991);
//   - fewer than 10 edges: one round only; fewer than 3 correspondences: returns 0, the pose unchanged.
// Not as the original: the projection and its Jacobian in double from one θ (KannalaBrandt8::project_jacobian_d;
// rounding differences only), and, when `par` is given, the edges of each iteration on it (shared/parallel.hpp;
// fixed chunks, the same result for any thread count). The tracking leaves pass none: with a few hundred edges
// the threads cost more than they save.
#include <vector>

#include <Eigen/Core>

#include "algorithms/orb_tracking/shared/parallel.hpp"
#include "types/camera/stereo_rig.hpp"
#include "types/map/atlas.hpp"
#include "types/tracking/frame.hpp"

namespace orb_tracking {

// world[i]: the world position of feature i's map point, nullptr if it has none (size frame.size()). Sets
// frame.outlier for the features with a point and the pose; returns the inlier count.
int optimizePose(tracking::Frame& frame, const camera::StereoRig& rig, const std::vector<const Eigen::Vector3f*>& world,
                 const ParallelFor& par = {});

// The frame's map points from the atlas (bad ones too, as the original uses whatever the frame points to)
int optimizePose(tracking::Frame& frame, const map::Atlas& atlas, const ParallelFor& par = {});

}  // namespace orb_tracking
