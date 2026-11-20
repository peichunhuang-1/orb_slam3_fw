#pragma once
// LocalMapping::CreateNewMapPoints and what only it uses: ORBmatcher::SearchForTriangulation (with KB8's
// epipolarConstrain, i.e. KannalaBrandt8::TriangulateMatches) and the linear two-view triangulation.
//
// Reproduced as written, quirks included (algorithms/orb_local_mapping/docs/DESIGN.md, "create_new_map_points"):
// - the baseline check of each neighbour uses the camera centre the previous neighbour's last match left in Ow1
//   (left or right camera), not always the current keyframe's left centre;
// - the inertial neighbours are padded with the previous keyframes while there are <= 10 (so up to 11);
// - SearchForTriangulation skips any feature with a map point set, and the new points are set in both keyframes
//   at once, so a feature triangulated with one neighbour is not matched again with the next;
// - no orientation check (the matcher is made with checkOri = false).
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

#include "types/camera/kannala_brandt.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace orb_local_mapping {

// GeometricTools::Triangulate / KannalaBrandt8::Triangulate: the DLT of two z = 1 rays under the 3x4 poses.
// false when the homogeneous w is 0 (x3D then left unchanged)
bool triangulateDlt(const Eigen::Vector3f& x_c1, const Eigen::Vector3f& x_c2, const Eigen::Matrix<float, 3, 4>& Tc1w,
                    const Eigen::Matrix<float, 3, 4>& Tc2w, Eigen::Vector3f& x3D);

// KannalaBrandt8::TriangulateMatches: kp1 in cam1, kp2 in cam2, R12 / t12 camera 2 in camera 1, sigma1 / sigma2
// the levels' σ². The depth in camera 1 (> 0) or a negative code: -1 low parallax, -2 / -3 behind camera 1 / 2,
// -4 / -5 reprojection error in camera 1 / 2. epipolarConstrain is "> 0.0001".
float triangulateMatches(const camera::KannalaBrandt8& cam1, const camera::KannalaBrandt8& cam2,
                         const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const Eigen::Matrix3f& R12,
                         const Eigen::Vector3f& t12, float sigma1, float sigma2, Eigen::Vector3f& p3D);

// ORBmatcher::SearchForTriangulation(kf1, kf2, pairs, coarse) with checkOri = false: features of kf1 and kf2
// with no map point, in the same vocabulary node (level 4), best Hamming distance <= TH_LOW (50) among those that
// pass the epipolar check (all of them when coarse), each kf2 feature matched once. Pairs by kf1 index.
int searchForTriangulation(const map::Atlas& a, const map::KeyFrame& kf1, const map::KeyFrame& kf2, bool coarse,
                           std::vector<std::pair<int, int>>& pairs);

// The neighbours CreateNewMapPoints goes over, in order: the 10 best covisibles, and when inertial the previous
// keyframes (not already in) while there are <= 10, up to 10 steps back
std::vector<map::KeyFrameId> triangulationNeighbours(const map::Atlas& a, map::KeyFrameId kf, bool inertial);

// CreateNewMapPoints for local_mapper.current. recently_lost: tracking's state is RECENTLY_LOST (bCoarse when
// inertial and the map's inertial BA 2 is done). Stops before a neighbour after the first when the queue is not
// empty (CheckNewKeyFrames). The new points (local mapping's ids) are in the map and appended to
// local_mapper.recent_points; returns how many.
int createNewMapPoints(map::Atlas& a, local_mapping::LocalMapper& m, bool recently_lost);

}  // namespace orb_local_mapping
