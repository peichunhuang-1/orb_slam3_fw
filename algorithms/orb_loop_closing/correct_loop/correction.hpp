#pragma once
// LoopClosing::CorrectLoop, run by local mapping (the atlas's only writer) once it has stopped
// (algorithms/orb_loop_closing/docs/DESIGN.md, "local mapping 的樹"), in two halves around the essential graph,
// which ba_solver computes:
//
// beginCorrectLoop  (the original up to OptimizeEssentialGraph)
//   UpdateConnections of the current keyframe; it and its covisibles get the loop's pose (each its pose relative
//   to the current keyframe, times mg2oLoopScw); the points they see are moved with the first of them (by id:
//   the original's map is keyed by pointer) that sees them, from its old pose to its new one; velocities
//   rotated as the pose (IMU initialized); their connections updated; the map's change index. Then the loop
//   fusion: each matched point replaces the current keyframe's point at that feature or is added there;
//   SearchAndFuse of the loop's region points into each corrected keyframe (fuseWithPose, th 4), the found
//   point replaced by the region's; the new links (LoopConnections): each corrected keyframe's connections
//   after updating them, minus those it had and minus the corrected keyframes. The pose graph problem:
//     4dof  inertial map with the IMU initialized (OptimizeEssentialGraph4DoF), the matched keyframe fixed
//     6dof  otherwise (OptimizeEssentialGraph), the map's first keyframe fixed
//   with the original's edges (ba/problem.hpp, PoseGraphEdge): the new links (weight >= 100, or the current
//   keyframe's link to the matched one), then per keyframe the spanning tree (6dof only; the 4dof's parent is
//   null in the original), the previous keyframe (6dof: when bImu), the loop edges to older keyframes, and the
//   covisibles with weight >= 100 that are older and not yet linked (6dof: not the parent or a child; 4dof: not
//   the previous or next keyframe, a child or a loop edge). nullopt: the current or matched keyframe went bad.
//
// endCorrectLoop  (the original from the optimization's write-back)
//   Each vertex's pose set; every point of the map moved from its reference keyframe's pose before the
//   optimization to its optimized one (6dof: the keyframe that corrected it, when the loop moved it); the
//   change index; InformNewBigChange; the loop edges (both ways).
#include <optional>
#include <vector>

#include "types/ba/problem.hpp"
#include "types/geometry/se3.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace orb_loop_closing {

// ORBmatcher::Fuse(pKF, Scw, vpPoints, th, vpReplacePoint) with Scw's scale 1: each good point not yet in kf,
// in front of the pose Tcw, projected (KB8) into its left image, in the scale range and under 60° off its normal,
// matched to the best left feature (Hamming <= 50) within th · scale of the predicted level (that level or one
// below). A feature holding a good point: that point goes in replace (per points index), the caller replaces it;
// holding none: the point is added there. Returns the matches.
int fuseWithPose(map::Atlas& a, map::KeyFrameId kf, const geometry::SE3f& Tcw,
                 const std::vector<map::MapPointId>& points, float th, std::vector<map::MapPointId>& replace);

std::optional<ba::Problem> beginCorrectLoop(map::Atlas& a, local_mapping::PendingLoop& loop);
void endCorrectLoop(map::Atlas& a, const local_mapping::PendingLoop& loop, const ba::Result& result);

}  // namespace orb_loop_closing
