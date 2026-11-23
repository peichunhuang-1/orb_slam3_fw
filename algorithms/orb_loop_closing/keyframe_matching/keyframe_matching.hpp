#pragma once
// The ORBmatcher searches of ORB-SLAM3's place recognition, between keyframes and from a pose:
//
//   searchByBoW              SearchByBoW(KeyFrame*, KeyFrame*, vpMatches12): kf1's left features with a good point
//                            against kf2's left features with a good point in the same vocabulary node, best
//                            < TH_LOW (50) and the ratio test, each kf2 feature used once, then the rotation
//                            check. Right-view features are skipped (with two cameras the original's mvKeysUn
//                            holds the left view only).
//   searchByProjection       SearchByProjection(pKF, Scw, vpPoints, vpMatched, th, ratioHamming), fixed scale:
//                            the points not yet in `matched` projected into kf's left view with the KB8 model,
//                            searched within th · scale factor of the predicted level, levels [pred - 1, pred],
//                            the best distance <= TH_LOW · ratio.
//   searchByProjection with  the overload with vpPointsKFs / vpMatchedKF: the same, except that it projects with
//   point keyframes          the pinhole formula fx·x/z + cx on the KB8 intrinsics, as the original does (no
//                            distortion). Reproduced as written: the matches it finds depend on it.
//
// Both projections take an empty slot only (`matched[idx]` kNone) and skip points already in `matched`.
#include <vector>

#include "types/geometry/se3.hpp"
#include "types/map/atlas.hpp"

namespace orb_loop_closing {

// matches12: per kf1 feature (size kf1.size()), kf2's matched point (kNone: none). Returns the number of matches.
int searchByBoW(const map::Atlas& atlas, const map::KeyFrame& kf1, const map::KeyFrame& kf2, float nn_ratio,
                bool check_orientation, std::vector<map::MapPointId>& matches12);

// matched: per kf feature (size kf.size()), in and out. Returns the number of new matches.
int searchByProjection(const map::Atlas& atlas, const map::KeyFrame& kf, const geometry::SE3f& Tcw,
                       const std::vector<map::MapPointId>& points, std::vector<map::MapPointId>& matched, int th,
                       float ratio_hamming);

// point_kfs: per point, the keyframe it came from; matched_kfs (size kf.size()) gets it for each new match
int searchByProjection(const map::Atlas& atlas, const map::KeyFrame& kf, const geometry::SE3f& Tcw,
                       const std::vector<map::MapPointId>& points, const std::vector<map::KeyFrameId>& point_kfs,
                       std::vector<map::MapPointId>& matched, std::vector<map::KeyFrameId>& matched_kfs, int th,
                       float ratio_hamming);

}  // namespace orb_loop_closing
