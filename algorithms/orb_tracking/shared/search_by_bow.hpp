#pragma once
// ORB-SLAM3's ORBmatcher::SearchByBoW(KeyFrame*, Frame&, ...): the keyframe's map points matched to the frame's
// features within the same vocabulary node (the FeatureVector, 4 levels up). Used by track_reference_keyframe
// (nn_ratio 0.7) and relocalize (0.75), both with the orientation check.
//
// Per keyframe feature with a good map point, over the frame features of its node not yet matched: the best and
// second best Hamming distance among the left features and among the right ones. As the original:
//   - left: best <= TH_LOW (50) and best < nn_ratio · second;
//   - right: only tried when the left best is <= TH_LOW (whether or not the left passed its ratio), and without
//     the ratio test (the original's condition ends in "|| true").
// Then the rotation consistency check (rotation_histogram.hpp).
#include <vector>

#include "types/map/atlas.hpp"
#include "types/tracking/frame.hpp"

namespace orb_tracking {

// matches: per frame feature, the matched map point (map::kNone: none). frame.feat must be computed. Returns the
// number of matches.
int searchByBoW(const map::KeyFrame& kf, const map::Atlas& atlas, const tracking::Frame& frame, float nn_ratio,
                bool check_orientation, std::vector<map::MapPointId>& matches);

}  // namespace orb_tracking
