#pragma once
// ORB-SLAM3's KeyFrameDatabase::DetectRelocalizationCandidates on the atlas's BoW index: keyframes sharing a word
// with the frame, those with more than 80 % of the most shared words scored (L1), each score accumulated over its
// 10 best covisibles queried by this frame, the best of each group above 75 % of the best accumulated score, in
// the current map only. The query scratch (mnRelocQuery, mnRelocWords, mRelocScore) is the tracker's
// (Tracker::reloc) and outlives the query as there: a covisible queried by this frame but not scored carries the
// score of an earlier query.
#include <vector>

#include "types/map/atlas.hpp"
#include "types/map/vocabulary.hpp"
#include "types/tracking/frame.hpp"
#include "types/tracking/tracker.hpp"

namespace orb_tracking {

// frame.bow must be computed
std::vector<map::KeyFrameId> relocalizationCandidates(const map::Atlas& atlas, tracking::Tracker& tracker,
                                                      const tracking::Frame& frame);

}  // namespace orb_tracking
