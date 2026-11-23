#pragma once
// ORB-SLAM3's KeyFrameDatabase::DetectNBestCandidates on the atlas's BoW index: the keyframes sharing a word with
// the query keyframe, except its covisibles and itself; those with more than 80 % of the most shared words scored
// (L1); each score accumulated over its 10 best covisibles scored for this query, the best keyframe of each group
// kept; by accumulated score, the first n in the query's map (loop candidates) and the first n in another live map
// (merge candidates).
//
// The query keyframe is already in the index (send_to_loop_closing adds it before handing it over; the original
// adds it after the query), so it is left out here: the candidates are the original's.
//
// The query scratch (mnPlaceRecognitionQuery, mnPlaceRecognitionWords, mPlaceRecognitionScore) is loop closing's
// and outlives the query as in the original; it is keyed by keyframe id and must be cleared when the atlas is
// reset (ids start again from 0).
#include <unordered_map>
#include <vector>

#include "types/map/atlas.hpp"

namespace orb_loop_closing {

struct PlaceRecognitionScratch {
    map::KeyFrameId query = map::kNone;  // mnPlaceRecognitionQuery
    int words = 0;                       // mnPlaceRecognitionWords
    float score = 0;                     // mPlaceRecognitionScore
};
using PlaceRecognitionScratchMap = std::unordered_map<map::KeyFrameId, PlaceRecognitionScratch>;

void detectNBestCandidates(const map::Atlas& atlas, PlaceRecognitionScratchMap& scratch, map::KeyFrameId kf, int n,
                           std::vector<map::KeyFrameId>& loop_candidates,
                           std::vector<map::KeyFrameId>& merge_candidates);

}  // namespace orb_loop_closing
