#pragma once
// ORB-SLAM3's place recognition (LoopClosing::NewDetectCommonRegions and what it calls) on an atlas snapshot,
// with fixed scale (SE3 for Sim3, algorithms/orb_loop_closing/docs/DESIGN.md):
//
//   detect                     NewDetectCommonRegions: the early outs (inertial map before BA2, small map), the
//                              check of the last loop / merge region from the last keyframe's pose, the BoW
//                              candidates of the regions not confirmed in this keyframe
//   refinePoseFromLastKeyFrame DetectAndReffineSim3FromLastKF
//   detectFromBoW              DetectCommonRegionsFromBoW
//   detectFromLastKeyFrame     DetectCommonRegionsFromLastKF
//   findMatchesByProjection    FindMatchesByProjection
//
// It only reads the atlas. KeyFrame::SetNotErase / SetErase go to `flags` in call order, for local mapping to
// apply (send_to_loop_closing already did the current keyframe's SetNotErase and its KeyFrameDatabase::add).
//
// Keyframes it keeps from one keyframe to the next (a region's matched and last current keyframe) may have gone
// bad meanwhile: their SetNotErase reaches local mapping one round later than in the original. Such a region is
// reset at the start of the next detect (DESIGN.md, "被刪掉的候選").
#include <cstdint>
#include <vector>

#include "algorithms/orb_loop_closing/keyframe_candidates/keyframe_candidates.hpp"
#include "types/geometry/se3.hpp"
#include "types/loop_closing/loop_result.hpp"
#include "types/map/atlas.hpp"

namespace orb_loop_closing {

// One detection state, loop or merge (mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences,
// mnLoopNumNotFound, mvpLoopMPs, mvpLoopMatchedMPs, mbLoopDetected; the merge ones likewise)
struct CommonRegion {
    map::KeyFrameId matched_kf = map::kNone;
    map::KeyFrameId last_current_kf = map::kNone;
    geometry::SE3d Tlw;  // the last current keyframe's pose from the match (mg2oLoopSlw)
    int coincidences = 0, not_found = 0;
    bool detected = false;
    std::vector<map::MapPointId> points;          // the matched keyframe's neighbourhood
    std::vector<map::MapPointId> matched_points;  // per feature of the last current keyframe
};

struct PlaceRecognitionStats {
    long keyframes = 0, skipped = 0;          // detect calls, early outs
    long regions_lost = 0;                    // regions reset as their keyframes went bad
    long candidates_loop = 0, candidates_merge = 0;
};

class PlaceRecognition {
public:
    using Flags = std::vector<loop_closing::EraseFlag>;

    // NewDetectCommonRegions for the current keyframe; true when a loop or a merge is detected (loop().detected,
    // merge().detected).
    bool detect(const map::Atlas& atlas, map::KeyFrameId current, Flags& flags);

    CommonRegion& loop() { return loop_; }
    CommonRegion& merge() { return merge_; }
    // Reset a region after it was handled or given up (Run's "Reset all variables"): SetErase of its last
    // current and matched keyframes
    void resetRegion(CommonRegion& r, Flags& flags);
    // ResetIfRequested: everything, the query scratch too (the atlas's ids start again)
    void clear();

    // DetectCommonRegionsFromBoW alone, for the current keyframe (tests)
    bool detectFromBoW(const map::Atlas& atlas, map::KeyFrameId current,
                       const std::vector<map::KeyFrameId>& candidates, CommonRegion& r, Flags& flags) {
        atlas_ = &atlas;
        current_ = atlas.goodKeyFrame(current);
        return current_ && detectFromBoW(candidates, r, flags);
    }

    bool debug = false;  // config.debug.loop_closing: the reference's [LC] lines
    const PlaceRecognitionStats& stats() const { return stats_; }

private:
    bool refinePoseFromLastKeyFrame(const map::KeyFrame& current, const map::KeyFrame& matched, geometry::SE3d& Tcw,
                                    int& n_proj, std::vector<map::MapPointId>& points,
                                    std::vector<map::MapPointId>& matched_points);
    bool detectFromBoW(const std::vector<map::KeyFrameId>& candidates, CommonRegion& r, Flags& flags);
    bool detectFromLastKeyFrame(const map::KeyFrame& current, const map::KeyFrame& matched, const geometry::SE3d& Tcw,
                                int& n_proj, std::vector<map::MapPointId>& points,
                                std::vector<map::MapPointId>& matched_points);
    int findMatchesByProjection(const map::KeyFrame& current, const map::KeyFrame& matched, const geometry::SE3d& Tcw,
                                std::vector<map::MapPointId>& points, std::vector<map::MapPointId>& matched_points);
    // The region still refers to good keyframes; else it is reset (without flags: they are bad)
    void checkRegion(CommonRegion& r);

    const map::Atlas* atlas_ = nullptr;
    const map::KeyFrame* current_ = nullptr;
    CommonRegion loop_, merge_;
    PlaceRecognitionScratchMap scratch_;
    unsigned seed_ = 0;  // RelativePoseRansac's, one per solver
    PlaceRecognitionStats stats_;
};

}  // namespace orb_loop_closing
