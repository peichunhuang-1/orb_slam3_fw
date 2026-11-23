#pragma once
// What loop closing found for one keyframe (message type "loop_closing/LoopResult"): the changes to the atlas
// its place recognition makes in ORB-SLAM3, for local mapping to apply, the only writer of the atlas
// (algorithms/orb_loop_closing/docs/DESIGN.md, "誰寫 atlas"), and the loop or merge it detected. Keyframes that
// went bad meanwhile are skipped.
//
// Poses are SE3 (stereo / stereo-inertial: the original's Sim3 with scale fixed to 1).
#include <cstdint>
#include <vector>

#include "fw/fw.hpp"
#include "types/geometry/se3.hpp"
#include "types/map/ids.hpp"

namespace loop_closing {

// KeyFrame::SetNotErase (not_erase) or SetErase (!not_erase)
struct EraseFlag {
    map::KeyFrameId keyframe = map::kNone;
    bool not_erase = false;
};

struct LoopResult {
    enum class Kind { none, loop, merge };

    std::uint64_t seq = 0;
    std::uint64_t epoch = 0;                // the LoopKeyFrame's
    map::KeyFrameId keyframe = map::kNone;  // mpCurrentKF
    // SetNotErase / SetErase, in the order the original calls them (the same keyframe can get both)
    std::vector<EraseFlag> erase_flags;

    // What was detected and is to be corrected (none: nothing, or a detection the original does not correct:
    // a bad loop, a merge before the IMU is initialized)
    Kind kind = Kind::none;
    map::KeyFrameId matched_kf = map::kNone;  // mpLoopMatchedKF / mpMergeMatchedKF
    geometry::SE3d Tcw;                       // mg2oLoopScw / mg2oMergeScw: the current keyframe corrected
    geometry::SE3d Tmw;                       // merge: mg2oMergeSmw
    geometry::SE3d T_old_new;                 // merge: mSold_new (yaw only when inertial after BA1)
    map::KeyFrameId last_current_kf = map::kNone;  // merge: mpMergeLastCurrentKF
    std::vector<map::MapPointId> matched_points;   // per current keyframe feature (mvpLoopMatchedMPs / mvpMergeMatchedMPs)
    std::vector<map::MapPointId> region_points;    // the matched neighbourhood (mvpLoopMapPoints / mvpMergeMPs)
};

}  // namespace loop_closing

FW_REGISTER_TYPE(loop_closing::LoopResult, "loop_closing/LoopResult")
