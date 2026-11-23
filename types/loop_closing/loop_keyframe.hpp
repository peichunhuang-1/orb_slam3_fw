#pragma once
// A keyframe local mapping hands to loop closing (message type "loop_closing/LoopKeyFrame"), in place of
// ORB-SLAM3's LoopClosing::InsertKeyFrame (algorithms/orb_loop_closing/docs/DESIGN.md, "訊息").
//
// epoch: local mapping's reset_epoch when it sent it. The atlas restarts the keyframe ids on a reset, so an id
// loop closing kept from an older epoch may name another keyframe: loop closing drops what it kept when the epoch
// changes (its ResetIfRequested), local mapping drops results of an older epoch.
#include <cstdint>

#include "fw/fw.hpp"
#include "types/map/ids.hpp"

namespace loop_closing {

struct LoopKeyFrame {
    map::MapId map = map::kNone;
    map::KeyFrameId keyframe = map::kNone;
    std::uint64_t epoch = 0;
};

}  // namespace loop_closing

FW_REGISTER_TYPE(loop_closing::LoopKeyFrame, "loop_closing/LoopKeyFrame")
