#pragma once
// A keyframe, by id, as other blocks are told about it (message type "map/KeyFrameRef"): tracking's
// `keyframe` output to local mapping, in place of ORB-SLAM3's InsertKeyFrame.
#include "fw/fw.hpp"
#include "types/map/ids.hpp"

namespace map {

struct KeyFrameRef {
    MapId map_id = kNone;
    KeyFrameId keyframe_id = kNone;
};

}  // namespace map

FW_REGISTER_TYPE(map::KeyFrameRef, "map/KeyFrameRef")
