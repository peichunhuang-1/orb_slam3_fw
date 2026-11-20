#pragma once
// Requests from tracking to local mapping (message type "tracking/MapperCommand", port `mapper_cmd`), in place
// of ORB-SLAM3's direct calls: InterruptBA. The resets (RequestReset, RequestResetActiveMap) are atlas edits
// (atlas_edits.hpp), so that they stay in order with the keyframes.
#include "fw/fw.hpp"

namespace tracking {

struct MapperCommand {
    bool interrupt_ba = false;
};

}  // namespace tracking

FW_REGISTER_TYPE(tracking::MapperCommand, "tracking/MapperCommand")
