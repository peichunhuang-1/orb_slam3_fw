#pragma once
// Requests from tracking to local mapping (message type "tracking/MapperCommand", port `mapper_cmd`), in place
// of ORB-SLAM3's direct calls: RequestResetActiveMap, RequestReset, InterruptBA.
#include "fw/fw.hpp"
#include "types/map/ids.hpp"

namespace tracking {

struct MapperCommand {
    bool reset_active_map = false;
    map::MapId map_id = map::kNone;  // the map to reset
    bool reset_all = false;
    bool interrupt_ba = false;
};

}  // namespace tracking

FW_REGISTER_TYPE(tracking::MapperCommand, "tracking/MapperCommand")
