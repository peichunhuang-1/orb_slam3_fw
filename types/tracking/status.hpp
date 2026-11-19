#pragma once
// One per frame from tracking (message type "tracking/Status", port `tracking`): the state and what the
// frame did, and the stage times.
#include <cstdint>

#include "fw/fw.hpp"
#include "types/map/ids.hpp"
#include "types/tracking/state.hpp"

namespace tracking {

struct Status {
    State state = State::NO_IMAGES_YET;
    long frame_id = -1;
    int inliers = 0;                   // mnMatchesInliers after the local map
    map::MapId map_id = map::kNone;    // the current map
    int maps = 0;
    std::uint64_t keyframes = 0;       // in the current map
    std::uint64_t points = 0;
    bool new_keyframe = false;
    bool relocalized = false;
    // Stage times (ms): IMU (correction and concatenation), pose estimate, local map, keyframe, the whole tick
    double t_imu_ms = 0, t_pose_ms = 0, t_local_map_ms = 0, t_keyframe_ms = 0, t_total_ms = 0;
};

}  // namespace tracking

FW_REGISTER_TYPE(tracking::Status, "tracking/Status")
