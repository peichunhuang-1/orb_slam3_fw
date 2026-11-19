#pragma once
// What tracking reads of local mapping (message type "tracking/MapperStatus", from local mapping's status,
// tracking's `mapper` input, latest). Without local mapping tracking sees none and takes the defaults: idle,
// accepting keyframes, the IMU not initialized.
#include <cstdint>

#include "fw/fw.hpp"
#include "types/imu/preintegrated.hpp"
#include "types/map/ids.hpp"

namespace tracking {

struct MapperStatus {
    bool accept_keyframes = true;   // AcceptKeyFrames
    int keyframes_in_queue = 0;     // KeyframesInQueue
    bool initializing = false;      // IsInitializing (IMU initialization running)
    bool stopped = false;           // isStopped
    bool stop_requested = false;    // stopRequested
    bool bad_imu = false;           // mbBadImu
    bool far_points = false;        // mbFarPoints
    float th_far_points = 0;        // mThFarPoints
    // The last IMU initialization (UpdateFrameIMU): applied once per sequence number
    std::uint64_t imu_update_seq = 0;  // 0: none yet
    float imu_scale = 1;
    imu::Bias imu_bias;
    map::KeyFrameId imu_keyframe = map::kNone;
};

}  // namespace tracking

FW_REGISTER_TYPE(tracking::MapperStatus, "tracking/MapperStatus")
