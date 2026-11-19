#pragma once
// One map of the atlas (ORB-SLAM3's Map): which keyframes and points are in it, and its state. Sets are by id
// (ORB-SLAM3's by pointer), so GetAllKeyFrames / GetAllMapPoints come out by id.
#include <set>
#include <vector>

#include "types/map/ids.hpp"

namespace map {

struct Map {
    MapId id = kNone;
    std::set<KeyFrameId> keyframes;
    std::set<MapPointId> points;
    std::vector<KeyFrameId> keyframe_origins;  // mvpKeyFrameOrigins
    std::vector<MapPointId> reference_points;   // mvpReferenceMapPoints (drawn only)

    KeyFrameId init_kf_id = 0;      // mnInitKFid
    KeyFrameId max_kf_id = 0;       // mnMaxKFid
    KeyFrameId initial_kf = kNone;  // mpKFinitial
    KeyFrameId lower_kf = kNone;    // mpKFlowerID

    bool inertial = false;         // mbIsInertial
    bool imu_initialized = false;  // mbImuInitialized
    bool imu_ba1 = false, imu_ba2 = false;
    bool in_use = false, bad = false;
    int change_index = 0;           // mnMapChange
    int change_notified = 0;        // mnMapChangeNotified
    int big_change_index = 0;       // mnBigChangeIdx
};

}  // namespace map
