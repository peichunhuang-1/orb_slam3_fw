#pragma once
// One map of the atlas (ORB-SLAM3's Map): which keyframes are in it, how many points, and its state. Sets are by
// id (ORB-SLAM3's by pointer), so GetAllKeyFrames comes out by id. It is an atlas tile
// (types/map/docs/DESIGN.md): its points are not listed here, as that list would be copied on every point added;
// GetAllMapPoints is mapPoints (atlas_ops.hpp), from the points' own MapPoint::map and in_map.
#include <cstddef>
#include <vector>

#include "types/map/flat.hpp"
#include "types/map/ids.hpp"

namespace map {

struct Map {
    MapId id = kNone;
    FlatSet<KeyFrameId> keyframes;
    std::size_t n_points = 0;  // MapPoint::in_map ones (MapPointsInMap)
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
