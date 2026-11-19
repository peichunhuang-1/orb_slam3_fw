#pragma once
// All maps, keyframes, map points and stereo rigs, and the keyframes' BoW index (resource type "map/Atlas":
// ORB-SLAM3's Atlas plus its KeyFrameDatabase, under one lock). Data and lookups only; the operations that keep
// it consistent are in atlas_ops.hpp.
//
// Storage: keyframes and points in slot vectors indexed by id (ids are given in order, from 0). A slot stays
// after its keyframe or point goes bad (ORB-SLAM3 keeps the object: GetReplaced, the trajectory's reference
// keyframes); only clearAtlas empties them, and ids then start again at 0 (ORB-SLAM3's Reset sets
// KeyFrame::nNextId = 0; its point ids go on, which no logic depends on).
#include <map>
#include <memory>
#include <vector>

#include "fw/fw.hpp"
#include "types/camera/stereo_rig.hpp"
#include "types/map/bow_index.hpp"
#include "types/map/ids.hpp"
#include "types/map/keyframe.hpp"
#include "types/map/map.hpp"
#include "types/map/map_point.hpp"

namespace map {

struct Atlas {
    std::vector<std::unique_ptr<KeyFrame>> keyframes;  // by id
    std::vector<std::unique_ptr<MapPoint>> points;     // by id
    std::map<MapId, Map> maps;                         // mspMaps
    std::map<MapId, Map> bad_maps;                     // mspBadMaps
    MapId current = kNone;                             // mpCurrentMap
    KeyFrameId last_init_kf_id = 0;                    // mnLastInitKFidMap
    MapId next_map_id = 0;                             // Map::nNextId (never reset)
    std::vector<camera::StereoRig> rigs;               // mvpCameras, as stereo pairs
    BowIndex bow_index;                                // KeyFrameDatabase

    // nullptr if there is no such keyframe / point (never made, or cleared)
    KeyFrame* keyframe(KeyFrameId id) { return id < keyframes.size() ? keyframes[id].get() : nullptr; }
    const KeyFrame* keyframe(KeyFrameId id) const { return id < keyframes.size() ? keyframes[id].get() : nullptr; }
    MapPoint* point(MapPointId id) { return id < points.size() ? points[id].get() : nullptr; }
    const MapPoint* point(MapPointId id) const { return id < points.size() ? points[id].get() : nullptr; }
    // The keyframe / point if it exists and is not bad
    KeyFrame* goodKeyFrame(KeyFrameId id) {
        KeyFrame* k = keyframe(id);
        return k && !k->bad ? k : nullptr;
    }
    const KeyFrame* goodKeyFrame(KeyFrameId id) const {
        const KeyFrame* k = keyframe(id);
        return k && !k->bad ? k : nullptr;
    }
    MapPoint* goodPoint(MapPointId id) {
        MapPoint* p = point(id);
        return p && !p->bad ? p : nullptr;
    }
    const MapPoint* goodPoint(MapPointId id) const {
        const MapPoint* p = point(id);
        return p && !p->bad ? p : nullptr;
    }
    // A live map (not a bad one), nullptr if none
    Map* findMap(MapId id) {
        const auto it = maps.find(id);
        return it == maps.end() ? nullptr : &it->second;
    }
    const Map* findMap(MapId id) const {
        const auto it = maps.find(id);
        return it == maps.end() ? nullptr : &it->second;
    }
    bool hasCurrentMap() const { return findMap(current) != nullptr; }
    // GetCurrentMap: creates the first map if there is none (createNewMap)
    Map& currentMap();
    const Map& currentMap() const;  // throws std::logic_error if there is none
};

}  // namespace map

FW_REGISTER_TYPE(map::Atlas, "map/Atlas")
