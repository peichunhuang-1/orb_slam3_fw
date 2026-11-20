#pragma once
// All maps, keyframes, map points and stereo rigs, and the keyframes' BoW index (resource type "map/Atlas":
// ORB-SLAM3's Atlas plus its KeyFrameDatabase, under one lock). Data and lookups only; the operations that keep
// it consistent are in atlas_ops.hpp.
//
// Storage (types/map/docs/DESIGN.md): a COW type of fw::TiledMap, one tile per keyframe, point, map, rig and BoW
// word, so a copy is O(1) and a change copies only the tiles it touches. A keyframe or point stays after it goes
// bad (ORB-SLAM3 keeps the object: GetReplaced, the trajectory's reference keyframes); only clearAtlas empties
// them, and ids then start again (ORB-SLAM3's Reset sets KeyFrame::nNextId = 0; its point ids go on, which no
// logic depends on).
//
// Ids are given in order. Keyframes and maps are made by tracking only; map points by tracking (even ids) and by
// local mapping (odd), so neither has to wait for the other to number a new point. Tracking does not change this
// resource: it sends its changes (tracking::AtlasEdit) and local mapping applies them; tracking_seq is the last
// one applied (types/map/docs/DESIGN.md, "tracking 改成只讀").
//
// Reading: keyframe(), point(), findMap() (const). Changing: editKeyFrame(), editPoint(), editMap(); they copy
// the tile on its first edit in a version and give a pointer to this version's own. That pointer, and any const
// one taken before, stay valid until the atlas is copied, assigned or cleared, or the handler that got the atlas
// returns; a const pointer taken before an edit of the same tile shows the tile as it was.
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "fw/fw.hpp"
#include "types/camera/stereo_rig.hpp"
#include "types/map/bow_index.hpp"
#include "types/map/ids.hpp"
#include "types/map/keyframe.hpp"
#include "types/map/map.hpp"
#include "types/map/map_point.hpp"

namespace map {

struct Atlas {
    fw::TiledMap<KeyFrameId, KeyFrame> keyframes;  // by id
    fw::TiledMap<MapPointId, MapPoint> points;     // by id
    fw::TiledMap<MapId, Map> maps;                 // mspMaps
    fw::TiledMap<MapId, Map> bad_maps;             // mspBadMaps
    fw::TiledMap<int, camera::StereoRig> rigs;     // mvpCameras, as stereo pairs, by index 0..n_rigs-1
    BowIndex bow_index;                            // KeyFrameDatabase
    MapId current = kNone;                         // mpCurrentMap
    KeyFrameId last_init_kf_id = 0;                // mnLastInitKFidMap
    MapId next_map_id = 0;                         // Map::nNextId (never reset)
    KeyFrameId next_keyframe_id = 0;               // KeyFrame::nNextId
    MapPointId next_point_id = 1;                  // MapPoint::nNextId of local mapping's points (odd)
    MapPointId next_tracking_point_id = 0;         // of tracking's (even)
    int n_rigs = 0;
    std::uint64_t tracking_seq = 0;                // the last tracking::AtlasEdit applied (0: none)

    // nullptr if there is no such keyframe / point (never made, or cleared)
    const KeyFrame* keyframe(KeyFrameId id) const { return keyframes.find(id); }
    const MapPoint* point(MapPointId id) const { return points.find(id); }
    KeyFrame* editKeyFrame(KeyFrameId id) { return keyframes.find(id) ? keyframes.edit(id).operator->() : nullptr; }
    MapPoint* editPoint(MapPointId id) { return points.find(id) ? points.edit(id).operator->() : nullptr; }
    // The keyframe / point if it exists and is not bad
    const KeyFrame* goodKeyFrame(KeyFrameId id) const {
        const KeyFrame* k = keyframe(id);
        return k && !k->bad ? k : nullptr;
    }
    const MapPoint* goodPoint(MapPointId id) const {
        const MapPoint* p = point(id);
        return p && !p->bad ? p : nullptr;
    }
    // A live map (not a bad one), nullptr if none
    const Map* findMap(MapId id) const { return maps.find(id); }
    Map* editMap(MapId id) { return maps.find(id) ? maps.edit(id).operator->() : nullptr; }
    bool hasCurrentMap() const { return findMap(current) != nullptr; }
    // GetCurrentMap: creates the first map if there is none (createNewMap)
    const Map& currentMap();
    const Map& currentMap() const;  // throws std::logic_error if there is none
    Map& editCurrentMap();          // currentMap, to change
    // A stereo rig by index (throws std::out_of_range if there is none)
    const camera::StereoRig& rig(int i) const {
        const camera::StereoRig* r = rigs.find(i);
        if (!r) throw std::out_of_range("map::Atlas: no such rig");
        return *r;
    }
};

}  // namespace map

FW_COW_TYPE(map::Atlas, keyframes, points, maps, bad_maps, rigs, bow_index, current, last_init_kf_id, next_map_id,
            next_keyframe_id, next_point_id, next_tracking_point_id, n_rigs, tracking_seq);
FW_REGISTER_TYPE(map::Atlas, "map/Atlas")
