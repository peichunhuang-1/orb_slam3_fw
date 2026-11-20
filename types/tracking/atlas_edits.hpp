#pragma once
// What tracking changes in the atlas, sent to local mapping to apply (message type "tracking/AtlasEdits", port
// `atlas_edits`; types/map/docs/DESIGN.md, "tracking 改成只讀"). Tracking does not write the atlas resource: it
// applies each edit to its own copy of the atlas (Tracker::view) and sends it; local mapping applies it to the
// resource and sets Atlas::tracking_seq. Until a snapshot has it, tracking applies it again on each new copy.
//
// Ids are given by tracking, from its copy: keyframes and maps (only tracking makes them), rigs, and its points
// (even ids). So applying an edit gives the same ids on any copy, whatever local mapping did in between.
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "fw/fw.hpp"
#include "types/camera/stereo_rig.hpp"
#include "types/map/atlas.hpp"
#include "types/map/ids.hpp"
#include "types/map/keyframe.hpp"

namespace tracking {

// A stereo point made with a keyframe (createStereoPoint): its id, left feature and world position
struct NewStereoPoint {
    map::MapPointId id = map::kNone;
    int index = -1;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
};

// IncreaseVisible / IncreaseFound of a point, summed over the frames since the last keyframe
struct PointCounts {
    int visible = 0, found = 0;
};

struct AtlasEdit {
    enum class Kind {
        new_map,           // CreateNewMap (CreateMapInAtlas, the first frame): map; inertial: SetInertialSensor
        map_inertial,      // SetInertialSensor of map
        add_rig,           // AddCamera: rig at rig_index
        reset_all,         // Tracking::Reset: clearAtlas, then new_map
        reset_active_map,  // Tracking::ResetActiveMap of map: KeyFrameDatabase::clearMap, Map::clear
        keyframe,          // StereoInitialization / CreateNewKeyFrame, then InsertKeyFrame
        counts,            // IncreaseVisible / IncreaseFound
    };
    std::uint64_t seq = 0;  // from 1, in the order tracking made them
    Kind kind = Kind::new_map;

    map::MapId map = map::kNone;
    bool inertial = false;

    int rig_index = -1;
    camera::StereoRig rig;

    // keyframe: as prepareKeyFrame made it (id, map, bImu), prev set; insert: in the map now (the first keyframe,
    // StereoInitialization; later ones join in ProcessNewKeyFrame); origin: mvpKeyFrameOrigins
    map::KeyFrame keyframe;
    bool insert = false, origin = false;
    std::vector<NewStereoPoint> points;

    std::vector<std::pair<map::MapPointId, PointCounts>> counts;
};

// The edits tracking has made that the last atlas it read did not have yet, in order. Each message has all of
// them, so one that is dropped loses nothing; local mapping skips the ones it already applied.
struct AtlasEdits {
    std::vector<std::shared_ptr<const AtlasEdit>> edits;
};

// Applies e to a (the resource or tracking's copy) and sets a.tracking_seq. The keyframe's links: the previous
// keyframe's next, if it is still there.
void applyEdit(map::Atlas& a, const AtlasEdit& e);

}  // namespace tracking

FW_REGISTER_TYPE(tracking::AtlasEdits, "tracking/AtlasEdits")
