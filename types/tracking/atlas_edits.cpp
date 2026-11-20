#include "types/tracking/atlas_edits.hpp"

#include "types/map/atlas_ops.hpp"

namespace tracking {

namespace {

void newMap(map::Atlas& a, map::MapId id, bool inertial) {
    map::createNewMap(a, id);
    if (inertial) a.editMap(id)->inertial = true;
}

void addKeyFrame(map::Atlas& a, const AtlasEdit& e) {
    const map::KeyFrameId id = e.keyframe.id;
    map::putKeyFrame(a, e.keyframe);
    if (map::KeyFrame* last = a.editKeyFrame(e.keyframe.prev)) last->next = id;
    if (e.insert) map::insertKeyFrame(a, id);
    for (const NewStereoPoint& p : e.points) map::createStereoPoint(a, id, p.index, p.pos, p.id);
    if (e.origin)
        if (map::Map* m = a.editMap(e.keyframe.map)) m->keyframe_origins.push_back(id);
}

}  // namespace

void applyEdit(map::Atlas& a, const AtlasEdit& e) {
    using K = AtlasEdit::Kind;
    switch (e.kind) {
        case K::new_map:
            newMap(a, e.map, e.inertial);
            break;
        case K::map_inertial:
            if (map::Map* m = a.editMap(e.map)) m->inertial = true;
            break;
        case K::add_rig:
            a.rigs.edit(e.rig_index).set(e.rig);
            if (e.rig_index >= a.n_rigs) a.n_rigs = e.rig_index + 1;
            break;
        case K::reset_all:
            map::clearAtlas(a);  // and the BoW index, ids from 0
            newMap(a, e.map, e.inertial);
            break;
        case K::reset_active_map:
            map::eraseMapFromBowIndex(a, e.map);
            map::clearMap(a, e.map);
            break;
        case K::keyframe:
            addKeyFrame(a, e);
            break;
        case K::counts:
            for (const auto& [id, c] : e.counts)
                if (map::MapPoint* p = a.editPoint(id)) {
                    p->visible += c.visible;
                    p->found += c.found;
                }
            break;
    }
    a.tracking_seq = e.seq;
}

}  // namespace tracking
