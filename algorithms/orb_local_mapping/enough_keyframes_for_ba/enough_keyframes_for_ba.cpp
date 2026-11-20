// enough_keyframes_for_ba: mpAtlas->KeyFramesInMap() > 2 (the current map)
#include "fw/fw.hpp"
#include "types/map/atlas.hpp"

namespace {

fw::bt::Status enough_keyframes_for_ba(fw::Read<map::Atlas> atlas) {
    const map::Map* m = atlas->findMap(atlas->current);
    return m && m->keyframes.size() > 2 ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(enough_keyframes_for_ba, RESOURCES(("atlas", map::Atlas, read)));
