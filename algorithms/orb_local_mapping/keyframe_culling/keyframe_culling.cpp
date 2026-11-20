// keyframe_culling: LocalMapping::KeyFrameCulling. A local keyframe is bad when 90% of its points are seen by at
// least 3 other keyframes at the same or a finer scale (inertial: also bounded by the time to its neighbours);
// stops after 20 when the BA was interrupted, or after 100.
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace {

fw::bt::Status keyframe_culling(fw::Write<map::Atlas> atlas, fw::Read<local_mapping::LocalMapper> mapper) {
    (void)atlas;
    (void)mapper;  // TODO KeyFrameCulling
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(keyframe_culling,
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, read)));
