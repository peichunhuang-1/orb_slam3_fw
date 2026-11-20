// keyframe_culling: LocalMapping::KeyFrameCulling (culling.hpp). A local keyframe is bad when 90% of its points
// are seen by at least 3 other keyframes at the same or a finer scale (inertial: 50%, and bounded by the time to
// its neighbours); stops after 20 when the BA was interrupted, or after 100.
#include "algorithms/orb_local_mapping/keyframe_culling/culling.hpp"
#include "fw/fw.hpp"

namespace {

fw::bt::Status keyframe_culling(fw::Write<map::Atlas> atlas, fw::Read<local_mapping::LocalMapper> mapper) {
    orb_local_mapping::keyFrameCulling(*atlas, *mapper);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(keyframe_culling,
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, read)));
