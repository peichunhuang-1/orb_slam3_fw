// search_in_neighbors: LocalMapping::SearchInNeighbors. The current keyframe's points are fused into its first
// and second neighbours (inertial: and the previous keyframes), then theirs into it (Fuse both ways); the fused
// points' descriptors, normals and depths and the covisibility are updated.
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace {

fw::bt::Status search_in_neighbors(fw::Write<map::Atlas> atlas, fw::Write<local_mapping::LocalMapper> mapper) {
    (void)atlas;
    (void)mapper;  // TODO SearchInNeighbors
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(search_in_neighbors,
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)));
