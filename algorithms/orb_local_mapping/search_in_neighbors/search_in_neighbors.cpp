// search_in_neighbors: LocalMapping::SearchInNeighbors (fuse.hpp). The current keyframe's points are fused into its
// first and second neighbours (inertial: and the previous keyframes), then theirs into it (Fuse both ways); the
// fused points' descriptors, normals and depths and the covisibility are updated. abort_ba is read as ORB-SLAM3
// reads mbAbortBA, but only another tick sets it, and create_new_map_points cleared it in this one.
#include "algorithms/orb_local_mapping/search_in_neighbors/fuse.hpp"
#include "fw/fw.hpp"

namespace {

fw::bt::Status search_in_neighbors(fw::Write<map::Atlas> atlas, fw::Read<local_mapping::LocalMapper> mapper) {
    orb_local_mapping::searchInNeighbors(*atlas, *mapper);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(search_in_neighbors,
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, read)));
