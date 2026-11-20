// map_point_culling: LocalMapping::MapPointCulling. The recent points that are bad, found in < 25% of the frames
// that should see them, or seen by too few keyframes (stereo: 3) two keyframes after their creation, are
// erased; those older than three keyframes leave the list.
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace {

fw::bt::Status map_point_culling(fw::Write<map::Atlas> atlas, fw::Write<local_mapping::LocalMapper> mapper) {
    (void)atlas;
    (void)mapper;  // TODO MapPointCulling
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(map_point_culling,
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)));
