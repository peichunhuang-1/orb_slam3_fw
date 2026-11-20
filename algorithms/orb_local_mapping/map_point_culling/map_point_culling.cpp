// map_point_culling: LocalMapping::MapPointCulling (culling.hpp).
#include "algorithms/orb_local_mapping/map_point_culling/culling.hpp"
#include "fw/fw.hpp"

namespace {

fw::bt::Status map_point_culling(fw::Write<map::Atlas> atlas, fw::Write<local_mapping::LocalMapper> mapper) {
    orb_local_mapping::mapPointCulling(*atlas, *mapper);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(map_point_culling,
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)));
