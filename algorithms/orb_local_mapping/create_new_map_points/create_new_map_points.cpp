// create_new_map_points: LocalMapping::CreateNewMapPoints (triangulation.hpp). Matches with the best covisible
// keyframes (inertial: also the previous ones) by SearchForTriangulation, triangulates, checks parallax, depth,
// reprojection and scale consistency; the new points join the recent ones. bCoarse when inertial, RECENTLY_LOST
// and BA2 is done. Then mbAbortBA = false (Run(), after CreateNewMapPoints).
#include "algorithms/orb_local_mapping/create_new_map_points/triangulation.hpp"
#include "fw/fw.hpp"
#include "types/tracking/status.hpp"

namespace {

fw::bt::Status create_new_map_points(fw::Optional<const tracking::Status&> tracking, fw::Write<map::Atlas> atlas,
                                     fw::Write<local_mapping::LocalMapper> mapper) {
    const bool recently_lost = tracking.has_value() && tracking->state == tracking::State::RECENTLY_LOST;
    orb_local_mapping::createNewMapPoints(*atlas, *mapper, recently_lost);
    mapper->abort_ba = false;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(create_new_map_points, INPUTS(("tracking", tracking::Status, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)));
