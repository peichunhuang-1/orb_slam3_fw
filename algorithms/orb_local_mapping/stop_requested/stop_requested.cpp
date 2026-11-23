// stop_requested: stopRequested(), the condition of the correction branch (loop closing's RequestStop: a loop to
// correct, algorithms/orb_loop_closing/docs/DESIGN.md, "local mapping 的樹")
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"

namespace {

fw::bt::Status stop_requested(fw::Read<local_mapping::LocalMapper> mapper) {
    return mapper->stop_requested ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(stop_requested, RESOURCES(("local_mapper", local_mapping::LocalMapper, read)));
