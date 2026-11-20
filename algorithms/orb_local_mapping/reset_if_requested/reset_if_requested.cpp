// reset_if_requested: ResetIfRequested at the end of one keyframe's processing in Run()
#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "fw/fw.hpp"

namespace {

fw::bt::Status reset_if_requested(fw::Write<local_mapping::LocalMapper> mapper) {
    orb_local_mapping::resetIfRequested(*mapper);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(reset_if_requested, RESOURCES(("local_mapper", local_mapping::LocalMapper, write)));
