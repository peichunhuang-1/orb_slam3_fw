// mapper_idle: the end of Run()'s loop when nothing is being processed: Stop() (loop closing, later),
// ResetIfRequested, SetAcceptKeyFrames(true), and the status for tracking.
#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "fw/fw.hpp"

namespace {

fw::bt::Status mapper_idle(fw::Write<map::Atlas> atlas, fw::Write<local_mapping::LocalMapper> mapper,
                           tracking::MapperStatus& status) {
    (void)atlas;  // TODO Stop(): the safe area to stop for loop closing
    local_mapping::LocalMapper& m = *mapper;
    orb_local_mapping::resetIfRequested(m);
    m.processing = false;
    status = orb_local_mapping::makeStatus(m);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(mapper_idle,
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)),
                      OUTPUTS(("mapper", tracking::MapperStatus)));
