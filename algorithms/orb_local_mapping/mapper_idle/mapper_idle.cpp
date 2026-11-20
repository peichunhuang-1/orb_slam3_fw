// mapper_idle: the end of Run()'s loop when nothing is being processed: ResetIfRequested, SetAcceptKeyFrames(true),
// and the status for tracking. Stop() comes with loop closing (stop_requested is never set before that); it only
// changes local_mapper's flags, so this leaf takes no atlas (a write would publish an empty version every tick).
#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "fw/fw.hpp"

namespace {

fw::bt::Status mapper_idle(fw::Write<local_mapping::LocalMapper> mapper, tracking::MapperStatus& status) {
    local_mapping::LocalMapper& m = *mapper;
    orb_local_mapping::resetIfRequested(m);
    m.processing = false;
    status = orb_local_mapping::makeStatus(m);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(mapper_idle, RESOURCES(("local_mapper", local_mapping::LocalMapper, write)),
                      OUTPUTS(("mapper", tracking::MapperStatus)));
