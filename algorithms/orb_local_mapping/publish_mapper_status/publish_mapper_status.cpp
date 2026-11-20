// publish_mapper_status: what tracking reads of local mapping, sent every tick (the original's tracking polls it).
#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "fw/fw.hpp"

namespace {

fw::bt::Status publish_mapper_status(fw::Read<local_mapping::LocalMapper> mapper, tracking::MapperStatus& status) {
    status = orb_local_mapping::makeStatus(*mapper);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(publish_mapper_status, RESOURCES(("local_mapper", local_mapping::LocalMapper, read)),
                      OUTPUTS(("mapper", tracking::MapperStatus)));
