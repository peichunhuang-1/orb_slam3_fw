// handle_mapper_command: tracking's InterruptBA. Its resets come as atlas edits (apply_tracking_edits), in order
// with its keyframes.
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/tracking/mapper_command.hpp"

namespace {

fw::bt::Status handle_mapper_command(const tracking::MapperCommand& mapper_cmd,
                                     fw::Write<local_mapping::LocalMapper> mapper) {
    if (mapper_cmd.interrupt_ba) mapper->abort_ba = true;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(handle_mapper_command, INPUTS(("mapper_cmd", tracking::MapperCommand, latest)),
                      RESOURCES(("local_mapper", local_mapping::LocalMapper, write)));
