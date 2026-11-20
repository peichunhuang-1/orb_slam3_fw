// handle_mapper_command: tracking's RequestReset, RequestResetActiveMap and InterruptBA. The resets are done by
// reset_if_requested / mapper_idle when the keyframe being processed is done (ResetIfRequested's place in Run());
// tracking does not wait for them.
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/tracking/mapper_command.hpp"

namespace {

fw::bt::Status handle_mapper_command(const tracking::MapperCommand& mapper_cmd,
                                     fw::Write<local_mapping::LocalMapper> mapper) {
    local_mapping::LocalMapper& m = *mapper;
    if (mapper_cmd.reset_all) {
        m.reset = local_mapping::PendingReset::all;
    } else if (mapper_cmd.reset_active_map && m.reset != local_mapping::PendingReset::all) {
        m.reset = local_mapping::PendingReset::active_map;
        m.reset_map = mapper_cmd.map_id;
    }
    if (mapper_cmd.interrupt_ba) m.abort_ba = true;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(handle_mapper_command, INPUTS(("mapper_cmd", tracking::MapperCommand, latest)),
                      RESOURCES(("local_mapper", local_mapping::LocalMapper, write)));
