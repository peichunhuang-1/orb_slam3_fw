// is_loop_pending: a loop from loop closing waits for its correction (apply_loop_result took it)
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"

namespace {

fw::bt::Status is_loop_pending(fw::Read<local_mapping::LocalMapper> mapper) {
    return mapper->loop.pending ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(is_loop_pending, RESOURCES(("local_mapper", local_mapping::LocalMapper, read)));
