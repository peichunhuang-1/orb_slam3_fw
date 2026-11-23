// is_merge_pending: a merge from loop closing waits to be done (apply_loop_result took it)
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"

namespace {

fw::bt::Status is_merge_pending(fw::Read<local_mapping::LocalMapper> mapper) {
    return mapper->merge.pending ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(is_merge_pending, RESOURCES(("local_mapper", local_mapping::LocalMapper, read)));
