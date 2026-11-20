// queue_empty: !CheckNewKeyFrames()
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"

namespace {

fw::bt::Status queue_empty(fw::Read<local_mapping::LocalMapper> mapper) {
    return mapper->queue.empty() ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(queue_empty, RESOURCES(("local_mapper", local_mapping::LocalMapper, read)));
