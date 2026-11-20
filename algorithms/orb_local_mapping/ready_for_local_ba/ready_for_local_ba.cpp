// ready_for_local_ba: !CheckNewKeyFrames() && !stopRequested(), before the local BA, IMU initialization and
// keyframe culling
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"

namespace {

fw::bt::Status ready_for_local_ba(fw::Read<local_mapping::LocalMapper> mapper) {
    return mapper->queue.empty() && !mapper->stop_requested ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(ready_for_local_ba, RESOURCES(("local_mapper", local_mapping::LocalMapper, read)));
