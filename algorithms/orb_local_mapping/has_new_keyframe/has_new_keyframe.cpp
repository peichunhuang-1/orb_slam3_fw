// has_new_keyframe: CheckNewKeyFrames() && !mbBadImu, the condition of Run()'s keyframe branch
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"

namespace {

fw::bt::Status has_new_keyframe(fw::Read<local_mapping::LocalMapper> mapper) {
    return !mapper->queue.empty() && !mapper->bad_imu ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(has_new_keyframe, RESOURCES(("local_mapper", local_mapping::LocalMapper, read)));
