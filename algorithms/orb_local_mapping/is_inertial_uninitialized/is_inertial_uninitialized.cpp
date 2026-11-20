// is_inertial_uninitialized: !mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial
#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "fw/fw.hpp"

namespace {

fw::bt::Status is_inertial_uninitialized(const bool& inertial, fw::Read<map::Atlas> atlas,
                                         fw::Read<local_mapping::LocalMapper> mapper) {
    if (!inertial) return fw::bt::Status::FAILURE;
    const map::Map* m = orb_local_mapping::currentMap(*atlas, *mapper);
    return m && !m->imu_initialized ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(is_inertial_uninitialized, INPUTS(("inertial", bool, latest)),
                         RESOURCES(("atlas", map::Atlas, read), ("local_mapper", local_mapping::LocalMapper, read)));
