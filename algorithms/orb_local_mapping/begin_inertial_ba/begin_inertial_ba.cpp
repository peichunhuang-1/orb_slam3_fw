// begin_inertial_ba: the conditions of VIBA 1 and 2 in Run(), and SetIniertialBA1 / SetIniertialBA2 before the
// InitializeIMU that follows it. SUCCESS when that stage is due:
//   mTinit < 50, inertial, the current keyframe's map IMU-initialized, tracking OK (its last status), and
//   stage 1: !BA1 && mTinit > 5;   stage 2: BA1 && !BA2 && mTinit > 15
// The flag is set before InitializeIMU runs, as the original (it stays set when InitializeIMU returns early).
#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"
#include "types/tracking/status.hpp"

namespace {

fw::bt::Status begin_inertial_ba(const int& stage, const bool& inertial, fw::Optional<const tracking::Status&> tracking,
                                 fw::Write<map::Atlas> atlas, fw::Read<local_mapping::LocalMapper> mapper) {
    using fw::bt::Status;
    const local_mapping::LocalMapper& m = *mapper;
    if (!(m.t_init < 50.f) || !inertial) return Status::FAILURE;
    const map::Map* map = orb_local_mapping::currentMap(*atlas, m);
    if (!map || !map->imu_initialized) return Status::FAILURE;
    if (!tracking.has_value() || tracking->state != tracking::State::OK) return Status::FAILURE;
    if (stage == 1) {
        if (map->imu_ba1 || !(m.t_init > 5.f)) return Status::FAILURE;
        atlas->editMap(map->id)->imu_ba1 = true;
        return Status::SUCCESS;
    }
    if (!map->imu_ba1 || map->imu_ba2 || !(m.t_init > 15.f)) return Status::FAILURE;
    atlas->editMap(map->id)->imu_ba2 = true;
    return Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(begin_inertial_ba,
                      INPUTS(("stage", int, latest), ("inertial", bool, latest), ("tracking", tracking::Status, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, read)));
