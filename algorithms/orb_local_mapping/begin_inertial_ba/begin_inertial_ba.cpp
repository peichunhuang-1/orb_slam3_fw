// begin_inertial_ba: the conditions of VIBA 1 and 2 in Run(), and SetIniertialBA1 / SetIniertialBA2 before the
// InitializeIMU that follows it. SUCCESS when that stage is due:
//   mTinit < 50, inertial, the IMU initialized, tracking OK, and
//   stage 1: !BA1 && mTinit > 5;   stage 2: BA1 && !BA2 && mTinit > 15
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"
#include "types/tracking/status.hpp"

namespace {

fw::bt::Status begin_inertial_ba(const int& stage, const bool& inertial, fw::Optional<const tracking::Status&> tracking,
                                 fw::Write<map::Atlas> atlas, fw::Read<local_mapping::LocalMapper> mapper) {
    (void)stage;
    (void)inertial;
    (void)tracking;
    (void)atlas;
    (void)mapper;  // TODO
    return fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_ACTION(begin_inertial_ba,
                      INPUTS(("stage", int, latest), ("inertial", bool, latest), ("tracking", tracking::Status, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, read)));
