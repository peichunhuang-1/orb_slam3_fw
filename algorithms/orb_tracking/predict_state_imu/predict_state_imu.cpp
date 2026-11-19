// predict_state_imu: RECENTLY_LOST, inertial: the pose from the IMU (PredictStateIMU, its result not checked, as
// the original); FAILURE before the IMU is initialized
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

fw::bt::Status predict_state_imu(fw::Read<map::Atlas> atlas, fw::Write<tracking::Tracker> tracker) {
    if (!orb_tracking::imuInitialized(*atlas)) return fw::bt::Status::FAILURE;
    orb_tracking::predictStateImu(*atlas, *tracker);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(predict_state_imu, RESOURCES(("atlas", map::Atlas, read), ("tracker", tracking::Tracker, write)));
