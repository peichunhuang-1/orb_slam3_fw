// predict_state_imu: RECENTLY_LOST, inertial: the pose from the IMU (PredictStateIMU, its result not checked, as
// the original); FAILURE before the IMU is initialized
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

fw::bt::Status predict_state_imu(fw::Write<tracking::Tracker> tracker) {
    if (!tracker->view.imuInitialized()) return fw::bt::Status::FAILURE;
    orb_tracking::predictStateImu(tracker->view, *tracker);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(predict_state_imu, RESOURCES(("tracker", tracking::Tracker, write)));
