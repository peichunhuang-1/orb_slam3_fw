// imu_bias_bookkeeping: Track() after the local map, IMU initialized and bOK: ResetFrameIMU at mnFramesToResetIMU
// frames after a relocalization (empty in ORB-SLAM3), else mLastBias = the frame's bias 30 frames after it.
// (The frame copy the original saves for the IMU reset is never used.)
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

fw::bt::Status imu_bias_bookkeeping(const bool& ok, fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    if (!orb_tracking::imuInitialized(tracker->view) || !ok) return fw::bt::Status::SUCCESS;
    if (t.current.id == t.last_reloc_frame_id + t.frames_to_reset_imu) {
        // ResetFrameIMU: nothing
    } else if (t.current.id > t.last_reloc_frame_id + 30) {
        t.last_bias = t.current.bias;
    }
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(imu_bias_bookkeeping, INPUTS(("ok", bool, latest)),
                      RESOURCES(("tracker", tracking::Tracker, write)));
