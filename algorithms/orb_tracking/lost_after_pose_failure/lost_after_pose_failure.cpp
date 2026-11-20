// lost_after_pose_failure: Track() when the pose estimate of an OK frame failed: LOST within mnFramesToResetIMU
// frames of a relocalization (inertial) or with 10 keyframes or fewer, else RECENTLY_LOST from this frame's time
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using tracking::State;

fw::bt::Status lost_after_pose_failure(fw::InOut<State>& state, fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    if (!state.has_value()) state.emplace(State::OK);
    if (t.current.id <= t.last_reloc_frame_id + t.frames_to_reset_imu && t.inertial) {
        *state = State::LOST;
    } else if (orb_tracking::keyFramesInMap(tracker->view) > 10) {
        *state = State::RECENTLY_LOST;
        t.timestamp_lost = t.current.timestamp;
    } else {
        *state = State::LOST;
    }
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(lost_after_pose_failure, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("tracker", tracking::Tracker, write)));
