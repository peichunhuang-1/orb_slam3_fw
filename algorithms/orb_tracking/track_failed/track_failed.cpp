// track_failed: !bOK. A frame that was OK (its pose estimate succeeded, the local map failed) becomes
// RECENTLY_LOST from now; inertial: the active map is reset when the IMU is not or recently initialized (no BA2)
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using tracking::State;
using namespace orb_tracking;

fw::bt::Status track_failed(fw::InOut<State>& state, fw::Write<tracking::Tracker> tracker,
                            bool& ok) {
    tracking::Tracker& t = *tracker;
    ok = false;
    if (state.has_value() && *state == State::OK) {
        if (t.inertial && (!imuInitialized(tracker->view) || !inertialBA2(tracker->view))) requestResetActiveMap(t);
        *state = State::RECENTLY_LOST;
        t.timestamp_lost = t.current.timestamp;
    }
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(track_failed, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("tracker", tracking::Tracker, write)),
                      OUTPUTS(("ok", bool)));
