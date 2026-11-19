// lost_time_within: RECENTLY_LOST for at most max_s (time_recently_lost 5 s inertial, 3 s stereo); over it the
// state becomes LOST and the node fails
#include "fw/fw.hpp"
#include "types/tracking/state.hpp"
#include "types/tracking/tracker.hpp"

namespace {

using tracking::State;

fw::bt::Status lost_time_within(fw::InOut<State>& state, const double& max_s, fw::Read<tracking::Tracker> tracker) {
    if (tracker->current.timestamp - tracker->timestamp_lost > max_s) {
        if (!state.has_value()) state.emplace(State::LOST);
        *state = State::LOST;
        return fw::bt::Status::FAILURE;
    }
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(lost_time_within, INPUTS(("state", tracking::State, inout), ("max_s", double, latest)),
                      RESOURCES(("tracker", tracking::Tracker, read)));
