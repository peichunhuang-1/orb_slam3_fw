// handle_lost: the frame became LOST ("Reset if the camera get lost soon after initialization"): reset the active
// map with 10 keyframes or fewer or (inertial) before the IMU initialization, else a new map
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using namespace orb_tracking;

fw::bt::Status handle_lost(fw::InOut<tracking::State>& state, fw::Write<map::Atlas> atlas,
                           fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    map::Atlas& a = *atlas;
    if (!state.has_value()) state.emplace(tracking::State::LOST);
    if (keyFramesInMap(a) <= 10 || (t.inertial && !imuInitialized(a))) requestResetActiveMap(t);
    else createMapInAtlas(a, t, *state);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(handle_lost, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("atlas", map::Atlas, write), ("tracker", tracking::Tracker, write)));
