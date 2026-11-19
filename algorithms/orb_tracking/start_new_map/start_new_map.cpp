// start_new_map: Track() with the state LOST at the start of the frame ("A new map is started"): reset the active
// map with fewer than 10 keyframes, else a new map; no last keyframe. Nothing else is done with the frame.
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using namespace orb_tracking;

fw::bt::Status start_new_map(fw::InOut<tracking::State>& state, fw::Write<map::Atlas> atlas,
                             fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    if (!state.has_value()) state.emplace(tracking::State::LOST);
    if (keyFramesInMap(*atlas) < 10) requestResetActiveMap(t);
    else createMapInAtlas(*atlas, t, *state);
    t.last_keyframe = map::kNone;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(start_new_map, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("atlas", map::Atlas, write), ("tracker", tracking::Tracker, write)));
