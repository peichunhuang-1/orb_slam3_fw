// start_new_map: Track() with the state LOST at the start of the frame ("A new map is started"): reset the active
// map with fewer than 10 keyframes, else a new map; no last keyframe. Nothing else is done with the frame.
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using namespace orb_tracking;

fw::bt::Status start_new_map(fw::InOut<tracking::State>& state, fw::Write<tracking::Tracker> tracker,
                             fw::Optional<tracking::AtlasEdits>& atlas_edits) {
    tracking::Tracker& t = *tracker;
    const std::uint64_t seq = t.next_edit_seq;
    if (!state.has_value()) state.emplace(tracking::State::LOST);
    if (keyFramesInMap(tracker->view) < 10) requestResetActiveMap(t);
    else createMapInAtlas(t, *state);
    t.last_keyframe = map::kNone;
    sendEdits(t, seq, atlas_edits);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(start_new_map, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("tracker", tracking::Tracker, write)),
                      OUTPUTS(("atlas_edits", tracking::AtlasEdits)));
