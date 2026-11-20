// handle_lost: the frame became LOST ("Reset if the camera get lost soon after initialization"): reset the active
// map with 10 keyframes or fewer or (inertial) before the IMU initialization, else a new map
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using namespace orb_tracking;

fw::bt::Status handle_lost(fw::InOut<tracking::State>& state, fw::Write<tracking::Tracker> tracker,
                           fw::Optional<tracking::AtlasEdits>& atlas_edits) {
    tracking::Tracker& t = *tracker;
    const std::uint64_t seq = t.next_edit_seq;
    const map::Atlas& a = tracker->view;
    if (!state.has_value()) state.emplace(tracking::State::LOST);
    if (a.keyFramesInMap() <= 10 || (t.inertial && !a.imuInitialized())) requestResetActiveMap(t);
    else createMapInAtlas(t, *state);
    sendEdits(t, seq, atlas_edits);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(handle_lost, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("tracker", tracking::Tracker, write)),
                      OUTPUTS(("atlas_edits", tracking::AtlasEdits)));
