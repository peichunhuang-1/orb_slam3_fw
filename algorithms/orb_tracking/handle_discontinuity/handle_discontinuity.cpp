// handle_discontinuity: what Tracking::Track() does on the discontinuities of frame_discontinuity, before it
// returns: bad IMU -> reset the active map; timestamp back -> new map (the IMU queue is cleared upstream, by
// imu_preintegration); jump in an inertial map, or a frame without IMU data in the middle of a run -> reset the
// active map, or a new map once inertial BA2 is done.
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using tracking::State;
using namespace orb_tracking;

fw::bt::Status handle_discontinuity(fw::InOut<State>& state, fw::Optional<const tracking::MapperStatus&> mapper,
                                    fw::Write<tracking::Tracker> tracker,
                                    fw::Optional<tracking::AtlasEdits>& atlas_edits) {
    tracking::Tracker& t = *tracker;
    const std::uint64_t seq = t.next_edit_seq;
    const map::Atlas& a = tracker->view;
    if (!state.has_value()) state.emplace(State::NO_IMAGES_YET);
    if (mapperOf(mapper).bad_imu) {
        requestResetActiveMap(t);
    } else if (t.last.timestamp > t.current.timestamp) {
        createMapInAtlas(t, *state);
    } else if (a.imuInitialized() && a.inertialBA2()) {
        createMapInAtlas(t, *state);
    } else {
        requestResetActiveMap(t);
    }
    sendEdits(t, seq, atlas_edits);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(handle_discontinuity,
                      INPUTS(("state", tracking::State, inout), ("mapper", tracking::MapperStatus, latest)),
                      RESOURCES(("tracker", tracking::Tracker, write)),
                      OUTPUTS(("atlas_edits", tracking::AtlasEdits)));
