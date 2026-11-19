// handle_discontinuity: what Tracking::Track() does on the discontinuities of frame_discontinuity, before it
// returns: bad IMU -> reset the active map; timestamp back -> new map (the IMU queue is cleared upstream, by
// imu_preintegration); jump in an inertial map -> reset the active map, or a new map once inertial BA2 is done.
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using tracking::State;
using namespace orb_tracking;

fw::bt::Status handle_discontinuity(fw::InOut<State>& state, fw::Optional<const tracking::MapperStatus&> mapper,
                                    fw::Write<map::Atlas> atlas, fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    map::Atlas& a = *atlas;
    if (!state.has_value()) state.emplace(State::NO_IMAGES_YET);
    if (mapperOf(mapper).bad_imu) {
        requestResetActiveMap(t);
    } else if (t.last.timestamp > t.current.timestamp) {
        createMapInAtlas(a, t, *state);
    } else if (imuInitialized(a) && inertialBA2(a)) {
        createMapInAtlas(a, t, *state);
    } else {
        requestResetActiveMap(t);
    }
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(handle_discontinuity,
                      INPUTS(("state", tracking::State, inout), ("mapper", tracking::MapperStatus, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("tracker", tracking::Tracker, write)));
