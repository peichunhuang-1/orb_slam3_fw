// frame_discontinuity: the checks at the start of Tracking::Track() after which the frame is not tracked: local
// mapping's bad-IMU flag, a timestamp older than the last frame's, or (inertial map) a jump of more than 1 s.
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using tracking::State;

fw::bt::Status frame_discontinuity(const State& state, fw::Optional<const tracking::MapperStatus&> mapper,
                                   fw::Read<map::Atlas> atlas, fw::Read<tracking::Tracker> tracker) {
    const tracking::Tracker& t = *tracker;
    if (orb_tracking::mapperOf(mapper).bad_imu) return fw::bt::Status::SUCCESS;
    if (state == State::NO_IMAGES_YET) return fw::bt::Status::FAILURE;
    if (t.last.timestamp > t.current.timestamp) return fw::bt::Status::SUCCESS;
    const map::Map* m = atlas->findMap(atlas->current);
    if (t.current.timestamp > t.last.timestamp + 1.0 && m && m->inertial) return fw::bt::Status::SUCCESS;
    return fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(frame_discontinuity,
                         INPUTS(("state", tracking::State, latest), ("mapper", tracking::MapperStatus, latest)),
                         RESOURCES(("atlas", map::Atlas, read), ("tracker", tracking::Tracker, read)));
