// frame_discontinuity: the checks at the start of Tracking::Track() after which the frame is not tracked: local
// mapping's bad-IMU flag, a timestamp older than the last frame's, or (inertial map) a jump of more than 1 s.
// And one the original does not have: an inertial tracker getting a frame without IMU data in the middle of a
// run (a last frame, no map just created). Its interval would be missing from the preintegration from the last
// keyframe, whose dT then falls short of the time elapsed and whose ΔV, ΔP leave the interval's motion out:
// handled as the jump (handle_discontinuity). imu_preintegration only sends such frames first and after the
// time goes back (both handled already), or when it has no samples on one side of the interval.
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using tracking::State;

fw::bt::Status frame_discontinuity(const State& state, fw::Optional<const tracking::MapperStatus&> mapper,
                                   fw::Read<tracking::Tracker> tracker) {
    const tracking::Tracker& t = *tracker;
    if (orb_tracking::mapperOf(mapper).bad_imu) return fw::bt::Status::SUCCESS;
    if (state == State::NO_IMAGES_YET) return fw::bt::Status::FAILURE;
    if (t.last.timestamp > t.current.timestamp) return fw::bt::Status::SUCCESS;
    const map::Map* m = tracker->view.findMap(tracker->view.current);
    if (t.current.timestamp > t.last.timestamp + 1.0 && m && m->inertial) return fw::bt::Status::SUCCESS;
    if (t.inertial && !t.created_map && t.last.valid() && t.current.features && t.current.features->stereo &&
        !t.current.features->stereo->imu)
        return fw::bt::Status::SUCCESS;
    return fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(frame_discontinuity,
                         INPUTS(("state", tracking::State, latest), ("mapper", tracking::MapperStatus, latest)),
                         RESOURCES(("tracker", tracking::Tracker, read)));
