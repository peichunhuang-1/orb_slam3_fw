// update_motion_model: mVelocity = Tcw · Twl of the last two frames, when both have a pose
#include "fw/fw.hpp"
#include "types/tracking/tracker.hpp"

namespace {

fw::bt::Status update_motion_model(fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    if (t.last.hasPose() && t.current.hasPose()) {
        t.velocity = t.current.pose() * t.last.pose().inverse();
        t.has_velocity = true;
    } else {
        t.has_velocity = false;
    }
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(update_motion_model, RESOURCES(("tracker", tracking::Tracker, write)));
