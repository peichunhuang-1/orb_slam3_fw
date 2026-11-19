// remember_last_frame: StereoInitialization failed: mLastFrame = Frame(mCurrentFrame), wait for the next frame
#include "fw/fw.hpp"
#include "types/tracking/tracker.hpp"

namespace {

fw::bt::Status remember_last_frame(fw::Write<tracking::Tracker> tracker) {
    tracker->last = tracker->current;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(remember_last_frame, RESOURCES(("tracker", tracking::Tracker, write)));
