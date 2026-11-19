// finish_frame: the frame's reference keyframe (the tracker's if it has none) and mLastFrame = Frame(mCurrentFrame)
#include "fw/fw.hpp"
#include "types/tracking/tracker.hpp"

namespace {

fw::bt::Status finish_frame(fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    if (t.current.reference_kf == map::kNone) t.current.reference_kf = t.reference_kf;
    t.last = t.current;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(finish_frame, RESOURCES(("tracker", tracking::Tracker, write)));
