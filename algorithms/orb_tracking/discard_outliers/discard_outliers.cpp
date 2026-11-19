// discard_outliers: the Huber outliers stay in the new keyframe (bundle adjustment decides) but not in the frame
// the next one is tracked from
#include "fw/fw.hpp"
#include "types/tracking/tracker.hpp"

namespace {

fw::bt::Status discard_outliers(fw::Write<tracking::Tracker> tracker) {
    tracking::Frame& f = tracker->current;
    for (std::size_t i = 0; i < f.points.size(); ++i)
        if (f.points[i] != map::kNone && f.outlier[i]) f.points[i] = map::kNone;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(discard_outliers, RESOURCES(("tracker", tracking::Tracker, write)));
