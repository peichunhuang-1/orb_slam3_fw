// clean_vo_matches: Track()'s "Clean VO matches" (the frame's points without observations leave it) and "Delete
// temporal MapPoints" (none are made: UpdateLastFrame makes them in localization mode only)
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

fw::bt::Status clean_vo_matches(fw::Write<tracking::Tracker> tracker) {
    tracking::Frame& f = tracker->current;
    for (std::size_t i = 0; i < f.points.size(); ++i) {
        if (f.points[i] == map::kNone) continue;
        const map::MapPoint* p = tracker->view.point(f.points[i]);
        if (!p || p->n_obs < 1) {
            f.outlier[i] = false;
            f.points[i] = map::kNone;
        }
    }
    tracker->temporal_points.clear();
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(clean_vo_matches, RESOURCES(("tracker", tracking::Tracker, write)));
