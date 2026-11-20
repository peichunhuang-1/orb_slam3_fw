// check_replaced_in_last_frame: Tracking::CheckReplacedInLastFrame: the last frame's points that local mapping
// fused into another point are replaced by it (one step, as GetReplaced)
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

fw::bt::Status check_replaced_in_last_frame(fw::Write<tracking::Tracker> tracker) {
    for (map::MapPointId& id : tracker->last.points) {
        const map::MapPoint* p = tracker->view.point(id);
        if (p && p->replaced != map::kNone) id = p->replaced;
    }
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(check_replaced_in_last_frame,
                      RESOURCES(("tracker", tracking::Tracker, write)));
