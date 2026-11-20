// use_reference_keyframe: Track()'s choice of TrackReferenceKeyFrame over the motion model: no velocity and the
// IMU not initialized, or a relocalization less than 2 frames ago
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

fw::bt::Status use_reference_keyframe(fw::Read<tracking::Tracker> tracker) {
    const tracking::Tracker& t = *tracker;
    const bool ref = (!t.has_velocity && !orb_tracking::imuInitialized(tracker->view)) ||
                     t.current.id < t.last_reloc_frame_id + 2;
    return ref ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(use_reference_keyframe,
                         RESOURCES(("tracker", tracking::Tracker, read)));
