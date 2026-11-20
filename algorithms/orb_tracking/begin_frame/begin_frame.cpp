// begin_frame: Tracking::Track() up to the state branch. SetNewBias with the last keyframe's bias,
// NO_IMAGES_YET -> NOT_INITIALIZED, mLastProcessedState, the IMU (PreintegrateIMU, except right after
// CreateMapInAtlas), and the map change index (mbMapUpdated).
//
// PreintegrateIMU is done upstream (imu_preintegration, at zero bias; DESIGN.md, "IMU 預積分"): here the frame's
// preintegration is taken at the last frame's bias, the original's linearization point, by its bias Jacobians,
// and appended to the one from the last keyframe. A frame without IMU data keeps no preintegration, as the
// original's early returns.
#include <memory>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using tracking::State;

fw::bt::Status begin_frame(fw::InOut<State>& state, fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    const map::Atlas& a = tracker->view;
    tracking::Frame& f = t.current;
    if (!state.has_value()) state.emplace(State::NO_IMAGES_YET);

    if (t.inertial)
        if (const map::KeyFrame* kf = a.keyframe(t.last_keyframe)) f.setNewBias(kf->bias);
    if (*state == State::NO_IMAGES_YET) *state = State::NOT_INITIALIZED;
    t.last_processed_state = *state;

    if (t.inertial && !t.created_map) {
        const std::shared_ptr<const imu::Preintegrated>& imu = f.features->stereo->imu;
        if (imu && t.preintegrated_from_kf) {
            auto own = std::make_shared<imu::Preintegrated>(*imu);
            own->setNewBias(t.last.bias);  // the original integrates from the last frame at its bias
            t.preintegrated_from_kf->append(*imu);
            f.preintegrated_frame = own;
            f.preintegrated = t.preintegrated_from_kf;
            f.last_keyframe = t.last_keyframe;
        }
        f.imu_preintegrated = true;
    }
    t.created_map = false;

    const map::Map& m = a.currentMap();  // made by make_frame
    int& notified = t.change_notified[m.id];  // the map's mnMapChangeNotified
    t.map_updated = false;
    if (m.change_index > notified) {
        notified = m.change_index;
        t.map_updated = true;
    }
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(begin_frame, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("tracker", tracking::Tracker, write)));
