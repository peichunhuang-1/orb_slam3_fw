// stereo_initialization: Tracking::StereoInitialization. With more than 500 keypoints (inertial: preintegrated,
// and the mean acceleration changed by 0.5 m/s² since the last frame) the frame becomes the first keyframe of the
// current map, at the origin (inertial: the body at the origin), with a map point per stereo match. FAILURE: not
// initialized (the frame is kept as the last one by remember_last_frame).
#include <memory>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "types/map/atlas_ops.hpp"
#include "types/map/keyframe_ref.hpp"

namespace {

using tracking::State;

fw::bt::Status stereo_initialization(fw::InOut<State>& state, fw::Write<map::Atlas> atlas,
                                     fw::Write<tracking::Tracker> tracker, fw::Optional<map::KeyFrameRef>& keyframe) {
    tracking::Tracker& t = *tracker;
    map::Atlas& a = *atlas;
    tracking::Frame& f = t.current;
    if (f.size() <= 500) return fw::bt::Status::FAILURE;

    if (t.inertial) {
        if (!f.preintegrated || !t.last.preintegrated) return fw::bt::Status::FAILURE;  // "not IMU meas"
        if (!f.preintegrated_frame || !t.last.preintegrated_frame ||
            (f.preintegrated_frame->avgA - t.last.preintegrated_frame->avgA).norm() < 0.5f)
            return fw::bt::Status::FAILURE;  // "not enough acceleration"
        t.preintegrated_from_kf = std::make_shared<imu::Preintegrated>(imu::Bias(), t.imu_noise);
        f.preintegrated = t.preintegrated_from_kf;
        f.setImuPoseVelocity(f.T_cb.rotationMatrix(), f.T_cb.translation(), Eigen::Vector3f::Zero());
    } else {
        f.setPose(geometry::SE3f());
    }

    const map::KeyFrameId kf = map::createKeyFrame(a, orb_tracking::keyFrameOf(f));
    map::insertKeyFrame(a, kf);

    // The stereo points, at their camera coordinates (the keyframe is at the origin)
    const orb::FrameFeatures& ff = *f.features;
    for (int i = 0; i < ff.n_left; ++i) {
        const int r = ff.rightOf(i);
        if (r == -1) continue;
        const map::MapPointId p = map::createStereoPoint(a, kf, i, ff.stereoPoint(i));
        f.points[std::size_t(i)] = p;
        f.points[std::size_t(r + ff.n_left)] = p;
    }

    keyframe = map::KeyFrameRef{a.current, kf};  // InsertKeyFrame

    t.last = f;  // before the reference keyframe is set, as the original
    t.last_keyframe_frame_id = f.id;
    t.last_keyframe = kf;
    t.local_keyframes.push_back(kf);
    t.local_points = map::mapPoints(a, a.current);  // GetAllMapPoints
    map::Map& m = a.editCurrentMap();
    t.reference_kf = kf;
    f.reference_kf = kf;
    m.reference_points = t.local_points;
    m.keyframe_origins.push_back(kf);
    t.new_keyframe = true;
    if (!state.has_value()) state.emplace(State::OK);
    *state = State::OK;
    if (a.maps.size() == 1) t.first_frame_id = f.id;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(stereo_initialization, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("atlas", map::Atlas, write), ("tracker", tracking::Tracker, write)),
                      OUTPUTS(("keyframe", map::KeyFrameRef)));
