// stereo_initialization: Tracking::StereoInitialization. With more than 500 keypoints (inertial: preintegrated,
// and the mean acceleration changed by 0.5 m/s² since the last frame) the frame becomes the first keyframe of the
// current map, at the origin (inertial: the body at the origin), with a map point per stereo match. FAILURE: not
// initialized (the frame is kept as the last one by remember_last_frame).
#include <memory>
#include <utility>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "types/map/atlas_ops.hpp"

namespace {

using tracking::State;

fw::bt::Status stereo_initialization(fw::InOut<State>& state, fw::Write<tracking::Tracker> tracker,
                                     fw::Optional<tracking::AtlasEdits>& atlas_edits) {
    tracking::Tracker& t = *tracker;
    const std::uint64_t seq = t.next_edit_seq;
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

    // The keyframe, in the map now, with a point per stereo match at its camera coordinates (the keyframe is at
    // the origin); InsertKeyFrame
    tracking::AtlasEdit e;
    e.kind = tracking::AtlasEdit::Kind::keyframe;
    e.keyframe = orb_tracking::keyFrameOf(f);
    map::prepareKeyFrame(t.view, e.keyframe);
    // The original's first keyframe shares the preintegration since it with the frames until the next keyframe
    // (unused: it has no previous keyframe). Here tracking keeps appending to that one (begin_frame) while local
    // mapping may read the keyframe's: the keyframe gets its own copy.
    if (e.keyframe.preintegrated)
        e.keyframe.preintegrated = std::make_shared<const imu::Preintegrated>(*e.keyframe.preintegrated);
    e.insert = true;
    e.origin = true;  // mvpKeyFrameOrigins
    const map::KeyFrameId kf = e.keyframe.id;
    const orb::FrameFeatures& ff = *f.features;
    map::MapPointId next = t.view.next_tracking_point_id;
    for (int i = 0; i < ff.n_left; ++i) {
        const int r = ff.rightOf(i);
        if (r == -1) continue;
        const map::MapPointId p = next;
        next += 2;
        e.points.push_back({p, i, ff.stereoPoint(i)});
        f.points[std::size_t(i)] = p;
        f.points[std::size_t(r + ff.n_left)] = p;
    }
    orb_tracking::flushCounts(t);
    orb_tracking::addEdit(t, std::move(e));
    orb_tracking::sendEdits(t, seq, atlas_edits);

    t.last = f;  // before the reference keyframe is set, as the original
    t.last_keyframe_frame_id = f.id;
    t.last_keyframe = kf;
    t.local_keyframes.push_back(kf);
    t.local_points = map::mapPoints(t.view, t.view.current);  // GetAllMapPoints
    t.reference_kf = kf;
    f.reference_kf = kf;
    t.new_keyframe = true;
    if (!state.has_value()) state.emplace(State::OK);
    *state = State::OK;
    if (t.view.maps.size() == 1) t.first_frame_id = f.id;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(stereo_initialization, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("tracker", tracking::Tracker, write)),
                      OUTPUTS(("atlas_edits", tracking::AtlasEdits)));
