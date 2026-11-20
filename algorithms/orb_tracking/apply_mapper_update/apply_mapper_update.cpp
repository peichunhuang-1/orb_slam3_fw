// apply_mapper_update: Tracking::UpdateFrameIMU, which local mapping calls after an IMU initialization (scale s,
// bias b, its current keyframe). Here local mapping publishes them in its status with a sequence number and
// tracking applies each one once, at the start of a frame (between two frames, where the original's call lands
// when tracking does not hold the map).
//
// The new frame is not preintegrated yet: begin_frame gives it the new keyframe's bias. So of the original's
// mCurrentFrame update only the last frame's is done here (the original's mCurrentFrame between two frames is the
// last frame).
//
// Also InitializeIMU's last mpTracker->mState = Tracking::OK: applied once per tracking_ok_seq, after the update.
#include <memory>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "types/geometry/so3.hpp"

namespace {

constexpr float kGravity = 9.81f;  // IMU::GRAVITY_VALUE

void updateFrameImu(const tracking::MapperStatus& mapper, tracking::Tracker& t);

fw::bt::Status apply_mapper_update(fw::Optional<const tracking::MapperStatus&> mapper, fw::InOut<tracking::State>& state,
                                   fw::Write<tracking::Tracker> tracker) {
    if (!mapper.has_value()) return fw::bt::Status::SUCCESS;
    tracking::Tracker& t = *tracker;
    if (mapper->imu_update_seq != 0 && mapper->imu_update_seq != t.applied_imu_update) updateFrameImu(*mapper, t);
    if (mapper->tracking_ok_seq != 0 && mapper->tracking_ok_seq != t.applied_tracking_ok) {
        t.applied_tracking_ok = mapper->tracking_ok_seq;
        state.emplace(tracking::State::OK);
    }
    return fw::bt::Status::SUCCESS;
}

void updateFrameImu(const tracking::MapperStatus& mapper_status, tracking::Tracker& t) {
    const tracking::MapperStatus* mapper = &mapper_status;
    const map::Atlas& a = t.view;
    t.applied_imu_update = mapper->imu_update_seq;
    const float s = mapper->imu_scale;
    const map::KeyFrame* kf = a.keyframe(mapper->imu_keyframe);
    if (!kf) return;

    // The trajectory of the keyframe's map is scaled (through bad references to their parents)
    for (std::size_t k = 0; k < t.relative_poses.size(); ++k) {
        if (t.lost[k]) continue;
        const map::KeyFrame* r = a.keyframe(t.references[k]);
        while (r && r->bad) r = a.keyframe(r->parent);
        if (r && r->map == kf->map) {
            Eigen::Vector3f tr = t.relative_poses[k].translation() * s;
            t.relative_poses[k] = geometry::SE3f(t.relative_poses[k].unitQuaternion(), tr);
        }
    }

    t.last_bias = mapper->imu_bias;
    t.last_keyframe = kf->id;
    // The last frame's preintegration is also a keyframe's when it was made one: that one is the atlas's (local
    // mapping gives it the same bias), so the frame takes its own copy. Else it is the one since the last keyframe
    // (preintegrated_from_kf), changed in place as the original's shared pointer.
    if (t.last.preintegrated && t.last.preintegrated != t.preintegrated_from_kf)
        t.last.preintegrated = std::make_shared<imu::Preintegrated>(*t.last.preintegrated);
    t.last.setNewBias(t.last_bias);

    // The last frame's state from its keyframe and preintegration
    const map::KeyFrame* lkf = a.keyframe(t.last.last_keyframe);
    if (!t.last.valid() || !lkf) return;
    if (t.last.id == lkf->data->frame_id) {
        t.last.setImuPoseVelocity(lkf->imuRotation(), lkf->imuPosition(), lkf->velocity());
    } else if (t.last.preintegrated) {
        const imu::Preintegrated& p = *t.last.preintegrated;
        const Eigen::Vector3f Gz(0, 0, -kGravity);
        const Eigen::Vector3f twb1 = lkf->imuPosition();
        const Eigen::Matrix3f Rwb1 = lkf->imuRotation();
        const Eigen::Vector3f Vwb1 = lkf->velocity();
        const float t12 = p.dT;
        t.last.setImuPoseVelocity(geometry::normalizeRotation(Eigen::Matrix3f(Rwb1 * p.updatedDeltaRotation())),
                                  twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * p.updatedDeltaPosition(),
                                  Vwb1 + Gz * t12 + Rwb1 * p.updatedDeltaVelocity());
    }
    t.first_imu_frame_id = t.current.id;
    return;
}

}  // namespace

FW_REGISTER_BT_ACTION(apply_mapper_update, INPUTS(("mapper", tracking::MapperStatus, latest), ("state", tracking::State, inout)),
                      RESOURCES(("tracker", tracking::Tracker, write)));
