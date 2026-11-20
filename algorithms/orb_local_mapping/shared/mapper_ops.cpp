#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"

namespace orb_local_mapping {

tracking::MapperStatus makeStatus(const local_mapping::LocalMapper& m) {
    tracking::MapperStatus s;
    s.accept_keyframes = !m.processing;
    s.keyframes_in_queue = int(m.queue.size());
    s.initializing = m.initializing;
    s.stopped = m.stopped;
    s.stop_requested = m.stop_requested;
    s.bad_imu = m.bad_imu;
    s.far_points = m.far_points;
    s.th_far_points = m.th_far_points;
    s.imu_update_seq = m.imu_update_seq;
    s.imu_scale = m.imu_scale;
    s.imu_bias = m.imu_bias;
    s.imu_keyframe = m.imu_keyframe;
    s.tracking_ok_seq = m.tracking_ok_seq;
    return s;
}

void resetIfRequested(local_mapping::LocalMapper& m) {
    if (m.reset == local_mapping::PendingReset::none) return;
    // Both kinds do the same here. The atlas was reset, and the queue emptied, when tracking's edit was applied
    // (apply_tracking_edits): the keyframes queued since are after the reset and stay.
    m.recent_points.clear();
    m.t_init = 0;
    m.bad_imu = false;
    m.idx_init = 0;  // mbResetRequested only; harmless for the active map's reset, which never reads it again
    m.reset = local_mapping::PendingReset::none;
    m.reset_map = map::kNone;
    m.pending = local_mapping::PendingOptimization{};  // a result still on its way is dropped (seq)
}

const map::KeyFrame* currentKeyFrame(const map::Atlas& a, const local_mapping::LocalMapper& m) {
    const map::KeyFrame* kf = a.goodKeyFrame(m.current);
    return kf && kf->map != map::kNone ? kf : nullptr;
}

const map::Map* currentMap(const map::Atlas& a, const local_mapping::LocalMapper& m) {
    const map::KeyFrame* kf = currentKeyFrame(a, m);
    if (!kf) return nullptr;
    return a.findMap(kf->map);
}

}  // namespace orb_local_mapping
