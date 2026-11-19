// need_new_keyframe: Tracking::NeedNewKeyFrame and Track()'s condition for inserting it (bOK, or RECENTLY_LOST
// inertial with IMU.InsertKFsWhenLost). SUCCESS: make a keyframe. InterruptBA is sent as mapper_cmd.
//
// As the original for KB8 stereo: mMinFrames = 0, thRefRatio 0.75 (the second camera), c1c for stereo only, c3
// (0.5 s since the last keyframe) for inertial; before the IMU initialization (inertial) only the 0.25 s rule.
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "params/orb_tracking_params.hpp"
#include "types/map/atlas_ops.hpp"
#include "types/tracking/mapper_command.hpp"

namespace {

using tracking::State;
using namespace orb_tracking;

bool needNewKeyFrame(const map::Atlas& a, const tracking::Tracker& t, const tracking::MapperStatus& mapper,
                     fw::Optional<tracking::MapperCommand>& mapper_cmd) {
    const tracking::Frame& f = t.current;
    const map::KeyFrame* last_kf = a.keyframe(t.last_keyframe);
    if (t.inertial && !imuInitialized(a)) return last_kf && f.timestamp - last_kf->timestamp >= 0.25;

    if (mapper.stopped || mapper.stop_requested) return false;

    const int nKFs = keyFramesInMap(a);
    if (f.id < t.last_reloc_frame_id + t.max_frames && nKFs > t.max_frames) return false;

    const int min_obs = nKFs <= 2 ? 2 : 3;
    const map::KeyFrame* ref = a.keyframe(t.reference_kf);
    const int ref_matches = ref ? map::trackedMapPoints(a, *ref, min_obs) : 0;

    const bool idle = mapper.accept_keyframes;

    int non_tracked_close = 0, tracked_close = 0;
    for (int i = 0; i < f.nLeft(); ++i) {
        const float z = f.features->depth(i);
        if (z > 0 && z < f.th_depth) {
            if (f.points[std::size_t(i)] != map::kNone && !f.outlier[std::size_t(i)]) ++tracked_close;
            else ++non_tracked_close;
        }
    }
    const bool insert_close = tracked_close < 100 && non_tracked_close > 70;
    const float th_ref_ratio = 0.75f;  // with a second camera, whatever nKFs

    const int inliers = t.matches_inliers;
    const bool c1a = f.id >= t.last_keyframe_frame_id + t.max_frames;
    const bool c1b = f.id >= t.last_keyframe_frame_id + 0 && idle;  // mMinFrames = 0
    const bool c1c = !t.inertial && (inliers < ref_matches * 0.25 || insert_close);
    const bool c2 = (inliers < ref_matches * th_ref_ratio || insert_close) && inliers > 15;
    const bool c3 = last_kf && t.inertial && f.timestamp - last_kf->timestamp >= 0.5;

    if (!(((c1a || c1b || c1c) && c2) || c3)) return false;
    if (idle || mapper.initializing) return true;
    tracking::MapperCommand cmd;  // InterruptBA
    cmd.interrupt_ba = true;
    mapper_cmd = cmd;
    return mapper.keyframes_in_queue < 3;
}

fw::bt::Status need_new_keyframe(const bool& ok, const State& state, const OrbTrackingParams& params,
                                 fw::Optional<const tracking::MapperStatus&> mapper, fw::Read<map::Atlas> atlas,
                                 fw::Read<tracking::Tracker> tracker, fw::Optional<tracking::MapperCommand>& mapper_cmd) {
    const tracking::Tracker& t = *tracker;
    const bool need = needNewKeyFrame(*atlas, t, mapperOf(mapper), mapper_cmd);
    const bool insert = ok || (params.insert_kfs_when_lost && state == State::RECENTLY_LOST && t.inertial);
    return need && insert ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_ACTION(need_new_keyframe,
                      INPUTS(("ok", bool, latest), ("state", tracking::State, latest),
                             ("params", OrbTrackingParams, latest), ("mapper", tracking::MapperStatus, latest)),
                      RESOURCES(("atlas", map::Atlas, read), ("tracker", tracking::Tracker, read)),
                      OUTPUTS(("mapper_cmd", tracking::MapperCommand)));
