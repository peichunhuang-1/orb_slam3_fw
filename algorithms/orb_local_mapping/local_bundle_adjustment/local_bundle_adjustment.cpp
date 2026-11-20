// local_bundle_adjustment: the local BA of Run(), computed outside the lock (docs/DESIGN.md, "鎖外計算").
//   start:   skipped (SUCCESS) when mbAbortBA is already set, as the original's return before optimize().
//            Inertial with the IMU initialized: mTinit accumulates (the last two steps moved > 5 cm), and
//            before BA2 too little motion (< 2 cm while mTinit < 10) requests the active map's reset and sets
//            mbBadImu; then LocalInertialBA (TODO: bLarge = tracking's inliers > 100, bRecInit = !BA2): for now
//            LocalBundleAdjustment stands in for it. LocalBundleAdjustment: its window and problem (local_ba.hpp);
//            no fixed keyframe: SUCCESS without optimizing, as the original. The problem is copied out and sent;
//            RUNNING.
//   running: the result with the problem's seq is written back by id (local_ba.hpp: the outlier observations
//            erased, poses, points; what went bad or left the map meanwhile skipped); SUCCESS. Other ticks
//            (keyframes, commands arriving meanwhile): RUNNING.
//   halted:  the result still on its way is dropped (its seq no longer matches).
#include "algorithms/orb_local_mapping/local_bundle_adjustment/local_ba.hpp"
#include "fw/fw.hpp"
#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"
#include "types/tracking/mapper_status.hpp"
#include "types/tracking/status.hpp"

namespace {

using fw::bt::Status;

class LocalBundleAdjustment {
public:
    Status start(const bool& inertial, fw::Optional<const tracking::Status&> tracking, fw::Write<map::Atlas> atlas,
                 fw::Write<local_mapping::LocalMapper> mapper, fw::Optional<ba::Problem>& ba_problem,
                 fw::Optional<tracking::MapperStatus>& status) {
        (void)tracking;
        (void)status;
        local_mapping::LocalMapper& m = *mapper;
        if (m.abort_ba) return Status::SUCCESS;

        const map::KeyFrame* kf = atlas->goodKeyFrame(m.current);
        const map::Map* map = kf ? atlas->findMap(kf->map) : nullptr;
        if (!map) return Status::SUCCESS;
        if (inertial && map->imu_initialized) {
            const map::KeyFrame* prev = atlas->keyframe(kf->prev);
            const map::KeyFrame* prev2 = prev ? atlas->keyframe(prev->prev) : nullptr;
            if (prev && prev2) {
                const float dist = (prev->cameraCenter() - kf->cameraCenter()).norm() +
                                   (prev2->cameraCenter() - prev->cameraCenter()).norm();
                if (dist > 0.05f) m.t_init += float(kf->data->timestamp - prev->data->timestamp);
                if (!map->imu_ba2 && m.t_init < 10.f && dist < 0.02f) {
                    // "Not enough motion for initializing. Reseting..."
                    m.reset = local_mapping::PendingReset::active_map;
                    m.reset_map = map->id;
                    m.bad_imu = true;
                }
            }
            // TODO LocalInertialBA; LocalBundleAdjustment meanwhile
        }

        std::optional<ba::Problem> p = orb_local_mapping::localBAProblem(*atlas, m.current);
        if (!p) return Status::SUCCESS;
        p->seq = m.next_seq++;
        m.pending = local_mapping::PendingOptimization{};
        m.pending.waiting = local_mapping::Waiting::local_ba;
        m.pending.seq = p->seq;
        m.pending.map = p->map;
        ba_problem = std::move(*p);
        return Status::RUNNING;
    }

    Status running(fw::Optional<const ba::Result&> ba_result, fw::Write<map::Atlas> atlas,
                   fw::Write<local_mapping::LocalMapper> mapper) {
        local_mapping::LocalMapper& m = *mapper;
        if (!ba_result.has_value() || ba_result->seq != m.pending.seq) return Status::RUNNING;
        if (ba_result->kind == ba::Kind::local && ba_result->ok)
            orb_local_mapping::applyLocalBA(*atlas, m.pending.map, *ba_result);
        m.pending = local_mapping::PendingOptimization{};
        return Status::SUCCESS;
    }

    void halted() {}
};

}  // namespace

FW_REGISTER_BT_ACTION_CLASS(LocalBundleAdjustment,
    INPUTS(("inertial", bool, latest), ("tracking", tracking::Status, latest), ("ba_result", ba::Result, latest)),
    RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)),
    OUTPUTS(("ba_problem", ba::Problem), ("mapper", tracking::MapperStatus)),
    ON_START(&LocalBundleAdjustment::start, USES("inertial", "tracking", "atlas", "local_mapper"),
             EMITS("ba_problem", "mapper")),
    ON_RUNNING(&LocalBundleAdjustment::running, USES("ba_result", "atlas", "local_mapper")),
    ON_HALTED(&LocalBundleAdjustment::halted));
