// local_bundle_adjustment: the local BA of Run(), computed outside the lock (docs/DESIGN.md, "鎖外計算").
//   start:   skipped (SUCCESS) when mbAbortBA is already set, as the original's return before optimize().
//            Inertial with the IMU initialized: mTinit accumulates; too little motion before BA2 requests the
//            active map's reset and sets mbBadImu; the problem is LocalInertialBA's (bLarge: tracking's inliers
//            > 100, bRecInit: !BA2). Otherwise LocalBundleAdjustment's. The problem is copied out and sent;
//            RUNNING.
//   running: the result with the problem's seq is written back by id (poses, velocities, biases, points; the
//            outlier observations erased; what went bad or left the map meanwhile skipped); SUCCESS. Other
//            ticks (keyframes, commands arriving meanwhile): RUNNING.
//   halted:  the result still on its way is dropped (its seq no longer matches).
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

        // TODO the inertial motion check, the local window and the fixed keyframes, the copy of the problem
        const map::Map* map = nullptr;
        if (const map::KeyFrame* kf = atlas->goodKeyFrame(m.current)) map = atlas->findMap(kf->map);
        ba::Problem p;
        p.seq = m.next_seq++;
        p.kind = inertial && map && map->imu_initialized ? ba::Kind::local_inertial : ba::Kind::local;
        p.map = map ? map->id : map::kNone;
        p.main_keyframe = m.current;
        m.pending = local_mapping::PendingOptimization{};
        m.pending.waiting = local_mapping::Waiting::local_ba;
        m.pending.seq = p.seq;
        ba_problem = std::move(p);
        return Status::RUNNING;
    }

    Status running(fw::Optional<const ba::Result&> ba_result, fw::Write<map::Atlas> atlas,
                   fw::Write<local_mapping::LocalMapper> mapper) {
        local_mapping::LocalMapper& m = *mapper;
        if (!ba_result.has_value() || ba_result->seq != m.pending.seq) return Status::RUNNING;
        (void)atlas;  // TODO write back by id
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
