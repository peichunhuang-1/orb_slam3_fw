// initialize_imu: LocalMapping::InitializeIMU(priorG, priorA, bFIBA), in three stages around two optimizations
// computed outside the lock (docs/DESIGN.md, "鎖外計算"; the steps are in imu_init.hpp):
//   1 (start)   SUCCESS at once when it is not due (a reset requested, < 10 keyframes, < 1 s of them).
//               bInitializing; the queued keyframes are processed (each the current one in turn, appended to the
//               temporal list); the gravity / velocity guess and the InertialOptimization problem are sent. RUNNING.
//   2 (running) InertialOptimization's result written back; scale < 0.1: done (SUCCESS). ApplyScaledRotation,
//               UpdateFrameIMU, bImu, SetImuInitialized; with bFIBA the FullInertialBA problem is sent (RUNNING).
//   3 (running) its result kept; the queued keyframes are processed; the correction from the origin keyframes
//               down the spanning tree; tracking's mState = OK, bInitializing cleared, IncreaseChangeIndex. SUCCESS.
// The original's last loop setting the keyframes still queued bad never finds one here: the queue was just
// emptied in the same tick. Without bFIBA (never in the trees) stage 3 follows stage 2 directly, without the
// correction (the original would set every keyframe to a stale mTcwGBA).
// The status (`mapper`) is sent at each stage, so tracking sees IsInitializing and UpdateFrameIMU at once.
#include "algorithms/orb_local_mapping/initialize_imu/imu_init.hpp"
#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "algorithms/orb_local_mapping/shared/process_keyframe.hpp"
#include "fw/fw.hpp"
#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"
#include "types/map/vocabulary.hpp"
#include "types/tracking/mapper_status.hpp"

namespace {

using fw::bt::Status;
using local_mapping::Waiting;

class InitializeImu {
public:
    Status start(const float& prior_g, const float& prior_a, const bool& fiba, const map::Vocabulary& vocabulary,
                 fw::Write<map::Atlas> atlas, fw::Write<local_mapping::LocalMapper> mapper,
                 fw::Optional<ba::Problem>& ba_problem, fw::Optional<tracking::MapperStatus>& status) {
        (void)fiba;
        local_mapping::LocalMapper& m = *mapper;
        std::vector<map::KeyFrameId> keyframes;
        double first_ts = 0;
        if (!orb_local_mapping::imuInitDue(*atlas, m, keyframes, first_ts)) return Status::SUCCESS;
        m.initializing = true;
        while (orb_local_mapping::processNewKeyFrame(*atlas, m, vocabulary)) keyframes.push_back(m.current);

        ba::Problem p = orb_local_mapping::inertialInitProblem(*atlas, m, keyframes, first_ts, prior_g, prior_a);
        p.seq = m.next_seq++;
        m.pending = local_mapping::PendingOptimization{};
        m.pending.waiting = Waiting::imu_init_inertial;
        m.pending.seq = p.seq;
        m.pending.map = p.map;
        m.pending.imu_keyframes = std::move(keyframes);
        m.pending.first_ts = first_ts;
        ba_problem = std::move(p);
        status = orb_local_mapping::makeStatus(m);
        return Status::RUNNING;
    }

    Status running(const float& prior_g, const float& prior_a, const bool& fiba,
                   fw::Optional<const ba::Result&> ba_result, const map::Vocabulary& vocabulary,
                   fw::Write<map::Atlas> atlas, fw::Write<local_mapping::LocalMapper> mapper,
                   fw::Optional<ba::Problem>& ba_problem, fw::Optional<tracking::MapperStatus>& status) {
        local_mapping::LocalMapper& m = *mapper;
        if (!ba_result.has_value() || ba_result->seq != m.pending.seq) return Status::RUNNING;
        local_mapping::PendingOptimization& w = m.pending;

        if (w.waiting == Waiting::imu_init_inertial) {
            if (!ba_result->ok || ba_result->kind != ba::Kind::inertial_init ||
                !orb_local_mapping::applyInertialInit(*atlas, m, w.map, w.imu_keyframes, *ba_result))
                return finish(m, status);  // "scale too small"
            if (fiba) {
                ba::Problem p = orb_local_mapping::fullInertialProblem(*atlas, w.map, prior_g, prior_a);
                p.seq = m.next_seq++;
                p.main_keyframe = m.current;
                w.waiting = Waiting::imu_init_full;
                w.seq = p.seq;
                w.gba_id = m.current;
                ba_problem = std::move(p);
                status = orb_local_mapping::makeStatus(m);
                return Status::RUNNING;
            }
        } else if (ba_result->ok && ba_result->kind == ba::Kind::full_inertial) {
            orb_local_mapping::storeFullInertial(m, w.gba_id, *ba_result);
        }

        // Stage 3
        while (orb_local_mapping::processNewKeyFrame(*atlas, m, vocabulary)) w.imu_keyframes.push_back(m.current);
        if (w.waiting == Waiting::imu_init_full) {
            orb_local_mapping::correctFromOrigins(*atlas, m, w.map, w.gba_id, w.imu_keyframes.size());
        } else {
            m.n_kfs = int(w.imu_keyframes.size());
            ++m.idx_init;
            ++m.tracking_ok_seq;
            if (map::Map* M = atlas->editMap(w.map)) ++M->change_index;
        }
        return finish(m, status);
    }

    void halted() {}

private:
    static Status finish(local_mapping::LocalMapper& m, fw::Optional<tracking::MapperStatus>& status) {
        m.pending = local_mapping::PendingOptimization{};
        m.initializing = false;
        status = orb_local_mapping::makeStatus(m);
        return Status::SUCCESS;
    }
};

}  // namespace

FW_REGISTER_BT_ACTION_CLASS(InitializeImu,
    INPUTS(("prior_g", float, latest), ("prior_a", float, latest), ("fiba", bool, latest),
           ("ba_result", ba::Result, latest), ("vocabulary", map::Vocabulary, latest)),
    RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)),
    OUTPUTS(("ba_problem", ba::Problem), ("mapper", tracking::MapperStatus)),
    ON_START(&InitializeImu::start, USES("prior_g", "prior_a", "fiba", "vocabulary", "atlas", "local_mapper"),
             EMITS("ba_problem", "mapper")),
    ON_RUNNING(&InitializeImu::running,
               USES("prior_g", "prior_a", "fiba", "ba_result", "vocabulary", "atlas", "local_mapper"),
               EMITS("ba_problem", "mapper")),
    ON_HALTED(&InitializeImu::halted));
