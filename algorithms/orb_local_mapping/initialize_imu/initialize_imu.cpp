// initialize_imu: LocalMapping::InitializeIMU(priorG, priorA, bFIBA), in three stages around two optimizations
// computed outside the lock (docs/DESIGN.md, "鎖外計算"):
//   1 (start)   returns SUCCESS at once when a reset is requested, the map has < 10 keyframes or < 1 s of them.
//               bInitializing; the queued keyframes are processed; the keyframes in temporal order; not
//               initialized yet: the gravity direction and the velocities (Rwg); the InertialOptimization
//               problem is sent. RUNNING.
//   2 (running) scale < 0.1: done. Velocities and biases written back, ApplyScaledRotation, UpdateFrameIMU (the
//               status' imu_update_seq), bImu, SetImuInitialized; bFIBA: the FullInertialBA problem is sent.
//   3 (running) the queued keyframes are processed; the correction is carried down the spanning tree to the
//               keyframes added meanwhile, the points corrected; the keyframes still queued set bad (as the
//               original); tracking's mState = OK (tracking_ok_seq); IncreaseChangeIndex. SUCCESS.
//   halted:     the stage is dropped, bInitializing cleared.
#include "fw/fw.hpp"
#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"
#include "types/map/vocabulary.hpp"
#include "types/tracking/mapper_status.hpp"

namespace {

using fw::bt::Status;

class InitializeImu {
public:
    Status start(const float& prior_g, const float& prior_a, const bool& fiba, fw::Write<map::Atlas> atlas,
                 fw::Write<local_mapping::LocalMapper> mapper, fw::Read<map::Vocabulary> vocabulary,
                 fw::Optional<ba::Problem>& ba_problem, fw::Optional<tracking::MapperStatus>& status) {
        (void)prior_g;
        (void)prior_a;
        (void)fiba;
        (void)atlas;
        (void)mapper;
        (void)vocabulary;
        (void)ba_problem;
        (void)status;
        return Status::SUCCESS;  // TODO stage 1
    }

    Status running(const float& prior_g, const float& prior_a, const bool& fiba,
                   fw::Optional<const ba::Result&> ba_result, fw::Write<map::Atlas> atlas,
                   fw::Write<local_mapping::LocalMapper> mapper, fw::Read<map::Vocabulary> vocabulary,
                   fw::Optional<ba::Problem>& ba_problem, fw::Optional<tracking::MapperStatus>& status) {
        (void)prior_g;
        (void)prior_a;
        (void)fiba;
        (void)ba_problem;
        (void)status;
        (void)atlas;
        (void)vocabulary;
        local_mapping::LocalMapper& m = *mapper;
        if (!ba_result.has_value() || ba_result->seq != m.pending.seq) return Status::RUNNING;
        // TODO stages 2 and 3
        m.pending = local_mapping::PendingOptimization{};
        m.initializing = false;
        return Status::SUCCESS;
    }

    void halted() {}
};

}  // namespace

FW_REGISTER_BT_ACTION_CLASS(InitializeImu,
    INPUTS(("prior_g", float, latest), ("prior_a", float, latest), ("fiba", bool, latest),
           ("ba_result", ba::Result, latest)),
    RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write),
              ("vocabulary", map::Vocabulary, read)),
    OUTPUTS(("ba_problem", ba::Problem), ("mapper", tracking::MapperStatus)),
    ON_START(&InitializeImu::start, USES("prior_g", "prior_a", "fiba", "atlas", "local_mapper", "vocabulary"),
             EMITS("ba_problem", "mapper")),
    ON_RUNNING(&InitializeImu::running,
               USES("prior_g", "prior_a", "fiba", "ba_result", "atlas", "local_mapper", "vocabulary"),
               EMITS("ba_problem", "mapper")),
    ON_HALTED(&InitializeImu::halted));
