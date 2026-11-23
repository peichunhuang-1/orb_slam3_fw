// MergeMaps: LoopClosing::MergeLocal2 for local_mapper.merge (stereo-inertial), run in local mapping's correction
// branch once it has stopped and emptied its queue (algorithms/orb_loop_closing/docs/DESIGN.md, "local mapping 的樹";
// merge.hpp has the steps).
//   start:   a global BA in progress is stopped (mbStopGBA; the original's MergeLocal2 does not relaunch it).
//            Dropped (SUCCESS) when a reset came since it was taken, its current or matched keyframe went bad, or
//            they are in the same map by now. The current map aligned to the merge map; the bias problem sent
//            when it has no BA2 yet (RUNNING), else the maps joined and MergeInertialBA's problem sent (RUNNING),
//            or done with fewer than 10 keyframes in the current map.
//   running: the bias result written back, then the maps joined as above; MergeInertialBA's result written back;
//            SUCCESS. A reset meanwhile: dropped.
// Done or dropped, the region's SetErase that Run makes after MergeLocal2 is applied, and the merge is no longer
// pending. The status is sent at each step (UpdateFrameIMU).
#include <cstdio>

#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "algorithms/orb_loop_closing/merge_maps/merge.hpp"
#include "fw/fw.hpp"
#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"
#include "types/map/atlas_ops.hpp"
#include "types/tracking/mapper_status.hpp"

namespace {

using fw::bt::Status;
using local_mapping::Waiting;

class MergeMaps {
public:
    Status start(fw::Write<map::Atlas> atlas, fw::Write<local_mapping::LocalMapper> mapper,
                 fw::Optional<ba::Problem>& ba_problem, fw::Optional<tracking::MapperStatus>& status) {
        local_mapping::LocalMapper& m = *mapper;
        map::Atlas& a = *atlas;
        if (m.gba.state != local_mapping::PendingGlobalBA::State::none) {
            std::fprintf(stderr, "local_mapping: stopping the global BA of kf %lu\n", (unsigned long)m.gba.loop_kf);
            ++m.gba_stopped;
            m.gba = local_mapping::PendingGlobalBA{};
        }
        if (m.merge.epoch != m.reset_epoch) return drop(a, m, false);
        const map::KeyFrame* cur = a.goodKeyFrame(m.merge.keyframe);
        const map::KeyFrame* matched = a.goodKeyFrame(m.merge.matched_kf);
        if (!cur || !matched || cur->map == matched->map || cur->map != a.current || !a.findMap(matched->map))
            return drop(a, m, true);
        m.merge.last_kf = a.goodKeyFrame(m.current) ? m.current : m.merge.keyframe;  // the tracker's last keyframe

        std::fprintf(stderr, "local_mapping: merging map %lu into map %lu: kf %lu with kf %lu\n",
                     (unsigned long)matched->map, (unsigned long)cur->map, (unsigned long)m.merge.keyframe,
                     (unsigned long)m.merge.matched_kf);
        std::optional<ba::Problem> p = orb_loop_closing::alignMergeInertial(a, m);
        if (p) return send(m, std::move(*p), Waiting::merge_bias, ba_problem, status);
        return join(a, m, ba_problem, status);
    }

    Status running(fw::Optional<const ba::Result&> ba_result, fw::Write<map::Atlas> atlas,
                   fw::Write<local_mapping::LocalMapper> mapper, fw::Optional<ba::Problem>& ba_problem,
                   fw::Optional<tracking::MapperStatus>& status) {
        local_mapping::LocalMapper& m = *mapper;
        map::Atlas& a = *atlas;
        if (m.merge.epoch != m.reset_epoch) {
            m.pending = local_mapping::PendingOptimization{};
            return drop(a, m, false);
        }
        if (!ba_result.has_value() || ba_result->seq != m.pending.seq) return Status::RUNNING;
        if (m.pending.waiting == Waiting::merge_bias) {
            orb_loop_closing::applyMergeBias(a, m, *ba_result);
            std::fprintf(stderr, "local_mapping: merge: biases of map %lu (%d iterations, %.1f ms)\n",
                         (unsigned long)m.merge.current_map, ba_result->iterations, ba_result->solve_ms);
            return join(a, m, ba_problem, status);
        }
        orb_loop_closing::applyMergeInertialBA(a, m.merge.current_map, *ba_result);
        std::fprintf(stderr, "local_mapping: merge: MergeInertialBA %zu keyframes, %zu points (%d iterations, %.1f ms%s)\n",
                     ba_result->keyframes.size(), ba_result->points.size(), ba_result->iterations, ba_result->solve_ms,
                     ba_result->ok ? "" : ", not finite: not written back");
        return done(a, m, status);
    }

    void halted() {}

private:
    static Status send(local_mapping::LocalMapper& m, ba::Problem p, Waiting w, fw::Optional<ba::Problem>& ba_problem,
                       fw::Optional<tracking::MapperStatus>& status) {
        p.seq = m.next_seq++;
        m.pending = local_mapping::PendingOptimization{};
        m.pending.waiting = w;
        m.pending.seq = p.seq;
        m.pending.map = p.map;
        ba_problem = std::move(p);
        status = orb_local_mapping::makeStatus(m);
        return Status::RUNNING;
    }

    static Status join(map::Atlas& a, local_mapping::LocalMapper& m, fw::Optional<ba::Problem>& ba_problem,
                       fw::Optional<tracking::MapperStatus>& status) {
        std::optional<ba::Problem> p = orb_loop_closing::joinMaps(a, m);
        if (p) return send(m, std::move(*p), Waiting::merge_inertial, ba_problem, status);
        std::fprintf(stderr, "local_mapping: merge: %zu keyframes in the current map, no MergeInertialBA\n",
                     m.merge.n_new);
        return done(a, m, status);
    }

    static Status done(map::Atlas& a, local_mapping::LocalMapper& m, fw::Optional<tracking::MapperStatus>& status) {
        ++m.merges_done;
        std::fprintf(stderr, "local_mapping: map %lu merged into map %lu\n", (unsigned long)m.merge.merge_map,
                     (unsigned long)m.merge.current_map);
        m.pending = local_mapping::PendingOptimization{};
        applyAfter(a, m);
        m.merge = local_mapping::PendingMerge{};
        status = orb_local_mapping::makeStatus(m);
        return Status::SUCCESS;
    }

    // Not merged: the region's reset still applies (when the ids still name the same keyframes)
    static Status drop(map::Atlas& a, local_mapping::LocalMapper& m, bool apply) {
        ++m.merges_gone;
        if (apply) applyAfter(a, m);
        m.merge = local_mapping::PendingMerge{};
        return Status::SUCCESS;
    }

    static void applyAfter(map::Atlas& a, local_mapping::LocalMapper& m) {
        for (const loop_closing::EraseFlag& f : m.merge.erase_after) {
            if (!a.goodKeyFrame(f.keyframe)) {
                ++m.loop_skipped_bad;
                continue;
            }
            if (f.not_erase)
                map::setNotErase(a, f.keyframe);
            else
                map::setErase(a, f.keyframe);
        }
    }
};

}  // namespace

FW_REGISTER_BT_ACTION_CLASS(MergeMaps,
    INPUTS(("ba_result", ba::Result, latest)),
    RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)),
    OUTPUTS(("ba_problem", ba::Problem), ("mapper", tracking::MapperStatus)),
    ON_START(&MergeMaps::start, USES("atlas", "local_mapper"), EMITS("ba_problem", "mapper")),
    ON_RUNNING(&MergeMaps::running, USES("ba_result", "atlas", "local_mapper"), EMITS("ba_problem", "mapper")),
    ON_HALTED(&MergeMaps::halted));
