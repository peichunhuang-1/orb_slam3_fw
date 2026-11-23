// CorrectLoop: LoopClosing::CorrectLoop for local_mapper.loop, run in local mapping's correction branch once it
// has stopped (algorithms/orb_loop_closing/docs/DESIGN.md, "local mapping 的樹"; correction.hpp).
//   start:   the synchronous part and the essential graph's problem (beginCorrectLoop), sent to ba_solver;
//            RUNNING. The loop is dropped (SUCCESS) when a reset came since it was taken, or its current or
//            matched keyframe went bad.
//   running: the result with the problem's seq written back (endCorrectLoop), then the region's SetErase that
//            the original makes after CorrectLoop; SUCCESS. A reset meanwhile: dropped. Other ticks: RUNNING.
// Either way the loop is no longer pending. The global BA the original then starts (when the IMU is not
// initialized, or the map has fewer than 200 keyframes and is the only one) is phase 3: counted (gba_wanted).
#include <cstdio>

#include "algorithms/orb_loop_closing/correct_loop/correction.hpp"
#include "fw/fw.hpp"
#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"
#include "types/map/atlas_ops.hpp"

namespace {

using fw::bt::Status;

class CorrectLoop {
public:
    Status start(fw::Write<map::Atlas> atlas, fw::Write<local_mapping::LocalMapper> mapper,
                 fw::Optional<ba::Problem>& ba_problem) {
        local_mapping::LocalMapper& m = *mapper;
        if (m.loop.epoch != m.reset_epoch) return drop(*atlas, m, false);
        std::optional<ba::Problem> p = orb_loop_closing::beginCorrectLoop(*atlas, m.loop);
        if (!p) return drop(*atlas, m, true);
        p->seq = m.next_seq++;
        m.pending = local_mapping::PendingOptimization{};
        m.pending.waiting = local_mapping::Waiting::pose_graph;
        m.pending.seq = p->seq;
        m.pending.map = p->map;
        std::fprintf(stderr, "local_mapping: correcting the loop of kf %lu with kf %lu: %s, %zu keyframes, %zu edges\n",
                     (unsigned long)m.loop.keyframe, (unsigned long)m.loop.matched_kf,
                     p->kind == ba::Kind::pose_graph_4dof ? "4DoF" : "6DoF", p->keyframes.size(), p->edges.size());
        ba_problem = std::move(*p);
        return Status::RUNNING;
    }

    Status running(fw::Optional<const ba::Result&> ba_result, fw::Write<map::Atlas> atlas,
                   fw::Write<local_mapping::LocalMapper> mapper) {
        local_mapping::LocalMapper& m = *mapper;
        if (m.loop.epoch != m.reset_epoch) {
            m.pending = local_mapping::PendingOptimization{};
            return drop(*atlas, m, false);
        }
        if (!ba_result.has_value() || ba_result->seq != m.pending.seq) return Status::RUNNING;
        map::Atlas& a = *atlas;
        orb_loop_closing::endCorrectLoop(a, m.loop, *ba_result);
        std::fprintf(stderr, "local_mapping: loop of kf %lu corrected (%d iterations, %.1f ms)\n",
                     (unsigned long)m.loop.keyframe, ba_result->iterations, ba_result->solve_ms);
        ++m.loops_corrected;
        if (const map::Map* M = a.findMap(m.loop.map))
            if (!M->imu_initialized || (M->keyframes.size() < 200 && a.maps.size() == 1)) ++m.gba_wanted;
        m.pending = local_mapping::PendingOptimization{};
        applyAfter(a, m);
        m.loop = local_mapping::PendingLoop{};
        return Status::SUCCESS;
    }

    void halted() {}

private:
    // Not corrected: the region's reset still applies (when the ids still name the same keyframes)
    static Status drop(map::Atlas& a, local_mapping::LocalMapper& m, bool apply) {
        ++m.loops_gone;
        if (apply) applyAfter(a, m);
        m.loop = local_mapping::PendingLoop{};
        return Status::SUCCESS;
    }

    static void applyAfter(map::Atlas& a, local_mapping::LocalMapper& m) {
        for (const loop_closing::EraseFlag& f : m.loop.erase_after) {
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

FW_REGISTER_BT_ACTION_CLASS(CorrectLoop,
    INPUTS(("ba_result", ba::Result, latest)),
    RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)),
    OUTPUTS(("ba_problem", ba::Problem)),
    ON_START(&CorrectLoop::start, USES("atlas", "local_mapper"), EMITS("ba_problem")),
    ON_RUNNING(&CorrectLoop::running, USES("ba_result", "atlas", "local_mapper")),
    ON_HALTED(&CorrectLoop::halted));
