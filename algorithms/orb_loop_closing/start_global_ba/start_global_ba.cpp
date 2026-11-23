// start_global_ba: the global BA a corrected loop made due (CorrectLoop: the original's new
// RunGlobalBundleAdjustment thread), built from the map as it is now and sent to gba_solver; then running. Dropped
// when a reset came since. Nothing due: nothing to do. Always SUCCESS.
#include <chrono>
#include <cstdio>

#include "algorithms/orb_loop_closing/global_ba/global_ba_ops.hpp"
#include "fw/fw.hpp"
#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace {

using State = local_mapping::PendingGlobalBA::State;

fw::bt::Status start_global_ba(fw::Read<map::Atlas> atlas, fw::Write<local_mapping::LocalMapper> mapper,
                               fw::Optional<ba::Problem>& gba_problem) {
    local_mapping::LocalMapper& m = *mapper;
    local_mapping::PendingGlobalBA& g = m.gba;
    if (g.state != State::due) return fw::bt::Status::SUCCESS;
    const map::Map* M = atlas->findMap(g.map);
    if (g.epoch != m.reset_epoch || !M) {
        ++m.gba_dropped;
        g = local_mapping::PendingGlobalBA{};
        return fw::bt::Status::SUCCESS;
    }
    ba::Problem p = orb_loop_closing::globalBAProblem(*atlas, g.map);
    p.seq = m.next_seq++;
    p.main_keyframe = g.loop_kf;
    g.state = State::running;
    g.seq = p.seq;
    g.imu_initialized = M->imu_initialized;
    g.started = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    ++m.gba_started;
    std::fprintf(stderr, "local_mapping: global BA for kf %lu: %s, %zu keyframes, %zu points, %zu observations\n",
                 (unsigned long)g.loop_kf, p.kind == ba::Kind::global ? "visual" : "full inertial",
                 p.keyframes.size(), p.points.size(), p.observations.size());
    gba_problem = std::move(p);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(start_global_ba,
                      RESOURCES(("atlas", map::Atlas, read), ("local_mapper", local_mapping::LocalMapper, write)),
                      OUTPUTS(("gba_problem", ba::Problem)));
