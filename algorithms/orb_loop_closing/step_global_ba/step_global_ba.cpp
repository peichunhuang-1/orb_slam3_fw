// step_global_ba: gba_solver's result for the running global BA (receive_inputs, on a gba_result tick). Partial:
// the next iteration is asked for (a Continue of the same seq). Final: kept, and local mapping asked to stop
// (RequestStop: stop_requested, abort_ba) so apply_global_ba writes it back. A result of another seq (a BA stopped
// by a later loop) or after a reset is dropped; gba_solver then gets no Continue and stops there. Always SUCCESS.
#include <cstdio>

#include "fw/fw.hpp"
#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"

namespace {

using State = local_mapping::PendingGlobalBA::State;

fw::bt::Status step_global_ba(const ba::Result& gba_result, fw::Write<local_mapping::LocalMapper> mapper,
                              fw::Optional<ba::Continue>& gba_continue) {
    local_mapping::LocalMapper& m = *mapper;
    local_mapping::PendingGlobalBA& g = m.gba;
    if (g.state != State::running || gba_result.seq != g.seq) return fw::bt::Status::SUCCESS;
    if (g.epoch != m.reset_epoch) {
        ++m.gba_dropped;
        g = local_mapping::PendingGlobalBA{};
        return fw::bt::Status::SUCCESS;
    }
    if (gba_result.partial) {
        gba_continue = ba::Continue{g.seq};
        return fw::bt::Status::SUCCESS;
    }
    std::fprintf(stderr, "local_mapping: global BA for kf %lu solved (%d iterations, %.0f ms)\n",
                 (unsigned long)g.loop_kf, gba_result.iterations, gba_result.solve_ms);
    g.result = gba_result;
    g.state = State::ready;
    m.stop_requested = true;  // RequestStop
    m.abort_ba = true;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(step_global_ba, INPUTS(("gba_result", ba::Result, latest)),
                      RESOURCES(("local_mapper", local_mapping::LocalMapper, write)),
                      OUTPUTS(("gba_continue", ba::Continue)));
