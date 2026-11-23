// apply_global_ba: the global BA's write-back in local mapping's correction branch, once stopped (the second half
// of RunGlobalBundleAdjustment: global_ba_ops.hpp, applyGlobalBA). Dropped when a reset came since. Nothing ready:
// nothing to do. Always SUCCESS.
#include <chrono>
#include <cstdio>

#include "algorithms/orb_loop_closing/global_ba/global_ba_ops.hpp"
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace {

using State = local_mapping::PendingGlobalBA::State;

fw::bt::Status apply_global_ba(fw::Write<map::Atlas> atlas, fw::Write<local_mapping::LocalMapper> mapper) {
    local_mapping::LocalMapper& m = *mapper;
    if (m.gba.state != State::ready) return fw::bt::Status::SUCCESS;
    const local_mapping::PendingGlobalBA g = std::move(m.gba);
    m.gba = local_mapping::PendingGlobalBA{};
    const auto t0 = std::chrono::steady_clock::now();
    if (g.epoch != m.reset_epoch || !orb_loop_closing::applyGlobalBA(*atlas, m, g)) {
        ++m.gba_dropped;
        std::fprintf(stderr, "local_mapping: global BA for kf %lu dropped\n", (unsigned long)g.loop_kf);
        return fw::bt::Status::SUCCESS;
    }
    ++m.gba_applied;
    const double now = std::chrono::duration<double>(t0.time_since_epoch()).count();
    std::fprintf(stderr, "local_mapping: global BA for kf %lu written back (%.1f ms; %.1f s since it started)\n",
                 (unsigned long)g.loop_kf,
                 std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(),
                 now - g.started);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(apply_global_ba,
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)));
