// apply_loop_result: loop closing's changes for one keyframe (types/loop_closing/loop_result.hpp), applied by
// local mapping, the atlas's only writer (algorithms/orb_loop_closing/docs/DESIGN.md, "local mapping 的樹").
// A result of an older reset epoch is dropped (its ids may name other keyframes now). SetNotErase / SetErase on a
// keyframe that went bad meanwhile are skipped and counted (DESIGN.md, "時序造成的差異").
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/loop_closing/loop_result.hpp"
#include "types/map/atlas_ops.hpp"

namespace {

fw::bt::Status apply_loop_result(const loop_closing::LoopResult& loop_result, fw::Write<map::Atlas> atlas,
                                 fw::Write<local_mapping::LocalMapper> mapper) {
    map::Atlas& a = *atlas;
    local_mapping::LocalMapper& m = *mapper;
    if (loop_result.epoch != m.reset_epoch) {
        ++m.loop_results_dropped;
        return fw::bt::Status::SUCCESS;
    }
    ++m.loop_results;
    for (const loop_closing::EraseFlag& f : loop_result.erase_flags) {
        if (!a.goodKeyFrame(f.keyframe)) {
            ++m.loop_skipped_bad;
            continue;
        }
        if (f.not_erase)
            map::setNotErase(a, f.keyframe);
        else
            map::setErase(a, f.keyframe);
    }
    // Phase 1: detected, not corrected yet (CorrectLoop, MergeLocal: phases 2 and 4)
    if (loop_result.kind == loop_closing::LoopResult::Kind::loop) ++m.loops_detected;
    if (loop_result.kind == loop_closing::LoopResult::Kind::merge) ++m.merges_detected;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(apply_loop_result, INPUTS(("loop_result", loop_closing::LoopResult, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)));
