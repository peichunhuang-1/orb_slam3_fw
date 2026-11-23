// apply_loop_result: loop closing's changes for one keyframe (types/loop_closing/loop_result.hpp), applied by
// local mapping, the atlas's only writer (algorithms/orb_loop_closing/docs/DESIGN.md, "local mapping 的樹").
// A result of an older reset epoch is dropped (its ids may name other keyframes now). SetNotErase / SetErase on a
// keyframe that went bad meanwhile are skipped and counted (DESIGN.md, "時序造成的差異").
// A loop is taken for correction (local_mapper.loop; RequestStop: stop_requested, abort_ba) unless another one is
// pending, the map had a big change since the detection (its poses are stale: the original detects and corrects
// in one thread, so never on a moved map), or its current or matched keyframe went bad; the flags after the
// correction then wait for it. A merge likewise (local_mapper.merge), stereo-inertial only (MergeLocal2; a stereo
// one is only counted). Otherwise all flags apply now.
#include <algorithm>
#include <cstddef>

#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/loop_closing/loop_result.hpp"
#include "types/map/atlas_ops.hpp"

namespace {

void applyEraseFlag(map::Atlas& a, local_mapping::LocalMapper& m, const loop_closing::EraseFlag& f) {
    if (!a.goodKeyFrame(f.keyframe)) {
        ++m.loop_skipped_bad;
        return;
    }
    if (f.not_erase)
        map::setNotErase(a, f.keyframe);
    else
        map::setErase(a, f.keyframe);
}

fw::bt::Status apply_loop_result(const loop_closing::LoopResult& loop_result, fw::Write<map::Atlas> atlas,
                                 fw::Write<local_mapping::LocalMapper> mapper) {
    map::Atlas& a = *atlas;
    local_mapping::LocalMapper& m = *mapper;
    if (loop_result.epoch != m.reset_epoch) {
        ++m.loop_results_dropped;
        return fw::bt::Status::SUCCESS;
    }
    ++m.loop_results;
    const std::vector<loop_closing::EraseFlag>& flags = loop_result.erase_flags;
    std::size_t now = flags.size();
    if (loop_result.kind == loop_closing::LoopResult::Kind::loop) {
        ++m.loops_detected;
        const map::KeyFrame* cur = a.goodKeyFrame(loop_result.keyframe);
        const map::Map* cur_map = cur ? a.findMap(cur->map) : nullptr;
        if (m.loop.pending || m.merge.pending) {
            ++m.loops_busy;
        } else if (!cur_map || !a.goodKeyFrame(loop_result.matched_kf)) {
            ++m.loops_gone;
        } else if (cur_map->big_change_index != loop_result.big_change) {
            ++m.loops_stale;
        } else {
            now = std::min(loop_result.erase_after_correction, flags.size());
            local_mapping::PendingLoop& l = m.loop;
            l = local_mapping::PendingLoop{};
            l.pending = true;
            l.epoch = m.reset_epoch;
            l.keyframe = loop_result.keyframe;
            l.matched_kf = loop_result.matched_kf;
            l.Tcw = loop_result.Tcw;
            l.matched_points = loop_result.matched_points;
            l.region_points = loop_result.region_points;
            l.erase_after.assign(flags.begin() + std::ptrdiff_t(now), flags.end());
            m.stop_requested = true;  // RequestStop
            m.abort_ba = true;
        }
    }
    if (loop_result.kind == loop_closing::LoopResult::Kind::merge) {
        ++m.merges_detected;
        const map::KeyFrame* cur = a.goodKeyFrame(loop_result.keyframe);
        const map::KeyFrame* matched = a.goodKeyFrame(loop_result.matched_kf);
        const map::Map* cur_map = cur ? a.findMap(cur->map) : nullptr;
        if (!m.inertial) {
            // MergeLocal (stereo): not ported
        } else if (m.loop.pending || m.merge.pending) {
            ++m.merges_busy;
        } else if (!cur_map || !matched || !a.findMap(matched->map) || matched->map == cur->map) {
            ++m.merges_gone;
        } else if (cur_map->big_change_index != loop_result.big_change) {
            ++m.merges_stale;
        } else {
            now = std::min(loop_result.erase_after_correction, flags.size());
            local_mapping::PendingMerge& g = m.merge;
            g = local_mapping::PendingMerge{};
            g.pending = true;
            g.epoch = m.reset_epoch;
            g.keyframe = loop_result.keyframe;
            g.matched_kf = loop_result.matched_kf;
            g.T_old_new = loop_result.T_old_new;
            g.erase_after.assign(flags.begin() + std::ptrdiff_t(now), flags.end());
            m.stop_requested = true;  // RequestStop
            m.abort_ba = true;
        }
    }
    for (std::size_t i = 0; i < now; ++i) applyEraseFlag(a, m, flags[i]);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(apply_loop_result, INPUTS(("loop_result", loop_closing::LoopResult, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)));
