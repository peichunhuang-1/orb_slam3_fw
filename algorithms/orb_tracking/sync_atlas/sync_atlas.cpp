// sync_atlas: the start of each frame. Tracking's copy of the atlas (Tracker::view) becomes the resource's
// current one (O(1): the tiles are shared), with tracking's edits it does not have yet applied again, so every
// leaf of the frame reads one consistent atlas, with tracking's own changes in it. The only tracking leaf that
// reads the resource; none writes it (types/map/docs/DESIGN.md, "tracking 改成只讀").
#include <algorithm>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

fw::bt::Status sync_atlas(fw::Read<map::Atlas> atlas, fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    const std::uint64_t applied = atlas->tracking_seq;
    t.pending.erase(std::remove_if(t.pending.begin(), t.pending.end(),
                                   [applied](const auto& e) { return e->seq <= applied; }),
                    t.pending.end());
    t.view = *atlas;
    for (const auto& e : t.pending) orb_tracking::applyToView(t, *e);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(sync_atlas, RESOURCES(("atlas", map::Atlas, read), ("tracker", tracking::Tracker, write)));
