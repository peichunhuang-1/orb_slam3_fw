// apply_tracking_edits: tracking's changes to the atlas (types/tracking/atlas_edits.hpp), in the order it made
// them; each message has all those the atlas did not have when tracking last read it, so the ones already applied
// (Atlas::tracking_seq) are skipped. Besides the atlas change:
//   - a keyframe: LocalMapping::InsertKeyFrame, it joins the queue and an optimization that has not started yet
//     is skipped (mbAbortBA);
//   - a reset (RequestReset / RequestResetActiveMap): the keyframes queued before it are dropped now (their
//     atlas is gone), the rest of ResetIfRequested is done after the keyframe being processed, as in Run(), or
//     now when there is none. The original's tracking waits for that before clearing the atlas; here the order
//     of the edits does it.
#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/tracking/atlas_edits.hpp"

namespace {

using Kind = tracking::AtlasEdit::Kind;

fw::bt::Status apply_tracking_edits(const tracking::AtlasEdits& atlas_edits, fw::Write<map::Atlas> atlas,
                                    fw::Write<local_mapping::LocalMapper> mapper) {
    map::Atlas& a = *atlas;
    local_mapping::LocalMapper& m = *mapper;
    for (const auto& e : atlas_edits.edits) {
        if (e->seq <= a.tracking_seq) continue;
        tracking::applyEdit(a, *e);
        if (e->kind == Kind::keyframe) {
            m.queue.push_back(e->keyframe.id);
            m.abort_ba = true;
        } else if (e->kind == Kind::reset_all || e->kind == Kind::reset_active_map) {
            if (e->kind == Kind::reset_all) {
                m.reset = local_mapping::PendingReset::all;
            } else if (m.reset != local_mapping::PendingReset::all) {
                m.reset = local_mapping::PendingReset::active_map;
                m.reset_map = e->map;
            }
            m.queue.clear();
            if (!m.processing) orb_local_mapping::resetIfRequested(m);
        }
    }
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(apply_tracking_edits, INPUTS(("atlas_edits", tracking::AtlasEdits, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)));
