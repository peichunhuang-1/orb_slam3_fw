// apply_pending_reset: System::TrackStereo's reset check before the frame. The reset a leaf asked for in an
// earlier frame (System::Reset / ResetActiveMap only set a flag) is done here: Tracking::Reset or
// Tracking::ResetActiveMap. The atlas part is an edit (atlas_edits): local mapping applies it in order with the
// keyframes and resets itself then (RequestReset / RequestResetActiveMap), so tracking does not wait for it.
#include <utility>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using tracking::State;

// Tracking::Reset
void resetAll(tracking::Tracker& t, State& state) {
    tracking::AtlasEdit e;
    e.kind = tracking::AtlasEdit::Kind::reset_all;  // clearAtlas and the BoW index (KeyFrameDatabase::clear),
    e.map = t.view.next_map_id;                     // keyframe ids from 0; then CreateNewMap
    e.inertial = t.inertial;
    orb_tracking::addEdit(t, std::move(e));
    t.counts.clear();  // their points are gone
    t.change_notified.clear();
    t.initial_frame_id = 0;
    t.next_frame_id = 0;  // Frame::nNextId
    state = State::NO_IMAGES_YET;
    t.relative_poses.clear();
    t.references.clear();
    t.frame_times.clear();
    t.lost.clear();
    t.current = tracking::Frame();
    t.last_reloc_frame_id = 0;
    t.last = tracking::Frame();
    t.reference_kf = map::kNone;
    t.last_keyframe = map::kNone;
    // The points and keyframes are gone (ids start again): so is their scratch
    t.point_scratch.clear();
    t.reloc.clear();
}

// Tracking::ResetActiveMap
void resetActiveMap(tracking::Tracker& t, State& state) {
    tracking::AtlasEdit e;
    e.kind = tracking::AtlasEdit::Kind::reset_active_map;  // KeyFrameDatabase::clearMap, Map::clear
    e.map = t.view.current;
    orb_tracking::addEdit(t, std::move(e));
    const map::Atlas& a = t.view;
    t.last_init_frame_id = t.next_frame_id;
    state = State::NO_IMAGES_YET;

    // The frames of the trajectory from mnInitialFrameId on are marked lost. As the original, the start index is
    // the smallest lowest keyframe id of the maps with keyframes (a keyframe id compared as a frame index).
    long index = t.first_frame_id;
    a.maps.for_each([&index](map::MapId, const map::Map& map) {
        if (map.keyframes.empty() || map.lower_kf == map::kNone) return;
        if (index > long(map.lower_kf)) index = long(map.lower_kf);
    });
    for (std::size_t k = 0; k < t.lost.size(); ++k, ++index)
        if (index >= t.initial_frame_id) t.lost[k] = true;

    t.initial_frame_id = t.current.id;
    t.last_reloc_frame_id = t.current.id;
    t.current = tracking::Frame();
    t.last = tracking::Frame();
    t.reference_kf = map::kNone;
    t.last_keyframe = map::kNone;
    t.has_velocity = false;
}

fw::bt::Status apply_pending_reset(fw::InOut<State>& state, fw::Write<tracking::Tracker> tracker,
                                   fw::Optional<tracking::AtlasEdits>& atlas_edits) {
    tracking::Tracker& t = *tracker;
    if (t.pending_reset == tracking::PendingReset::none) return fw::bt::Status::SUCCESS;
    if (!state.has_value()) state.emplace(State::NO_IMAGES_YET);
    const std::uint64_t seq = t.next_edit_seq;
    if (t.pending_reset == tracking::PendingReset::all) resetAll(t, *state);
    else resetActiveMap(t, *state);
    t.pending_reset = tracking::PendingReset::none;
    orb_tracking::sendEdits(t, seq, atlas_edits);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(apply_pending_reset, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("tracker", tracking::Tracker, write)),
                      OUTPUTS(("atlas_edits", tracking::AtlasEdits)));
