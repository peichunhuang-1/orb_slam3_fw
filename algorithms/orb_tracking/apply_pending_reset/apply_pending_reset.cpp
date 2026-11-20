// apply_pending_reset: System::TrackStereo's reset check before the frame. The reset a leaf asked for in an
// earlier frame (System::Reset / ResetActiveMap only set a flag) is done here: Tracking::Reset or
// Tracking::ResetActiveMap, and local mapping is told through mapper_cmd (RequestReset / RequestResetActiveMap).
#include <list>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "types/map/atlas_ops.hpp"
#include "types/tracking/mapper_command.hpp"

namespace {

using tracking::State;

// Tracking::Reset
void resetAll(map::Atlas& a, tracking::Tracker& t, State& state, tracking::MapperCommand& cmd) {
    cmd.reset_all = true;
    map::clearAtlas(a);  // and the BoW index (KeyFrameDatabase::clear), keyframe ids from 0
    map::createNewMap(a);
    if (t.inertial) a.editCurrentMap().inertial = true;
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
void resetActiveMap(map::Atlas& a, tracking::Tracker& t, State& state, tracking::MapperCommand& cmd) {
    const map::MapId m = a.current;
    cmd.reset_active_map = true;
    cmd.map_id = m;
    map::eraseMapFromBowIndex(a, m);  // KeyFrameDatabase::clearMap
    map::clearMap(a, m);
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

fw::bt::Status apply_pending_reset(fw::InOut<State>& state, fw::Write<map::Atlas> atlas,
                                   fw::Write<tracking::Tracker> tracker,
                                   fw::Optional<tracking::MapperCommand>& mapper_cmd) {
    tracking::Tracker& t = *tracker;
    if (t.pending_reset == tracking::PendingReset::none) return fw::bt::Status::SUCCESS;
    if (!state.has_value()) state.emplace(State::NO_IMAGES_YET);
    tracking::MapperCommand cmd;
    if (t.pending_reset == tracking::PendingReset::all) resetAll(*atlas, t, *state, cmd);
    else resetActiveMap(*atlas, t, *state, cmd);
    t.pending_reset = tracking::PendingReset::none;
    mapper_cmd = cmd;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(apply_pending_reset, INPUTS(("state", tracking::State, inout)),
                      RESOURCES(("atlas", map::Atlas, write), ("tracker", tracking::Tracker, write)),
                      OUTPUTS(("mapper_cmd", tracking::MapperCommand)));
