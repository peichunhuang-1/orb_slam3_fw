// tracked_or_recently_lost: bOK || mState == RECENTLY_LOST (motion model, keyframe decision)
#include "fw/fw.hpp"
#include "types/tracking/state.hpp"

namespace {

fw::bt::Status tracked_or_recently_lost(const bool& ok, const tracking::State& state) {
    return ok || state == tracking::State::RECENTLY_LOST ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(tracked_or_recently_lost,
                         INPUTS(("ok", bool, latest), ("state", tracking::State, latest)));
