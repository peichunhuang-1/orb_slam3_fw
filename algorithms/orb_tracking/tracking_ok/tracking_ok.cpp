// tracking_ok: bOK after the local map: mState = OK
#include "fw/fw.hpp"
#include "types/tracking/state.hpp"

namespace {

fw::bt::Status tracking_ok(fw::InOut<tracking::State>& state, bool& ok) {
    if (!state.has_value()) state.emplace(tracking::State::OK);
    *state = tracking::State::OK;
    ok = true;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(tracking_ok, INPUTS(("state", tracking::State, inout)), OUTPUTS(("ok", bool)));
