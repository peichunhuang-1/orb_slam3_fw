// state_is: mState == value
#include "fw/fw.hpp"
#include "types/tracking/state.hpp"

namespace {

fw::bt::Status state_is(const tracking::State& state, const tracking::State& value) {
    return state == value ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_CONDITION(state_is, INPUTS(("state", tracking::State, latest), ("value", tracking::State, latest)));
