// is_true: a boolean key as a branch condition (inertial)
#include "fw/fw.hpp"

namespace {

fw::bt::Status is_true(const bool& value) { return value ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE; }

}  // namespace

FW_REGISTER_BT_CONDITION(is_true, INPUTS(("value", bool, latest)));
