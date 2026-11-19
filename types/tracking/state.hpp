#pragma once
// Tracking state (ORB-SLAM3's eTrackingState), on the tree's blackboard key `state`. SYSTEM_NOT_READY and
// OK_KLT are not used by ORB-SLAM3 and left out.
#include "fw/fw.hpp"

namespace tracking {

enum class State { NO_IMAGES_YET, NOT_INITIALIZED, OK, RECENTLY_LOST, LOST };

}  // namespace tracking

FW_ENUM(tracking::State, "tracking/State", NO_IMAGES_YET, NOT_INITIALIZED, OK, RECENTLY_LOST, LOST)
