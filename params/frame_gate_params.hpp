#pragma once
// Parameters of the frame_gate block (algorithms/frame_gate): the images of [drop_from, drop_to) seconds after
// the first one are dropped. drop_to <= drop_from: none.
#include "fw/fw.hpp"

struct FrameGateParams {
    double drop_from = 0;
    double drop_to = 0;
};
FW_PARAMS(FrameGateParams,
    FIELD(drop_from, DEFAULT(0.0), RANGE(0.0, 1e6), DESC("start of the dropped span, s after the first image")),
    FIELD(drop_to, DEFAULT(0.0), RANGE(0.0, 1e6), DESC("end of the dropped span, s after the first image (<= drop_from: none)")));
