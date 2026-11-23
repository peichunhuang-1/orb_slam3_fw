#pragma once
// Parameters of the loop_closing block (algorithms/orb_loop_closing): connect config://orb_loop_closing.yaml ->
// loop_closing.params.
#include "fw/fw.hpp"

struct OrbLoopClosingParams {
    bool active = true;  // mbActiveLC (loopClosing in the settings): false, no place recognition
    bool debug = false;  // config.debug.loop_closing: the reference's [LC] lines on stderr
};
FW_PARAMS(OrbLoopClosingParams,
    FIELD(active, DEFAULT(true), DESC("place recognition on (ORB-SLAM3's loopClosing setting); off: no loop, no merge")),
    FIELD(debug, DEFAULT(false), DESC("print the [LC] detection trace of the reference (config.debug.loop_closing)")));
