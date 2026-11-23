// frame_gate (test only): passes images through unchanged, except those of a span of time, so that tracking sees
// a gap in the frames while the IMU goes on. Used by graphs/slam_merge_job.graph.yaml to make tracking start a
// second map and test map merging (algorithms/orb_loop_closing/docs/DESIGN.md, "測試地圖合併：frame_gate").
// Dropping the IMU too would be an IMU gap, an error for imu_preintegration (on_error: stop_box).
//
// The span is relative to the first image this block sees; put one block on each camera, both see the same first
// stamp.
#include <cstdint>
#include <cstdio>

#include "fw/fw.hpp"
#include "fw/ros/msgs/image.hpp"
#include "params/frame_gate_params.hpp"

namespace {

class FrameGate {
public:
    void on_image(Msg<fw::Image> image, const FrameGateParams& params, fw::Optional<Msg<fw::Image>>& out) {
        const std::int64_t t = image.stamp().ns;
        if (!have_first_) {
            first_ = t;
            have_first_ = true;
        }
        const double s = double(t - first_) * 1e-9;
        if (params.drop_to > params.drop_from && s >= params.drop_from && s < params.drop_to) {
            if (dropped_++ == 0) std::fprintf(stderr, "frame_gate: dropping images from %.3f s\n", s);
            return;
        }
        if (dropped_ > 0 && !reported_) {
            std::fprintf(stderr, "frame_gate: %ld images dropped, passing again at %.3f s\n", dropped_, s);
            reported_ = true;
        }
        out = image;
    }

private:
    bool have_first_ = false, reported_ = false;
    std::int64_t first_ = 0;
    long dropped_ = 0;
};

}  // namespace

FW_REGISTER_ALGORITHM_CLASS(FrameGate,
    INPUTS(("params", FrameGateParams, latest)),
    OUTPUTS(("out", fw::Image)),
    ON(("in", fw::Image, trigger), &FrameGate::on_image, USES("params"), EMITS("out")));
