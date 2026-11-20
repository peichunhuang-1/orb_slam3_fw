// send_to_loop_closing: mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame), as the loop_keyframe output
#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "fw/fw.hpp"
#include "types/map/keyframe_ref.hpp"

namespace {

fw::bt::Status send_to_loop_closing(fw::Read<map::Atlas> atlas, fw::Read<local_mapping::LocalMapper> mapper,
                                    fw::Optional<map::KeyFrameRef>& loop_keyframe) {
    if (const map::KeyFrame* kf = orb_local_mapping::currentKeyFrame(*atlas, *mapper))
        loop_keyframe = map::KeyFrameRef{kf->map, kf->id};
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(send_to_loop_closing,
                      RESOURCES(("atlas", map::Atlas, read), ("local_mapper", local_mapping::LocalMapper, read)),
                      OUTPUTS(("loop_keyframe", map::KeyFrameRef)));
