// send_to_loop_closing: mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame), as the loop_keyframe output, with the
// two atlas writes ORB-SLAM3's place recognition makes on the keyframe it takes (local mapping is the atlas's only
// writer: algorithms/orb_loop_closing/docs/DESIGN.md, "send_to_loop_closing 的改變"):
//   - SetNotErase, now rather than a tick later, so keyframe culling cannot take it before loop closing is done;
//   - KeyFrameDatabase::add. The original adds it once its query is done; loop closing leaves the queried
//     keyframe out of its candidates instead, which gives the same ones. Relocalization finds it at once.
// As InsertKeyFrame, the keyframe with id 0 is not sent (and so neither protected nor indexed).
#include "algorithms/orb_local_mapping/shared/mapper_ops.hpp"
#include "fw/fw.hpp"
#include "types/loop_closing/loop_keyframe.hpp"
#include "types/map/atlas_ops.hpp"

namespace {

fw::bt::Status send_to_loop_closing(fw::Write<map::Atlas> atlas, fw::Read<local_mapping::LocalMapper> mapper,
                                    fw::Optional<loop_closing::LoopKeyFrame>& loop_keyframe) {
    map::Atlas& a = *atlas;
    const map::KeyFrame* kf = orb_local_mapping::currentKeyFrame(a, *mapper);
    if (!kf || kf->id == 0) return fw::bt::Status::SUCCESS;
    const loop_closing::LoopKeyFrame out{kf->map, kf->id, mapper->reset_epoch};
    map::setNotErase(a, out.keyframe);
    map::addToBowIndex(a, out.keyframe);
    loop_keyframe = out;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(send_to_loop_closing,
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, read)),
                      OUTPUTS(("loop_keyframe", loop_closing::LoopKeyFrame)));
