// insert_keyframe: LocalMapping::InsertKeyFrame. The keyframe joins the queue; an optimization that has not
// started yet is skipped (mbAbortBA).
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/keyframe_ref.hpp"

namespace {

fw::bt::Status insert_keyframe(const map::KeyFrameRef& keyframe, fw::Write<local_mapping::LocalMapper> mapper) {
    mapper->queue.push_back(keyframe.keyframe_id);
    mapper->abort_ba = true;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(insert_keyframe, INPUTS(("keyframe", map::KeyFrameRef, latest)),
                      RESOURCES(("local_mapper", local_mapping::LocalMapper, write)));
