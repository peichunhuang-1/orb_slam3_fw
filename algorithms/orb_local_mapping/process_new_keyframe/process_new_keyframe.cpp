// process_new_keyframe: LocalMapping::ProcessNewKeyFrame, and SetAcceptKeyFrames(false) for the keyframe's
// processing. FAILURE when every queued keyframe was gone (reset meanwhile): nothing to process.
#include "algorithms/orb_local_mapping/shared/process_keyframe.hpp"
#include "fw/fw.hpp"

namespace {

fw::bt::Status process_new_keyframe(const map::Vocabulary& vocabulary, fw::Write<map::Atlas> atlas,
                                    fw::Write<local_mapping::LocalMapper> mapper) {
    mapper->processing = true;
    return orb_local_mapping::processNewKeyFrame(*atlas, *mapper, vocabulary) ? fw::bt::Status::SUCCESS
                                                                              : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_ACTION(process_new_keyframe,
                      INPUTS(("vocabulary", map::Vocabulary, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)));
