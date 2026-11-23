// empty_queue: LocalMapping::EmptyQueue (only ProcessNewKeyFrame for each queued keyframe: no triangulation, no
// local BA, not sent to loop closing, as the original). Called by loop closing before CorrectLoop only: the global
// BA's write-back stops local mapping without it (stop_mapping).
#include "algorithms/orb_local_mapping/shared/process_keyframe.hpp"
#include "fw/fw.hpp"

namespace {

fw::bt::Status empty_queue(const map::Vocabulary& vocabulary, fw::Write<map::Atlas> atlas,
                           fw::Write<local_mapping::LocalMapper> mapper) {
    local_mapping::LocalMapper& m = *mapper;
    while (orb_local_mapping::processNewKeyFrame(*atlas, m, vocabulary)) {
    }
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(empty_queue,
                      INPUTS(("vocabulary", map::Vocabulary, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("local_mapper", local_mapping::LocalMapper, write)));
