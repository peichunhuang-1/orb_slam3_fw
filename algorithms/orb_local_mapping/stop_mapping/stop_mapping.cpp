// stop_mapping: LocalMapping::Stop once a stop was requested and the keyframe in hand is done: stopped (unless
// not_stop), which tracking reads (no new keyframe while stopped). The keyframes still queued wait for the Release
// (loop closing empties the queue first, empty_queue; the global BA's write-back does not).
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"

namespace {

fw::bt::Status stop_mapping(fw::Write<local_mapping::LocalMapper> mapper) {
    if (!mapper->not_stop) mapper->stopped = true;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(stop_mapping, RESOURCES(("local_mapper", local_mapping::LocalMapper, write)));
