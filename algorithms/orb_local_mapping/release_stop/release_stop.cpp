// release_stop: LocalMapping::Release once the correction is done: not stopped, no stop requested. The original
// also deletes the keyframes queued meanwhile; tracking makes none while it sees the stop, and the few that its
// one-tick-old status lets through are kept and processed.
#include "fw/fw.hpp"
#include "types/local_mapping/local_mapper.hpp"

namespace {

fw::bt::Status release_stop(fw::Write<local_mapping::LocalMapper> mapper) {
    mapper->stopped = false;
    mapper->stop_requested = false;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(release_stop, RESOURCES(("local_mapper", local_mapping::LocalMapper, write)));
