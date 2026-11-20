#pragma once
// Local mapping's bookkeeping shared by several leaves: the status tracking reads, ResetIfRequested, the map
// queries. See algorithms/orb_local_mapping/docs/DESIGN.md.
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"
#include "types/tracking/mapper_status.hpp"

namespace orb_local_mapping {

// What tracking reads (AcceptKeyFrames, KeyframesInQueue, IsInitializing, ..., the IMU initialization's results)
tracking::MapperStatus makeStatus(const local_mapping::LocalMapper& m);

// LocalMapping::ResetIfRequested: the queue and the recent points are dropped, the inertial state restarts
void resetIfRequested(local_mapping::LocalMapper& m);

// The current keyframe if it exists, is not bad and is still in a map
const map::KeyFrame* currentKeyFrame(const map::Atlas& a, const local_mapping::LocalMapper& m);

// The map of the current keyframe (mpCurrentKeyFrame->GetMap()), nullptr if none
const map::Map* currentMap(const map::Atlas& a, const local_mapping::LocalMapper& m);

}  // namespace orb_local_mapping
