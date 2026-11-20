#pragma once
// LocalMapping::MapPointCulling over local_mapper.recent_points, relative to local_mapper.current: the recent
// points that are bad, found in < 25% of the frames that should see them, or seen by too few features (stereo: 3)
// two keyframes after their creation, are erased; those three or more keyframes old leave the list.
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace orb_local_mapping {

void mapPointCulling(map::Atlas& a, local_mapping::LocalMapper& m);

}  // namespace orb_local_mapping
