#pragma once
// TrackLocalMap's local map (only track_local_map uses it): ORB-SLAM3's Tracking::UpdateLocalKeyFrames,
// UpdateLocalPoints, SearchLocalPoints, with Frame::isInFrustum and ORBmatcher::SearchByProjection(Frame&,
// vector<MapPoint*>, th, bFarPoints, thFarPoints). The per-frame marks (mnTrackReferenceForFrame) are local sets;
// the per-point projection (mbTrackInView, ...) is the tracker's point scratch, as it outlives the frame there.
#include "types/map/atlas.hpp"
#include "types/tracking/mapper_status.hpp"
#include "types/tracking/state.hpp"
#include "types/tracking/tracker.hpp"

namespace orb_tracking {

// UpdateLocalMap: the local keyframes (and the reference keyframe: the one sharing most points) and their points
void updateLocalMap(const map::Atlas& atlas, tracking::Tracker& tracker);

// SearchLocalPoints: the local points not in the frame yet, projected and matched
void searchLocalPoints(const map::Atlas& atlas, tracking::Tracker& tracker, tracking::State state,
                       const tracking::MapperStatus& mapper);

}  // namespace orb_tracking
