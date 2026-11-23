#pragma once
// The whole-map bundle adjustments' bookkeeping, shared by the IMU initialization's full inertial BA
// (initialize_imu) and loop closing's global BA (algorithms/orb_loop_closing/global_ba):
//
//   mapKeyFrames     every good keyframe of the map (by id) as problem states with their rigs, prev linked
//                    within the problem; `index` maps a keyframe id to its index
//   addMapPoints     every good point of the map seen by one of those keyframes, with its observations by them (a
//                    right observation kept only when its feature index, left ones first, is below the number of
//                    right features: the original's rightIndex < mvKeysRight.size() before subtracting NLeft, so a
//                    point may be left with none: the visual BA keeps it as is, the inertial one leaves it out)
//   storeGlobalBA    the result kept as the original's mTcwGBA, mVwbGBA, mBiasGBA, mPosGBA, mnBAGlobalForKF
//   propagateGlobalBA  the write-back of RunGlobalBundleAdjustment and InitializeIMU: from the map's origin
//                    keyframes down the spanning tree, keyframes the BA did not see (added meanwhile) get their
//                    parent's correction; each keyframe takes its GBA pose (velocity and bias when bImu); points
//                    take their GBA position or follow their reference keyframe's correction (no
//                    UpdateNormalAndDepth, as the original)
#include <unordered_map>

#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace orb_local_mapping {

ba::Problem mapKeyFrames(const map::Atlas& a, map::MapId map, std::unordered_map<map::KeyFrameId, int>& index);

void addMapPoints(const map::Atlas& a, map::MapId map, const std::unordered_map<map::KeyFrameId, int>& index,
                  ba::Problem& p);

void storeGlobalBA(local_mapping::LocalMapper& m, map::KeyFrameId gba_id, const ba::Result& r);

void propagateGlobalBA(map::Atlas& a, local_mapping::LocalMapper& m, map::MapId map, map::KeyFrameId gba_id);

}  // namespace orb_local_mapping
