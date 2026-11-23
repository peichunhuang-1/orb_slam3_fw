#pragma once
// LocalMapping::InitializeIMU's steps on the atlas, between the two optimizations ba_solver computes
// (initialize_imu.cpp runs them in its three stages and processes the queued keyframes in between):
//
//   imuInitDue           the early returns: a reset requested (mbResetRequested), fewer than 10 keyframes in the
//                        current map, fewer than 10 keyframes back along mPrevKF, or less than 1 s of them.
//                        Fills vpKF (the keyframes in temporal order, the current one last) and mFirstTs.
//   inertialInitProblem  the gravity direction and velocity guess when the map's IMU is not initialized (dirG
//                        from the preintegrated velocities, each keyframe's velocity from its and the previous
//                        one's IMU positions, Rwg, mTinit), else Rwg = I and the current keyframe's bias; then the
//                        InertialOptimization problem on every keyframe of the map (by id: the original's
//                        GetAllKeyFrames is by address, and the bias vertices start at its first keyframe's, so
//                        the port's start at the one with the smallest id)
//   applyInertialInit    InertialOptimization's write-back (velocities; the bias, the preintegration
//                        re-integrated when the gyro bias moved by more than 0.01), then, unless the scale is below
//                        0.1, ApplyScaledRotation, UpdateFrameIMU (the status' imu_update_seq), bImu and
//                        SetImuInitialized. false: the scale was too small (the initialization stops there).
//   fullInertialProblem  FullInertialBA(map, 100, false, nLoopId = the current keyframe, ..., bInit = prior_a != 0)
//                        on the whole map. As the original, a right observation is kept only when its feature index
//                        (left ones first) is below the number of right features.
//   storeFullInertial    its result kept as the original's mTcwGBA, mVwbGBA, mBiasGBA, mPosGBA, mnBAGlobalForKF
//                        (shared/global_ba.hpp, storeGlobalBA)
//   correctFromOrigins   InitializeIMU's last part: the GBA's write-back down the spanning tree
//                        (shared/global_ba.hpp, propagateGlobalBA); mnKFs, mIdxInit, tracking's mState = OK
//                        (tracking_ok_seq), bInitializing cleared, IncreaseChangeIndex.
#include <vector>

#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace orb_local_mapping {

bool imuInitDue(const map::Atlas& a, const local_mapping::LocalMapper& m, std::vector<map::KeyFrameId>& keyframes,
                double& first_ts);

// Changes the keyframes' velocities (first initialization) and m's Rwg, scale, bg, ba, t_init
ba::Problem inertialInitProblem(map::Atlas& a, local_mapping::LocalMapper& m,
                                const std::vector<map::KeyFrameId>& keyframes, double first_ts, float prior_g,
                                float prior_a);

bool applyInertialInit(map::Atlas& a, local_mapping::LocalMapper& m, map::MapId map,
                       const std::vector<map::KeyFrameId>& keyframes, const ba::Result& r);

ba::Problem fullInertialProblem(const map::Atlas& a, map::MapId map, float prior_g, float prior_a);

void storeFullInertial(local_mapping::LocalMapper& m, map::KeyFrameId gba_id, const ba::Result& r);

void correctFromOrigins(map::Atlas& a, local_mapping::LocalMapper& m, map::MapId map, map::KeyFrameId gba_id,
                        std::size_t n_keyframes);

}  // namespace orb_local_mapping
