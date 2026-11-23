#pragma once
// LoopClosing::RunGlobalBundleAdjustment on the atlas, around the optimization gba_solver runs
// (algorithms/orb_loop_closing/docs/DESIGN.md, "global BA：分段迭代"):
//
//   globalBAProblem  the map's IMU not initialized: Optimizer::GlobalBundleAdjustemnt(map, 10, ..., nLoopKF,
//                    bRobust = false), the `global` kind, the map's init keyframe fixed; else FullInertialBA(map, 7,
//                    bFixLocal = false, nLoopKF), the full_inertial kind with a bias per keyframe (bInit = false),
//                    nothing fixed, λ from 1e-5. Every good keyframe and point of the map
//                    (shared/global_ba.hpp).
//   applyGlobalBA    the write-back once local mapping has stopped: dropped (false) when the map is gone or a
//                    visual BA's map had its IMU initialized meanwhile (the original's !bImuInit &&
//                    isImuInitialized()); else the result is stored as mTcwGBA, ... for nLoopKF and propagated down
//                    the spanning tree (propagateGlobalBA), then InformNewBigChange and IncreaseChangeIndex.
#include "types/ba/problem.hpp"
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"

namespace orb_loop_closing {

ba::Problem globalBAProblem(const map::Atlas& a, map::MapId map);

bool applyGlobalBA(map::Atlas& a, local_mapping::LocalMapper& m, const local_mapping::PendingGlobalBA& g);

}  // namespace orb_loop_closing
