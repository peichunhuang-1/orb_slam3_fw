#include "algorithms/orb_loop_closing/global_ba/global_ba_ops.hpp"

#include <unordered_map>

#include "algorithms/orb_local_mapping/shared/global_ba.hpp"

namespace orb_loop_closing {

ba::Problem globalBAProblem(const map::Atlas& a, map::MapId map) {
    std::unordered_map<map::KeyFrameId, int> index;
    ba::Problem p = orb_local_mapping::mapKeyFrames(a, map, index);
    const map::Map* M = a.findMap(map);
    if (!M) return p;
    if (M->imu_initialized) {
        p.kind = ba::Kind::full_inertial;
        p.iterations = 7;
        p.initial_lambda = 1e-5;
        p.shared_bias = false;
    } else {
        p.kind = ba::Kind::global;
        p.iterations = 10;
        p.initial_lambda = 0;
        for (ba::KeyFrameState& k : p.keyframes) k.fixed = k.id == M->init_kf_id;
    }
    orb_local_mapping::addMapPoints(a, map, index, p);
    return p;
}

bool applyGlobalBA(map::Atlas& a, local_mapping::LocalMapper& m, const local_mapping::PendingGlobalBA& g) {
    const map::Map* M = a.findMap(g.map);
    if (!M || M->bad) return false;
    if (!g.imu_initialized && M->imu_initialized) return false;
    orb_local_mapping::storeGlobalBA(m, g.loop_kf, g.result);
    orb_local_mapping::propagateGlobalBA(a, m, g.map, g.loop_kf);
    map::Map* E = a.editMap(g.map);
    ++E->big_change_index;
    ++E->change_index;
    return true;
}

}  // namespace orb_loop_closing
