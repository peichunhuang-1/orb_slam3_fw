#include "algorithms/orb_local_mapping/map_point_culling/culling.hpp"

#include "types/map/atlas_ops.hpp"

namespace orb_local_mapping {

void mapPointCulling(map::Atlas& a, local_mapping::LocalMapper& m) {
    const int current = int(m.current);
    constexpr int kThObs = 3;  // cnThObs (stereo)
    for (auto it = m.recent_points.begin(); it != m.recent_points.end();) {
        const map::MapPoint* p = a.point(*it);
        if (!p || p->bad) {
            it = m.recent_points.erase(it);
        } else if (p->foundRatio() < 0.25f) {
            map::setPointBad(a, *it);
            it = m.recent_points.erase(it);
        } else if (current - int(p->first_kf) >= 2 && p->n_obs <= kThObs) {
            map::setPointBad(a, *it);
            it = m.recent_points.erase(it);
        } else if (current - int(p->first_kf) >= 3) {
            it = m.recent_points.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace orb_local_mapping
