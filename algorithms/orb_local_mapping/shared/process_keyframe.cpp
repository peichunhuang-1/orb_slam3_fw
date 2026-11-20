#include "algorithms/orb_local_mapping/shared/process_keyframe.hpp"

#include "types/map/atlas_ops.hpp"

namespace orb_local_mapping {

bool processNewKeyFrame(map::Atlas& a, local_mapping::LocalMapper& m, const map::Vocabulary& voc) {
    while (!m.queue.empty()) {
        const map::KeyFrameId id = m.queue.front();
        m.queue.pop_front();
        const map::KeyFrame* good = a.goodKeyFrame(id);
        if (!good || good->map == map::kNone) continue;
        map::KeyFrame* kf = a.editKeyFrame(id);
        m.current = id;

        map::computeBoW(*kf, voc);

        for (std::size_t i = 0; i < kf->points.size(); ++i) {
            const map::MapPointId p = kf->points[i];
            const map::MapPoint* mp = a.goodPoint(p);
            if (!mp) continue;
            if (!mp->inKeyFrame(id)) {
                map::addObservation(a, p, id, int(i));
                map::updateNormalAndDepth(a, p);
                map::computeDistinctiveDescriptors(a, p);
            } else {
                // Only the new stereo points tracking made with the keyframe
                m.recent_points.push_back(p);
            }
        }

        map::updateConnections(a, id);
        map::insertKeyFrame(a, id);
        ++m.processed;
        return true;
    }
    return false;
}

}  // namespace orb_local_mapping
