#include "algorithms/orb_local_mapping/keyframe_culling/culling.hpp"

#include <memory>

#include "types/map/atlas_ops.hpp"

namespace orb_local_mapping {

namespace {

constexpr int kNd = 21;    // the optimizable window
constexpr int kThObs = 3;  // other keyframes that must see a point

// The point's finest level in keyframe k (left or right view)
int levelIn(const map::KeyFrame& k, const map::Observation& o) {
    const orb::FrameFeatures& f = *k.data->features;
    int level = -1;
    if (o.left != -1) level = f.keys(0)[std::size_t(o.left)].octave;
    if (o.right != -1) {
        const int r = f.keys(1)[std::size_t(o.right - f.n_left)].octave;
        level = (level == -1 || level > r) ? r : level;
    }
    return level;
}

// More than redundant_th of kf's close points seen by more than kThObs others at the same or a finer scale
bool redundant(const map::Atlas& a, const map::KeyFrame& kf, float redundant_th) {
    const orb::FrameFeatures& f = *kf.data->features;
    int n_redundant = 0, n_points = 0;
    for (std::size_t i = 0; i < kf.points.size(); ++i) {
        const map::MapPoint* p = a.point(kf.points[i]);
        if (!p || p->bad) continue;
        if (int(i) >= f.n_left) continue;  // no depth (culling.hpp)
        const float z = f.depth(int(i));
        if (z > kf.data->th_depth || z < 0) continue;
        ++n_points;
        if (p->n_obs <= kThObs) continue;
        const int level = f.keys(0)[i].octave;
        int n_obs = 0;
        for (const auto& [kid, o] : p->observations) {
            if (kid == kf.id) continue;
            const map::KeyFrame* ki = a.keyframe(kid);
            if (ki && levelIn(*ki, o) <= level + 1 && ++n_obs > kThObs) break;
        }
        if (n_obs > kThObs) ++n_redundant;
    }
    return float(n_redundant) > redundant_th * float(n_points);
}

// Takes k out of the prev/next chain, its preintegration merged into the next one's, then culls it
void unlinkAndCull(map::Atlas& a, map::KeyFrameId k) {
    const map::KeyFrame* kf = a.keyframe(k);
    const map::KeyFrameId prev = kf->prev, next = kf->next;
    const std::shared_ptr<const imu::Preintegrated> pre = kf->preintegrated;
    map::KeyFrame* n = a.editKeyFrame(next);
    if (n->preintegrated && pre) {
        auto merged = std::make_shared<imu::Preintegrated>(*n->preintegrated);
        merged->mergePrevious(*pre);
        n->preintegrated = std::move(merged);
    }
    n->prev = prev;
    a.editKeyFrame(prev)->next = next;
    map::KeyFrame* e = a.editKeyFrame(k);
    e->next = map::kNone;
    e->prev = map::kNone;
    map::setKeyFrameBad(a, k);
}

}  // namespace

int keyFrameCulling(map::Atlas& a, const local_mapping::LocalMapper& m) {
    const map::KeyFrameId cur_id = m.current;
    if (!a.keyframe(cur_id)) return 0;
    map::updateBestCovisibles(a, cur_id);
    const map::KeyFrame* cur = a.keyframe(cur_id);
    const std::vector<map::KeyFrameId> local = cur->ordered_connected;
    const float redundant_th = m.inertial ? 0.5f : 0.9f;

    const bool init_imu = a.imuInitialized();
    const std::size_t kfs_in_map = std::size_t(a.keyFramesInMap());
    const map::Map* kf_map = a.findMap(cur->map);  // mpCurrentKeyFrame->GetMap()
    const bool ba2 = kf_map && kf_map->imu_ba2;

    // The last keyframe of the optimizable window
    map::KeyFrameId last_id = 0;
    if (m.inertial) {
        const map::KeyFrame* k = cur;
        for (int n = 0; n < kNd && k->prev != map::kNone; ++n) k = a.keyframe(k->prev);
        last_id = k->id;
    }

    int count = 0, culled = 0;
    for (map::KeyFrameId id : local) {
        ++count;
        const map::KeyFrame* kf = a.keyframe(id);
        const map::Map* map = kf ? a.findMap(kf->map) : nullptr;
        if (kf && !kf->bad && !(map && id == map->init_kf_id) && redundant(a, *kf, redundant_th)) {
            if (!m.inertial) {
                map::setKeyFrameBad(a, id);
                culled += a.keyframe(id)->bad;
            } else if (kfs_in_map > std::size_t(kNd) && !(id > cur_id - 2) && kf->prev != map::kNone &&
                       kf->next != map::kNone) {
                const map::KeyFrame* p = a.keyframe(kf->prev);
                const double t = a.keyframe(kf->next)->data->timestamp - p->data->timestamp;
                if ((init_imu && id < last_id && t < 3.) || t < 0.5 ||
                    (!ba2 && (kf->imuPosition() - p->imuPosition()).norm() < 0.02f && t < 3)) {
                    unlinkAndCull(a, id);
                    culled += a.keyframe(id)->bad;
                }
            }
        }
        if ((count > 20 && m.abort_ba) || count > 100) break;
    }
    return culled;
}

}  // namespace orb_local_mapping
