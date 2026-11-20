#include "algorithms/orb_local_mapping/local_bundle_adjustment/local_ba.hpp"

#include <unordered_map>
#include <unordered_set>

#include "types/map/atlas_ops.hpp"

namespace orb_local_mapping {

std::optional<ba::Problem> localBAProblem(const map::Atlas& a, map::KeyFrameId kf) {
    const map::KeyFrame* cur = a.keyframe(kf);
    if (!cur) return std::nullopt;
    const map::MapId m = cur->map;
    const map::Map* M = a.findMap(m);
    if (!M) return std::nullopt;
    auto good = [&](const map::KeyFrame* k) { return k && !k->bad && k->map == m; };

    std::vector<map::KeyFrameId> local{kf};
    std::unordered_set<map::KeyFrameId> marked{kf};  // mnBALocalForKF
    for (const map::KeyFrameId n : cur->ordered_connected) {
        marked.insert(n);
        if (good(a.keyframe(n))) local.push_back(n);
    }

    bool init_local = false;
    std::vector<map::MapPointId> points;
    std::unordered_set<map::MapPointId> point_marked;
    for (const map::KeyFrameId id : local) {
        if (id == M->init_kf_id) init_local = true;
        for (const map::MapPointId pid : a.keyframe(id)->points) {
            if (pid == map::kNone) continue;
            const map::MapPoint* p = a.point(pid);
            if (p && !p->bad && p->map == m && point_marked.insert(pid).second) points.push_back(pid);
        }
    }

    std::vector<map::KeyFrameId> fixed;
    std::unordered_set<map::KeyFrameId> fixed_marked;  // mnBAFixedForKF
    for (const map::MapPointId pid : points)
        for (const auto& o : a.point(pid)->observations) {
            if (marked.count(o.first) || !fixed_marked.insert(o.first).second) continue;
            if (good(a.keyframe(o.first))) fixed.push_back(o.first);
        }
    if (fixed.empty() && !init_local) return std::nullopt;

    ba::Problem p;
    p.kind = ba::Kind::local;
    p.map = m;
    p.main_keyframe = kf;
    p.iterations = 10;
    p.initial_lambda = M->inertial ? 100.0 : 0.0;
    std::unordered_map<int, int> rig_index;
    std::unordered_map<map::KeyFrameId, int> kf_index;
    auto addKeyFrame = [&](map::KeyFrameId id, bool is_fixed) {
        const map::KeyFrame& k = *a.keyframe(id);
        const auto [it, added] = rig_index.emplace(k.data->rig, int(p.rigs.size()));
        if (added) p.rigs.push_back(a.rig(k.data->rig));
        ba::KeyFrameState s;
        s.id = id;
        s.timestamp = k.data->timestamp;
        s.rig = it->second;
        s.fixed = is_fixed;
        s.Tcw = k.pose();
        s.Trl = k.data->Trl;
        kf_index.emplace(id, int(p.keyframes.size()));
        p.keyframes.push_back(std::move(s));
    };
    for (const map::KeyFrameId id : local) addKeyFrame(id, id == M->init_kf_id);
    for (const map::KeyFrameId id : fixed) addKeyFrame(id, true);

    for (const map::MapPointId pid : points) {
        const map::MapPoint& mp = *a.point(pid);
        const int pi = int(p.points.size());
        p.points.push_back({pid, mp.pos, false});
        for (const auto& [kid, idx] : mp.observations) {
            const auto it = kf_index.find(kid);
            if (it == kf_index.end()) continue;  // bad or in another map
            const orb::FrameFeatures& f = *a.keyframe(kid)->data->features;
            for (const int i : {idx.left, idx.right}) {
                if (i == -1) continue;
                const cv::KeyPoint& kp = f.keypoint(i);
                ba::Observation o;
                o.keyframe = it->second;
                o.point = pi;
                o.right = f.isRight(i);
                o.feature = i;
                o.uv = Eigen::Vector2f(kp.pt.x, kp.pt.y);
                o.inv_sigma2 = f.inv_level_sigma2[std::size_t(kp.octave)];
                p.observations.push_back(o);
            }
        }
    }
    return p;
}

void applyLocalBA(map::Atlas& a, map::MapId m, const ba::Result& r) {
    for (const ba::Result::Erase& e : r.outliers) {
        const map::MapPoint* p = a.point(e.point);
        if (!p || p->bad || p->map != m) continue;
        map::eraseMapPointMatch(a, e.keyframe, e.point);
        map::eraseObservation(a, e.point, e.keyframe);
    }
    for (const ba::KeyFrameState& k : r.keyframes) {
        const map::KeyFrame* kf = a.keyframe(k.id);
        if (!kf || kf->bad || kf->map != m) continue;
        a.editKeyFrame(k.id)->setPose(k.Tcw);
    }
    for (const ba::PointState& s : r.points) {
        const map::MapPoint* p = a.point(s.id);
        if (!p || p->map != m) continue;
        a.editPoint(s.id)->pos = s.pos;
        map::updateNormalAndDepth(a, s.id);
    }
    if (map::Map* M = a.editMap(m)) ++M->change_index;
}

}  // namespace orb_local_mapping
