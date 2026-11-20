#include "algorithms/orb_local_mapping/local_bundle_adjustment/local_ba.hpp"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "types/map/atlas_ops.hpp"

namespace orb_local_mapping {

namespace {

// The problem's keyframes, points and observations, from the atlas
struct ProblemBuilder {
    const map::Atlas& a;
    ba::Problem& p;
    std::unordered_map<int, int> rig_index;
    std::unordered_map<map::KeyFrameId, int> kf_index;

    ProblemBuilder(const map::Atlas& atlas, ba::Problem& problem) : a(atlas), p(problem) {}

    void addKeyFrame(map::KeyFrameId id, bool is_fixed) {
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
    }

    // The point and its observations by the problem's keyframes; right_level_quirk: LocalInertialBA's level for
    // a right observation (the left feature's, else 0)
    void addPoint(map::MapPointId pid, bool right_level_quirk) {
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
                int level = kp.octave;
                if (right_level_quirk && o.right) level = idx.left != -1 ? f.keypoint(idx.left).octave : 0;
                o.inv_sigma2 = f.inv_level_sigma2[std::size_t(level)];
                p.observations.push_back(o);
            }
        }
    }
};

}  // namespace

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
    ProblemBuilder b(a, p);
    for (const map::KeyFrameId id : local) b.addKeyFrame(id, id == M->init_kf_id);
    for (const map::KeyFrameId id : fixed) b.addKeyFrame(id, true);
    for (const map::MapPointId pid : points) b.addPoint(pid, false);
    return p;
}

std::optional<ba::Problem> localInertialBAProblem(map::Atlas& a, map::KeyFrameId kf, bool large, bool rec_init) {
    const map::KeyFrame* cur = a.keyframe(kf);
    if (!cur) return std::nullopt;
    const map::MapId m = cur->map;
    const map::Map* M = a.findMap(m);
    if (!M) return std::nullopt;

    const int max_opt = large ? 25 : 10;
    const int Nd = std::min(int(M->keyframes.size()) - 2, max_opt);
    std::vector<map::KeyFrameId> opt{kf};
    std::unordered_set<map::KeyFrameId> local_marked{kf}, fixed_marked;  // mnBALocalForKF, mnBAFixedForKF
    for (int i = 1; i < Nd; ++i) {
        const map::KeyFrameId prev = a.keyframe(opt.back())->prev;
        if (prev == map::kNone || !a.keyframe(prev)) break;
        opt.push_back(prev);
        local_marked.insert(prev);
    }

    std::vector<map::MapPointId> points;
    std::unordered_set<map::MapPointId> point_marked;
    for (const map::KeyFrameId id : opt)
        for (const map::MapPointId pid : a.keyframe(id)->points) {
            if (pid == map::kNone) continue;
            const map::MapPoint* q = a.point(pid);
            if (q && !q->bad && point_marked.insert(pid).second) points.push_back(pid);
        }

    std::vector<map::KeyFrameId> fixed;  // lFixedKeyFrames
    {
        const map::KeyFrameId prev = a.keyframe(opt.back())->prev;
        if (prev != map::kNone && a.keyframe(prev)) {
            fixed.push_back(prev);
            fixed_marked.insert(prev);
        } else {
            local_marked.erase(opt.back());
            fixed_marked.insert(opt.back());
            fixed.push_back(opt.back());
            opt.pop_back();
        }
    }
    constexpr std::size_t kMaxFixed = 200;
    for (const map::MapPointId pid : points) {
        for (const auto& o : a.point(pid)->observations) {
            if (local_marked.count(o.first) || !fixed_marked.insert(o.first).second) continue;
            const map::KeyFrame* k = a.keyframe(o.first);
            if (k && !k->bad) {
                fixed.push_back(o.first);
                break;
            }
        }
        if (fixed.size() >= kMaxFixed) break;
    }

    // SetNewBias(the previous keyframe's bias) on each inertial edge's preintegration
    for (const map::KeyFrameId id : opt) {
        const map::KeyFrame* k = a.keyframe(id);
        const map::KeyFrame* prev = a.keyframe(k->prev);
        if (!prev || !k->imu || !prev->imu || !k->preintegrated) continue;
        auto pre = std::make_shared<imu::Preintegrated>(*k->preintegrated);
        pre->setNewBias(prev->bias);
        a.editKeyFrame(id)->preintegrated = std::move(pre);
    }

    ba::Problem p;
    p.kind = ba::Kind::local_inertial;
    p.map = m;
    p.main_keyframe = kf;
    p.large = large;
    p.rec_init = rec_init;
    p.iterations = large ? 4 : 10;
    p.initial_lambda = large ? 1e-2 : 1.0;
    ProblemBuilder b(a, p);
    auto addInertial = [&](map::KeyFrameId id, bool is_fixed) {
        const map::KeyFrame& k = *a.keyframe(id);
        b.addKeyFrame(id, is_fixed);
        ba::KeyFrameState& s = p.keyframes.back();
        s.inertial = k.imu;
        s.T_cb = k.data->T_cb;
        s.velocity = k.velocity();
        s.bias = k.bias;
        s.preintegrated = k.preintegrated;
    };
    for (const map::KeyFrameId id : opt) addInertial(id, false);
    for (const map::KeyFrameId id : fixed) {
        const map::KeyFrame* k = a.keyframe(id);
        if (k->bad || k->map != m) continue;  // in no edge
        addInertial(id, true);
    }
    for (ba::KeyFrameState& s : p.keyframes) {
        const auto it = b.kf_index.find(a.keyframe(s.id)->prev);
        if (it != b.kf_index.end()) s.prev = it->second;
    }
    const Eigen::Vector3f O = a.keyframe(kf)->cameraCenter();  // (cur may be stale after the edits)
    for (const map::MapPointId pid : points) {
        b.addPoint(pid, true);
        p.points.back().close = (p.points.back().pos - O).norm() < 10.f;
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
        map::KeyFrame* e = a.editKeyFrame(k.id);
        e->setPose(k.Tcw);
        if (k.inertial) {
            e->setVelocity(k.velocity);
            e->setNewBias(k.bias);
        }
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
