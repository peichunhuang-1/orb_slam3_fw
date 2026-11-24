#include "algorithms/orb_loop_closing/merge_maps/merge.hpp"

#include <algorithm>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "algorithms/orb_local_mapping/local_bundle_adjustment/local_ba.hpp"
#include "algorithms/orb_local_mapping/shared/global_ba.hpp"
#include "algorithms/orb_loop_closing/correct_loop/correction.hpp"
#include "types/map/atlas_ops.hpp"

namespace orb_loop_closing {

using map::KeyFrameId;
using map::MapPointId;

namespace {

// UpdateFrameIMU(s, b, the tracker's last keyframe), through local mapping's status
void updateFrameImu(local_mapping::LocalMapper& m, float s, const imu::Bias& b) {
    ++m.imu_update_seq;
    m.imu_scale = s;
    m.imu_bias = b;
    m.imu_keyframe = m.merge.last_kf;
}

// KeyFrame::GetMapPoints: its good points, by id (the original's set is by address)
std::set<MapPointId> goodPoints(const map::Atlas& a, KeyFrameId id) {
    std::set<MapPointId> s;
    if (const map::KeyFrame* kf = a.keyframe(id))
        for (const MapPointId p : kf->points)
            if (p != map::kNone && a.goodPoint(p)) s.insert(p);
    return s;
}

local_mapping::PointScratch& pointScratch(local_mapping::LocalMapper& m, MapPointId id) {
    if (id >= m.point_scratch.size()) m.point_scratch.resize(std::size_t(id) + 1);
    return m.point_scratch[std::size_t(id)];
}

}  // namespace

std::optional<ba::Problem> alignMergeInertial(map::Atlas& a, local_mapping::LocalMapper& m) {
    local_mapping::PendingMerge& g = m.merge;
    const map::KeyFrame* cur = a.keyframe(g.keyframe);
    g.current_map = cur->map;
    g.merge_map = a.keyframe(g.matched_kf)->map;

    const geometry::SE3f T_on = g.T_old_new.cast<float>();
    map::applyScaledRotation(a, a.current, T_on, 1.f, false);
    updateFrameImu(m, 1.f, a.keyframe(g.keyframe)->bias);

    const map::Map* M = a.findMap(g.current_map);
    g.n_new = M->keyframes.size();
    if (M->imu_ba2) return std::nullopt;

    std::unordered_map<KeyFrameId, int> index;
    ba::Problem p = orb_local_mapping::mapKeyFrames(a, g.current_map, index);
    p.kind = ba::Kind::inertial_init;
    p.main_keyframe = g.keyframe;
    p.iterations = 200;
    p.initial_lambda = 1e3;
    p.prior_g = 1e2f;
    p.prior_a = 1e6f;
    p.fixed_gravity = true;
    p.Rwg = Eigen::Matrix3d::Identity();
    p.scale = 1;
    for (ba::KeyFrameState& s : p.keyframes) s.fixed = true;
    if (!p.keyframes.empty()) {
        p.bg = p.keyframes.front().bias.g.cast<double>();
        p.ba = p.keyframes.front().bias.a.cast<double>();
    }
    return p;
}

void applyMergeBias(map::Atlas& a, local_mapping::LocalMapper& m, const ba::Result& r) {
    imu::Bias b;
    b.g = r.bg.cast<float>();
    b.a = r.ba.cast<float>();
    for (const ba::KeyFrameState& s : r.keyframes) {
        map::KeyFrame* e = a.editKeyFrame(s.id);
        if (!e || e->bad) continue;
        e->setVelocity(s.velocity);
        const bool moved = (e->bias.g - b.g).norm() > 0.01f;
        e->setNewBias(b);
        if (moved && e->preintegrated) {
            auto p = std::make_shared<imu::Preintegrated>(*e->preintegrated);
            p->reintegrate();
            e->preintegrated = std::move(p);
        }
    }
    updateFrameImu(m, 1.f, b);
    if (map::Map* M = a.editMap(m.merge.current_map)) {
        M->imu_ba2 = true;
        M->imu_ba1 = true;
        M->imu_initialized = true;
    }
}

std::optional<ba::Problem> joinMaps(map::Atlas& a, local_mapping::LocalMapper& m, bool original_points) {
    local_mapping::PendingMerge& g = m.merge;
    const map::MapId to = g.current_map, from = g.merge_map;

    // The merge map's keyframes and points into the current map
    if (const map::Map* M = a.findMap(from)) {
        const std::vector<KeyFrameId> kfs(M->keyframes.begin(), M->keyframes.end());
        for (const KeyFrameId k : kfs) {
            const map::KeyFrame* kf = a.keyframe(k);
            if (!kf || kf->bad || kf->map != from) continue;
            map::moveKeyFrame(a, k, to);
        }
        for (const MapPointId p : map::mapPoints(a, from))
            if (!a.point(p)->bad) map::movePoint(a, p, to);
        if (const map::Map* F = a.findMap(from))
            if (map::KeyFrame* o = a.editKeyFrame(F->initial_kf)) o->first_connection = false;
        map::setMapBad(a, from);  // the original's MergeLocal2 leaves it (MergeLocal sets its merged map bad)
    }

    // The spanning tree: the matched keyframe's path to its root reversed, under the current keyframe
    {
        KeyFrameId child = a.keyframe(g.matched_kf)->parent;
        KeyFrameId parent = g.matched_kf;
        map::changeParent(a, g.matched_kf, g.keyframe);
        while (child != map::kNone) {
            map::eraseChild(a, child, parent);
            const KeyFrameId old_parent = a.keyframe(child)->parent;
            map::changeParent(a, child, parent);
            parent = child;
            child = old_parent;
        }
    }

    // The welding area: the matched side's points fused into the current side
    std::vector<KeyFrameId>& merge_kfs = m.merge_connected_kfs;
    merge_kfs.clear();  // the original's member is never cleared: a later merge would fuse the first one's
    merge_kfs.push_back(g.matched_kf);
    {
        const std::vector<KeyFrameId> aux = a.keyframe(g.matched_kf)->ordered_connected;
        merge_kfs.insert(merge_kfs.end(), aux.begin(), aux.end());
        if (merge_kfs.size() > 6) merge_kfs.resize(6);
    }
    map::updateConnections(a, g.keyframe);
    std::vector<KeyFrameId> current_kfs{g.keyframe};
    {
        const std::vector<KeyFrameId> aux = a.keyframe(g.keyframe)->ordered_connected;
        current_kfs.insert(current_kfs.end(), aux.begin(), aux.end());
        if (current_kfs.size() > 6) current_kfs.resize(6);
    }
    std::set<MapPointId> merge_points;
    for (const KeyFrameId k : merge_kfs) {
        const std::set<MapPointId> s = goodPoints(a, k);
        merge_points.insert(s.begin(), s.end());
        if (merge_points.size() > 1000) break;
    }
    const std::vector<MapPointId> fuse(merge_points.begin(), merge_points.end());
    std::vector<MapPointId> rep;
    for (const KeyFrameId k : current_kfs) {
        const map::KeyFrame* kf = a.keyframe(k);
        if (!kf) continue;
        fuseWithPose(a, k, kf->pose(), fuse, 4, rep);
        for (std::size_t i = 0; i < rep.size(); ++i)
            if (rep[i] != map::kNone) map::replace(a, rep[i], fuse[i]);
    }
    for (const KeyFrameId k : current_kfs)
        if (a.goodKeyFrame(k)) map::updateConnections(a, k);
    for (const KeyFrameId k : merge_kfs)
        if (a.goodKeyFrame(k)) map::updateConnections(a, k);

    if (g.n_new < 10) return std::nullopt;
    return mergeInertialBAProblem(a, m, g.last_kf, g.matched_kf, original_points);
}

ba::Problem mergeInertialBAProblem(map::Atlas& a, local_mapping::LocalMapper& m, KeyFrameId last_kf,
                                   KeyFrameId merge_kf, bool original_points) {
    const int Nd = 6;
    const std::size_t maxCovKF = 30;
    const KeyFrameId max_kf = last_kf;
    auto prevOf = [&](KeyFrameId k) { return a.keyframe(k)->prev; };
    auto nextOf = [&](KeyFrameId k) { return a.keyframe(k)->next; };

    // mnBALocalForKF / mnBAFixedForKF of the keyframes: sets for this call
    std::unordered_set<KeyFrameId> local, fixed_marked;
    std::vector<KeyFrameId> opt, cov, fixed;
    auto pushOpt = [&](KeyFrameId k) {
        if (std::find(opt.begin(), opt.end(), k) == opt.end()) opt.push_back(k);  // the original: twice
        local.insert(k);
    };

    // The last keyframe's sliding window
    pushOpt(last_kf);
    for (int i = 1; i < Nd; ++i) {
        const KeyFrameId p = prevOf(opt.back());
        if (p == map::kNone) break;
        pushOpt(p);
    }
    if (const KeyFrameId p = prevOf(opt.back()); p != map::kNone) {
        cov.push_back(p);
        local.insert(p);
    } else {
        cov.push_back(opt.back());
        opt.pop_back();
    }

    // The matched keyframe's temporal neighbours
    pushOpt(merge_kf);
    for (int i = 1; i < Nd / 2; ++i) {
        const KeyFrameId p = prevOf(opt.back());
        if (p == map::kNone) break;
        pushOpt(p);
    }
    if (const KeyFrameId p = prevOf(opt.back()); p != map::kNone) {
        fixed.push_back(p);
        fixed_marked.insert(p);
    } else {
        local.erase(opt.back());
        fixed_marked.insert(opt.back());
        fixed.push_back(opt.back());
        opt.pop_back();
    }
    if (const KeyFrameId n = nextOf(merge_kf); n != map::kNone) pushOpt(n);
    while (opt.size() < std::size_t(2 * Nd)) {
        const KeyFrameId n = nextOf(opt.back());
        if (n == map::kNone) break;
        const std::size_t before = opt.size();
        pushOpt(n);
        if (opt.size() == before) break;  // the walk came back (taken once)
    }

    // Local points, and the observation counts of every good point the optimized keyframes see
    // (original_points: a point the last keyframe's last local BA took counts as taken already)
    std::vector<MapPointId> points;
    std::unordered_map<MapPointId, int> obs_count;
    for (const KeyFrameId k : opt)
        for (const MapPointId pid : a.keyframe(k)->points) {
            if (pid == map::kNone || !a.goodPoint(pid)) continue;
            local_mapping::PointScratch& s = pointScratch(m, pid);
            const bool taken = original_points ? s.ba_local_for == last_kf : obs_count.count(pid) != 0;
            if (!taken) {
                obs_count[pid] = 1;
                points.push_back(pid);
                s.ba_local_for = last_kf;
            } else {
                ++obs_count[pid];
            }
        }

    // Covisible keyframes from the points seen least
    std::vector<std::pair<MapPointId, int>> pairs(obs_count.begin(), obs_count.end());
    std::sort(pairs.begin(), pairs.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
    std::sort(pairs.begin(), pairs.end(), [](const auto& x, const auto& y) { return x.second < y.second; });
    for (std::size_t i = 0; i < pairs.size() && i < maxCovKF; ++i)
        for (const auto& [kid, idx] : a.point(pairs[i].first)->observations) {
            (void)idx;
            if (local.count(kid) || fixed_marked.count(kid)) continue;
            local.insert(kid);
            if (a.goodKeyFrame(kid)) {
                cov.push_back(kid);
                break;
            }
        }

    // SetNewBias(the previous keyframe's bias) on each optimized keyframe's preintegration
    for (const KeyFrameId id : opt) {
        const map::KeyFrame* k = a.keyframe(id);
        const map::KeyFrame* prev = a.keyframe(k->prev);
        if (!prev || !k->imu || !prev->imu || !k->preintegrated) continue;
        auto pre = std::make_shared<imu::Preintegrated>(*k->preintegrated);
        pre->setNewBias(prev->bias);
        a.editKeyFrame(id)->preintegrated = std::move(pre);
    }

    ba::Problem p;
    p.kind = ba::Kind::merge_inertial;
    p.map = a.keyframe(last_kf)->map;
    p.main_keyframe = last_kf;
    p.iterations = 8;
    p.initial_lambda = 1e3;
    std::unordered_map<int, int> rig_index;
    std::unordered_map<KeyFrameId, int> kf_index;
    auto addKeyFrame = [&](KeyFrameId id, bool is_fixed) {
        if (kf_index.count(id)) return;
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
        s.inertial = k.imu;
        s.T_cb = k.data->T_cb;
        s.velocity = k.velocity();
        s.bias = k.bias;
        s.preintegrated = k.preintegrated;
        kf_index.emplace(id, int(p.keyframes.size()));
        p.keyframes.push_back(std::move(s));
    };
    for (const KeyFrameId id : opt) addKeyFrame(id, false);
    for (const KeyFrameId id : cov) addKeyFrame(id, false);
    for (const KeyFrameId id : fixed) addKeyFrame(id, true);
    // The inertial edges: into the optimized keyframes only
    for (const KeyFrameId id : opt) {
        const auto it = kf_index.find(a.keyframe(id)->prev);
        if (it != kf_index.end()) p.keyframes[std::size_t(kf_index.at(id))].prev = it->second;
    }

    for (const MapPointId pid : points) {
        const map::MapPoint& mp = *a.point(pid);
        const int pi = int(p.points.size());
        p.points.push_back({pid, mp.pos, false});
        for (const auto& [kid, idx] : mp.observations) {
            const auto it = kf_index.find(kid);
            if (it == kf_index.end() || kid > max_kf || !a.goodKeyFrame(kid)) continue;
            if (idx.left == -1) continue;  // right only: the original reads mvKeysUn[-1]
            const orb::FrameFeatures& f = *a.keyframe(kid)->data->features;
            const cv::KeyPoint& kp = f.keypoint(idx.left);
            ba::Observation o;
            o.keyframe = it->second;
            o.point = pi;
            o.right = false;
            o.feature = idx.left;
            o.uv = Eigen::Vector2f(kp.pt.x, kp.pt.y);
            o.inv_sigma2 = f.inv_level_sigma2[std::size_t(kp.octave)];
            p.observations.push_back(o);
        }
    }
    return p;
}

void applyMergeInertialBA(map::Atlas& a, map::MapId map, const ba::Result& r) {
    if (r.ok) {
        orb_local_mapping::applyLocalBA(a, map, r);
        return;
    }
    for (const ba::Result::Erase& e : r.outliers) {
        const map::MapPoint* p = a.point(e.point);
        if (!p || p->bad || p->map != map) continue;
        map::eraseMapPointMatch(a, e.keyframe, e.point);
        map::eraseObservation(a, e.point, e.keyframe);
    }
    if (map::Map* M = a.editMap(map)) ++M->change_index;
}

}  // namespace orb_loop_closing
