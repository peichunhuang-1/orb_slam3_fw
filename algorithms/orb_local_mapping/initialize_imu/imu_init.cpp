#include "algorithms/orb_local_mapping/initialize_imu/imu_init.hpp"

#include <cmath>
#include <deque>
#include <list>
#include <unordered_map>

#include "types/geometry/so3.hpp"
#include "types/map/atlas_ops.hpp"

namespace orb_local_mapping {

namespace {

local_mapping::KeyFrameScratch& kfScratch(local_mapping::LocalMapper& m, map::KeyFrameId id) {
    if (id >= m.keyframe_scratch.size()) m.keyframe_scratch.resize(std::size_t(id) + 1);
    return m.keyframe_scratch[std::size_t(id)];
}

local_mapping::PointScratch& pointScratch(local_mapping::LocalMapper& m, map::MapPointId id) {
    if (id >= m.point_scratch.size()) m.point_scratch.resize(std::size_t(id) + 1);
    return m.point_scratch[std::size_t(id)];
}

// The keyframes of the map (by id) as problem states, with their rigs; index of each by id
ba::Problem mapKeyFrames(const map::Atlas& a, map::MapId map, std::unordered_map<map::KeyFrameId, int>& index) {
    ba::Problem p;
    p.map = map;
    const map::Map* M = a.findMap(map);
    if (!M) return p;
    std::unordered_map<int, int> rig_index;
    for (const map::KeyFrameId id : M->keyframes) {
        const map::KeyFrame* k = a.keyframe(id);
        if (!k || k->bad) continue;
        const auto [it, added] = rig_index.emplace(k->data->rig, int(p.rigs.size()));
        if (added) p.rigs.push_back(a.rig(k->data->rig));
        ba::KeyFrameState s;
        s.id = id;
        s.timestamp = k->data->timestamp;
        s.rig = it->second;
        s.Tcw = k->pose();
        s.Trl = k->data->Trl;
        s.inertial = k->imu;
        s.T_cb = k->data->T_cb;
        s.velocity = k->velocity();
        s.bias = k->bias;
        s.preintegrated = k->preintegrated;
        index.emplace(id, int(p.keyframes.size()));
        p.keyframes.push_back(std::move(s));
    }
    for (ba::KeyFrameState& s : p.keyframes) {
        const map::KeyFrame* k = a.keyframe(s.id);
        const auto it = index.find(k->prev);
        if (k->prev != map::kNone && it != index.end()) s.prev = it->second;
    }
    return p;
}

}  // namespace

bool imuInitDue(const map::Atlas& a, const local_mapping::LocalMapper& m, std::vector<map::KeyFrameId>& keyframes,
                double& first_ts) {
    constexpr std::size_t kMinKeyFrames = 10;
    constexpr double kMinTime = 1.0;
    if (m.reset == local_mapping::PendingReset::all) return false;
    const map::Map* current = a.findMap(a.current);
    if (!current || current->keyframes.size() < kMinKeyFrames) return false;
    const map::KeyFrame* cur = a.keyframe(m.current);
    if (!cur) return false;

    std::list<map::KeyFrameId> chain;
    const map::KeyFrame* k = cur;
    while (k->prev != map::kNone && a.keyframe(k->prev)) {
        chain.push_front(k->id);
        k = a.keyframe(k->prev);
    }
    chain.push_front(k->id);
    keyframes.assign(chain.begin(), chain.end());
    if (keyframes.size() < kMinKeyFrames) return false;
    first_ts = a.keyframe(keyframes.front())->data->timestamp;
    return cur->data->timestamp - first_ts >= kMinTime;
}

ba::Problem inertialInitProblem(map::Atlas& a, local_mapping::LocalMapper& m,
                                const std::vector<map::KeyFrameId>& keyframes, double first_ts, float prior_g,
                                float prior_a) {
    const map::KeyFrame* cur = a.keyframe(m.current);
    const map::MapId map = cur->map;
    if (!a.findMap(map)->imu_initialized) {
        Eigen::Vector3f dirG = Eigen::Vector3f::Zero();
        for (const map::KeyFrameId id : keyframes) {
            const map::KeyFrame* kf = a.keyframe(id);
            if (!kf->preintegrated || kf->prev == map::kNone) continue;
            const map::KeyFrame* prev = a.keyframe(kf->prev);
            if (!prev) continue;
            dirG -= prev->imuRotation() * kf->preintegrated->updatedDeltaVelocity();
            const Eigen::Vector3f vel = (kf->imuPosition() - prev->imuPosition()) / kf->preintegrated->dT;
            const map::KeyFrameId prev_id = kf->prev;
            a.editKeyFrame(id)->setVelocity(vel);
            a.editKeyFrame(prev_id)->setVelocity(vel);
        }
        dirG = dirG / dirG.norm();
        const Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
        const Eigen::Vector3f v = gI.cross(dirG);
        const float nv = v.norm();
        const float cosg = gI.dot(dirG);
        const float ang = std::acos(cosg);
        const Eigen::Vector3f vzg = v * ang / nv;
        m.Rwg = geometry::exp(vzg).cast<double>();
        m.t_init = float(a.keyframe(m.current)->data->timestamp - first_ts);
    } else {
        m.Rwg = Eigen::Matrix3d::Identity();
        m.bg = cur->bias.g.cast<double>();
        m.ba = cur->bias.a.cast<double>();
    }
    m.scale = 1.0;

    std::unordered_map<map::KeyFrameId, int> index;
    ba::Problem p = mapKeyFrames(a, map, index);
    p.kind = ba::Kind::inertial_init;
    p.main_keyframe = m.current;
    p.iterations = 200;
    p.initial_lambda = prior_g != 0.f ? 1e3 : 0.0;
    p.prior_g = prior_g;
    p.prior_a = prior_a;
    p.Rwg = m.Rwg;
    p.scale = m.scale;
    for (ba::KeyFrameState& s : p.keyframes) s.fixed = true;
    if (!p.keyframes.empty()) {
        p.bg = p.keyframes.front().bias.g.cast<double>();
        p.ba = p.keyframes.front().bias.a.cast<double>();
    }
    return p;
}

bool applyInertialInit(map::Atlas& a, local_mapping::LocalMapper& m, map::MapId map,
                       const std::vector<map::KeyFrameId>& keyframes, const ba::Result& r) {
    if (!a.findMap(map)) return false;  // reset meanwhile
    m.Rwg = r.Rwg;
    m.scale = r.scale;
    m.bg = r.bg;
    m.ba = r.ba;
    for (const ba::KeyFrameState& s : r.keyframes) {
        const map::KeyFrame* kf = a.keyframe(s.id);
        if (!kf || kf->map != map) continue;
        map::KeyFrame* e = a.editKeyFrame(s.id);
        e->setVelocity(s.velocity);
        const bool moved = (e->bias.g - r.bg.cast<float>()).norm() > 0.01f;
        e->setNewBias(s.bias);
        if (moved && e->preintegrated) {
            auto p = std::make_shared<imu::Preintegrated>(*e->preintegrated);
            p->reintegrate();
            e->preintegrated = std::move(p);
        }
    }
    if (r.scale < 1e-1) return false;

    const geometry::SE3f Twg(Eigen::Matrix3f(m.Rwg.cast<float>().transpose()), Eigen::Vector3f::Zero());
    map::applyScaledRotation(a, map, Twg, float(m.scale), true);
    // UpdateFrameIMU(mScale, vpKF[0]'s bias, the current keyframe), then again with scale 1: once, combined
    ++m.imu_update_seq;
    m.imu_scale = float(m.scale);
    if (const map::KeyFrame* k0 = a.keyframe(keyframes.front())) m.imu_bias = k0->bias;
    m.imu_keyframe = m.current;

    map::Map* M = a.editMap(map);
    if (!M->imu_initialized) {
        for (const map::KeyFrameId id : keyframes)
            if (map::KeyFrame* kf = a.editKeyFrame(id)) kf->imu = true;
        M->imu_initialized = true;
        if (map::KeyFrame* kf = a.editKeyFrame(m.current)) kf->imu = true;
    }
    return true;
}

ba::Problem fullInertialProblem(const map::Atlas& a, map::MapId map, float prior_g, float prior_a) {
    std::unordered_map<map::KeyFrameId, int> index;
    ba::Problem p = mapKeyFrames(a, map, index);
    p.kind = ba::Kind::full_inertial;
    p.iterations = 100;
    p.initial_lambda = 1e-5;
    p.shared_bias = prior_a != 0.f;
    p.prior_g = prior_g;
    p.prior_a = prior_a;
    for (const map::MapPointId pid : map::mapPoints(a, map)) {
        const map::MapPoint& mp = *a.point(pid);
        if (mp.bad) continue;
        const int pi = int(p.points.size());
        p.points.push_back({pid, mp.pos, false});
        for (const auto& [kid, idx] : mp.observations) {
            const auto it = index.find(kid);
            if (it == index.end()) continue;  // bad, queued (mnId > maxKFid) or in another map
            const map::KeyFrame& kf = *a.keyframe(kid);
            const orb::FrameFeatures& f = *kf.data->features;
            const int n_right = kf.size() - kf.nLeft();
            for (const int i : {idx.left, idx.right}) {
                if (i == -1) continue;
                // rightIndex < mvKeysRight.size(), on the index before subtracting NLeft
                if (i == idx.right && i >= n_right) continue;
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

void storeFullInertial(local_mapping::LocalMapper& m, map::KeyFrameId gba_id, const ba::Result& r) {
    for (const ba::KeyFrameState& k : r.keyframes) {
        local_mapping::KeyFrameScratch& s = kfScratch(m, k.id);
        s.Tcw_gba = k.Tcw;
        if (k.inertial) {
            s.velocity_gba = k.velocity;
            s.bias_gba = k.bias;
        }
        s.ba_global_for = gba_id;
    }
    for (const ba::PointState& p : r.points) {
        local_mapping::PointScratch& s = pointScratch(m, p.id);
        s.pos_gba = p.pos;
        s.ba_global_for = gba_id;
    }
}

void correctFromOrigins(map::Atlas& a, local_mapping::LocalMapper& m, map::MapId map, map::KeyFrameId gba_id,
                        std::size_t n_keyframes) {
    const map::Map* M = a.findMap(map);
    if (!M) return;
    std::deque<map::KeyFrameId> to_check(M->keyframe_origins.begin(), M->keyframe_origins.end());
    while (!to_check.empty()) {
        const map::KeyFrameId id = to_check.front();
        to_check.pop_front();
        const map::KeyFrame* kf = a.keyframe(id);
        if (!kf) continue;
        const geometry::SE3f Twc = kf->poseInverse();
        const geometry::SE3f Tcw_gba = kfScratch(m, id).Tcw_gba;
        for (const map::KeyFrameId cid : kf->children) {
            const map::KeyFrame* c = a.keyframe(cid);
            if (!c || c->bad) continue;
            local_mapping::KeyFrameScratch& cs = kfScratch(m, cid);
            if (cs.ba_global_for != gba_id) {
                const geometry::SE3f Tchildc = c->pose() * Twc;
                cs.Tcw_gba = Tchildc * Tcw_gba;
                const Eigen::Quaternionf Rcor = cs.Tcw_gba.unitQuaternion().conjugate() * c->pose().unitQuaternion();
                if (c->hasVelocity()) cs.velocity_gba = Rcor * c->velocity();
                cs.bias_gba = c->bias;
                cs.ba_global_for = gba_id;
            }
            to_check.push_back(cid);
        }
        local_mapping::KeyFrameScratch& s = kfScratch(m, id);
        map::KeyFrame* e = a.editKeyFrame(id);
        s.Tcw_before_gba = e->pose();
        e->setPose(s.Tcw_gba);
        if (e->imu) {
            s.velocity_before_gba = e->velocity();
            e->setVelocity(s.velocity_gba);
            e->setNewBias(s.bias_gba);
        }
    }

    for (const map::MapPointId pid : map::mapPoints(a, map)) {
        const map::MapPoint* mp = a.point(pid);
        if (mp->bad) continue;
        const local_mapping::PointScratch& ps = pointScratch(m, pid);
        if (ps.ba_global_for == gba_id) {
            a.editPoint(pid)->pos = ps.pos_gba;
            continue;
        }
        const map::KeyFrame* ref = a.keyframe(mp->ref_kf);
        if (!ref) continue;
        const local_mapping::KeyFrameScratch& rs = kfScratch(m, mp->ref_kf);
        if (rs.ba_global_for != gba_id) continue;
        const Eigen::Vector3f Xc = rs.Tcw_before_gba * mp->pos;
        a.editPoint(pid)->pos = ref->poseInverse() * Xc;
    }

    m.n_kfs = int(n_keyframes);
    ++m.idx_init;
    ++m.tracking_ok_seq;
    m.initializing = false;
    if (map::Map* E = a.editMap(map)) ++E->change_index;
}

}  // namespace orb_local_mapping
