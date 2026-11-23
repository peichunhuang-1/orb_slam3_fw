#include "algorithms/orb_local_mapping/initialize_imu/imu_init.hpp"

#include <cmath>
#include <list>
#include <unordered_map>

#include "algorithms/orb_local_mapping/shared/global_ba.hpp"
#include "types/geometry/so3.hpp"
#include "types/map/atlas_ops.hpp"

namespace orb_local_mapping {

bool imuInitDue(const map::Atlas& a, const local_mapping::LocalMapper& m, std::vector<map::KeyFrameId>& keyframes,
                double& first_ts) {
    constexpr std::size_t kMinKeyFrames = 10;
    constexpr double kMinTime = 1.0;
    if (m.reset == local_mapping::PendingReset::all) return false;
    if (a.keyFramesInMap() < int(kMinKeyFrames)) return false;
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
    addMapPoints(a, map, index, p);
    return p;
}

void storeFullInertial(local_mapping::LocalMapper& m, map::KeyFrameId gba_id, const ba::Result& r) {
    storeGlobalBA(m, gba_id, r);
}

void correctFromOrigins(map::Atlas& a, local_mapping::LocalMapper& m, map::MapId map, map::KeyFrameId gba_id,
                        std::size_t n_keyframes) {
    propagateGlobalBA(a, m, map, gba_id);
    m.n_kfs = int(n_keyframes);
    ++m.idx_init;
    ++m.tracking_ok_seq;
    m.initializing = false;
    if (map::Map* E = a.editMap(map)) ++E->change_index;
}

}  // namespace orb_local_mapping
