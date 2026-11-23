#include "algorithms/orb_local_mapping/shared/global_ba.hpp"

#include <deque>

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

}  // namespace

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


void addMapPoints(const map::Atlas& a, map::MapId map, const std::unordered_map<map::KeyFrameId, int>& index,
                  ba::Problem& p) {
    for (const map::MapPointId pid : map::mapPoints(a, map)) {
        const map::MapPoint& mp = *a.point(pid);
        if (mp.bad) continue;
        const int pi = int(p.points.size());
        p.points.push_back({pid, mp.pos, false});
        bool seen = false;
        for (const auto& [kid, idx] : mp.observations) {
            const auto it = index.find(kid);
            if (it == index.end()) continue;  // bad, queued (mnId > maxKFid) or in another map
            seen = true;
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
        if (!seen) p.points.pop_back();
    }
}

void storeGlobalBA(local_mapping::LocalMapper& m, map::KeyFrameId gba_id, const ba::Result& r) {
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

void propagateGlobalBA(map::Atlas& a, local_mapping::LocalMapper& m, map::MapId map, map::KeyFrameId gba_id) {
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

}

}  // namespace orb_local_mapping
