#include "algorithms/orb_local_mapping/search_in_neighbors/fuse.hpp"

#include <unordered_set>

#include "types/map/atlas_ops.hpp"
#include "types/orb/hamming.hpp"

namespace orb_local_mapping {

namespace {
constexpr int kThLow = 50;  // ORBmatcher::TH_LOW
}  // namespace

int fuse(map::Atlas& a, map::KeyFrameId kf_id, const std::vector<map::MapPointId>& points, float th) {
    FuseProjections proj;
    return fuse(a, kf_id, points, th, proj);
}

int fuse(map::Atlas& a, map::KeyFrameId kf_id, const std::vector<map::MapPointId>& points, float th,
         FuseProjections& proj) {
    const map::KeyFrame* kf = a.keyframe(kf_id);
    if (!kf) return 0;
    const std::shared_ptr<const map::KeyFrameData> data = kf->data;
    const orb::FrameFeatures& f = *data->features;
    const camera::KannalaBrandt8& cam = a.rig(data->rig).camera(0);
    const geometry::SE3f Tcw = kf->pose();
    const Eigen::Vector3f Ow = kf->cameraCenter();

    if (proj.state.size() != points.size()) {
        proj.state.assign(points.size(), 0);
        proj.uv.resize(points.size());
        proj.level.resize(points.size());
    }
    int fused = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const map::MapPointId pid = points[i];
        if (pid == map::kNone) continue;
        const map::MapPoint* p = a.point(pid);
        if (!p || p->bad || p->inKeyFrame(kf_id)) continue;

        if (proj.state[i] == 0) {
            proj.state[i] = -1;
            const Eigen::Vector3f Pw = p->pos;
            const Eigen::Vector3f Pc = Tcw * Pw;
            if (Pc(2) < 0.0f) continue;  // depth must be positive
            const Eigen::Vector2f uv = cam.project_orb(Pc);
            if (!f.inImage(uv(0), uv(1))) continue;

            // Depth inside the scale pyramid, viewing angle under 60°
            const Eigen::Vector3f PO = Pw - Ow;
            const float dist3D = PO.norm();
            if (dist3D < p->minDistanceInvariance() || dist3D > p->maxDistanceInvariance()) continue;
            if (PO.dot(p->normal) < 0.5 * dist3D) continue;

            proj.state[i] = 1;
            proj.uv[i] = uv;
            proj.level[i] = p->predictScale(dist3D, f.log_scale_factor, f.levels);
        }
        if (proj.state[i] < 0) continue;
        const Eigen::Vector2f uv = proj.uv[i];
        const int level = proj.level[i];
        const float radius = th * f.scale_factors[std::size_t(level)];
        const std::vector<std::size_t> near = f.inArea(0, uv(0), uv(1), radius);
        if (near.empty()) continue;

        int best_dist = 256, best_idx = -1;
        for (std::size_t j : near) {
            const cv::KeyPoint& kp = f.keys(0)[j];
            const int kp_level = kp.octave;
            if (kp_level < level - 1 || kp_level > level) continue;
            const float ex = uv(0) - kp.pt.x, ey = uv(1) - kp.pt.y;
            const float e2 = ex * ex + ey * ey;
            if (e2 * f.inv_level_sigma2[std::size_t(kp_level)] > 5.99) continue;
            const int idx = int(j);
            const int dist = orb::hamming256(p->descriptor.data(), f.descriptor(idx));
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = idx;
            }
        }
        if (best_dist > kThLow) continue;

        // A point already there: the one with more observations stays; otherwise a new observation
        const map::MapPointId in_kf = a.keyframe(kf_id)->points[std::size_t(best_idx)];
        if (in_kf != map::kNone) {
            const map::MapPoint* q = a.point(in_kf);
            if (q && !q->bad) {
                if (q->n_obs > p->n_obs)
                    map::replace(a, pid, in_kf);
                else
                    map::replace(a, in_kf, pid);
            }
        } else {
            map::addObservation(a, pid, kf_id, best_idx);
            a.editKeyFrame(kf_id)->points[std::size_t(best_idx)] = pid;
        }
        ++fused;
    }
    return fused;
}

std::vector<map::KeyFrameId> fuseTargets(const map::Atlas& a, map::KeyFrameId cur_id, bool inertial, bool abort_ba) {
    const map::KeyFrame* cur = a.keyframe(cur_id);
    std::vector<map::KeyFrameId> targets;
    if (!cur) return targets;
    std::unordered_set<map::KeyFrameId> marked;
    auto take = [&](map::KeyFrameId id) {
        const map::KeyFrame* k = a.keyframe(id);
        if (!k || k->bad || marked.count(id)) return false;
        targets.push_back(id);
        marked.insert(id);
        return true;
    };
    for (map::KeyFrameId id : cur->bestCovisibles(10)) take(id);

    // Some covisibles of the covisibles
    for (std::size_t i = 0, n = targets.size(); i < n; ++i) {
        for (map::KeyFrameId id : a.keyframe(targets[i])->bestCovisibles(20))
            if (id != cur_id) take(id);
        if (abort_ba) break;
    }

    // Temporal neighbours
    if (inertial) {
        map::KeyFrameId id = cur->prev;
        while (targets.size() < 20 && id != map::kNone) {
            const map::KeyFrame* k = a.keyframe(id);
            if (!k) break;
            take(id);
            id = k->prev;
        }
    }
    return targets;
}

void searchInNeighbors(map::Atlas& a, const local_mapping::LocalMapper& m) {
    const map::KeyFrameId cur_id = m.current;
    if (!a.keyframe(cur_id)) return;
    const std::vector<map::KeyFrameId> targets = fuseTargets(a, cur_id, m.inertial, m.abort_ba);

    // The current keyframe's points into the targets
    // The second call is ORB-SLAM3's Fuse(pKF, points, true), meant for the right camera: th = 1 (fuse.hpp)
    const std::vector<map::MapPointId> matches = a.keyframe(cur_id)->points;
    FuseProjections proj;
    for (map::KeyFrameId t : targets) {
        proj.state.clear();
        fuse(a, t, matches, 3.0f, proj);
        fuse(a, t, matches, 1.0f, proj);
    }
    if (m.abort_ba) return;

    // The targets' points into the current keyframe
    std::vector<map::MapPointId> candidates;
    std::unordered_set<map::MapPointId> seen;
    for (map::KeyFrameId t : targets)
        for (map::MapPointId pid : a.keyframe(t)->points) {
            if (pid == map::kNone) continue;
            const map::MapPoint* p = a.point(pid);
            if (!p || p->bad || !seen.insert(pid).second) continue;
            candidates.push_back(pid);
        }
    proj.state.clear();
    fuse(a, cur_id, candidates, 3.0f, proj);
    fuse(a, cur_id, candidates, 1.0f, proj);

    // Update the points and the covisibility
    const std::vector<map::MapPointId> after = a.keyframe(cur_id)->points;
    for (map::MapPointId pid : after) {
        if (pid == map::kNone) continue;
        const map::MapPoint* p = a.point(pid);
        if (!p || p->bad) continue;
        map::computeDistinctiveDescriptors(a, pid);
        map::updateNormalAndDepth(a, pid);
    }
    map::updateConnections(a, cur_id);
}

}  // namespace orb_local_mapping
