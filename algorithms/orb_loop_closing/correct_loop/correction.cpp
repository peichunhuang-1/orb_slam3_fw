#include "algorithms/orb_loop_closing/correct_loop/correction.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>

#include "types/map/atlas_ops.hpp"
#include "types/orb/hamming.hpp"

namespace orb_loop_closing {

namespace {

using geometry::SE3d;
using geometry::SE3f;
using map::KeyFrameId;
using map::MapPointId;

constexpr int kThLow = 50;    // ORBmatcher::TH_LOW
constexpr int kMinFeat = 100;  // the essential graph's covisibility weight

}  // namespace

int fuseWithPose(map::Atlas& a, KeyFrameId kf_id, const SE3f& Tcw, const std::vector<MapPointId>& points, float th,
                 std::vector<MapPointId>& replace) {
    replace.assign(points.size(), map::kNone);
    const map::KeyFrame* kf = a.keyframe(kf_id);
    if (!kf) return 0;
    const std::shared_ptr<const map::KeyFrameData> data = kf->data;
    const orb::FrameFeatures& f = *data->features;
    const camera::KannalaBrandt8& cam = a.rig(data->rig).camera(0);
    const Eigen::Vector3f Ow = Tcw.inverse().translation();

    int fused = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const MapPointId pid = points[i];
        const map::MapPoint* p = a.point(pid);
        if (!p || p->bad || p->inKeyFrame(kf_id)) continue;

        const Eigen::Vector3f Pw = p->pos;
        const Eigen::Vector3f Pc = Tcw * Pw;
        if (Pc(2) < 0.0f) continue;
        const Eigen::Vector2f uv = cam.project_orb(Pc);
        if (!f.inImage(uv(0), uv(1))) continue;

        const Eigen::Vector3f PO = Pw - Ow;
        const float dist3D = PO.norm();
        if (dist3D < p->minDistanceInvariance() || dist3D > p->maxDistanceInvariance()) continue;
        if (PO.dot(p->normal) < 0.5 * dist3D) continue;

        const int level = p->predictScale(dist3D, f.log_scale_factor, f.levels);
        const float radius = th * f.scale_factors[std::size_t(level)];
        const std::vector<std::size_t> near = f.inArea(0, uv(0), uv(1), radius);
        if (near.empty()) continue;

        int best_dist = 256, best_idx = -1;
        for (std::size_t j : near) {
            const int kp_level = f.keys(0)[j].octave;
            if (kp_level < level - 1 || kp_level > level) continue;
            const int dist = orb::hamming256(p->descriptor.data(), f.descriptor(int(j)));
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = int(j);
            }
        }
        if (best_dist > kThLow) continue;

        const MapPointId in_kf = a.keyframe(kf_id)->points[std::size_t(best_idx)];
        if (in_kf != map::kNone) {
            const map::MapPoint* q = a.point(in_kf);
            if (q && !q->bad) replace[i] = in_kf;
        } else {
            map::addObservation(a, pid, kf_id, best_idx);
            a.editKeyFrame(kf_id)->points[std::size_t(best_idx)] = pid;
        }
        ++fused;
    }
    return fused;
}

namespace {

// The essential graph's edges, as the original adds them (ba/problem.hpp)
class EdgeBuilder {
public:
    EdgeBuilder(const map::Atlas& a, ba::Problem& p, const std::unordered_map<KeyFrameId, int>& index,
                const std::map<KeyFrameId, SE3d>& non_corrected)
        : a_(a), p_(p), index_(index), non_corrected_(non_corrected) {}

    int vertex(KeyFrameId id) const {
        const auto it = index_.find(id);
        return it == index_.end() ? -1 : it->second;
    }
    const SE3d& scw(int v) const { return p_.pose_graph[std::size_t(v)].Tcw; }  // vScw
    // NonCorrectedSim3 when there, else vScw
    const SE3d& before(KeyFrameId id, int v) const {
        const auto it = non_corrected_.find(id);
        return it != non_corrected_.end() ? it->second : scw(v);
    }
    void add(int i, int j, const SE3d& T) { p_.edges.push_back({i, j, T}); }

    void build(KeyFrameId cur, KeyFrameId loop_kf, const std::map<KeyFrameId, std::set<KeyFrameId>>& connections,
               bool four_dof) {
        std::set<std::pair<KeyFrameId, KeyFrameId>> inserted;
        // The new links
        for (const auto& [kf_id, conns] : connections) {
            const int i = vertex(kf_id);
            if (i < 0) continue;
            const map::KeyFrame& kf = *a_.keyframe(kf_id);
            for (KeyFrameId other : conns) {
                if ((kf_id != cur || other != loop_kf) && kf.weight(other) < kMinFeat) continue;
                const int j = vertex(other);
                if (j < 0) continue;
                if (four_dof)
                    add(i, j, scw(i) * scw(j).inverse());  // Sij
                else
                    add(i, j, scw(j) * scw(i).inverse());  // Sji
                inserted.insert({std::min(kf_id, other), std::max(kf_id, other)});
            }
        }

        // The normal edges
        for (std::size_t vi = 0; vi < p_.keyframes.size(); ++vi) {
            const int i = int(vi);
            const KeyFrameId id = p_.keyframes[vi].id;
            const map::KeyFrame& kf = *a_.keyframe(id);
            const SE3d Siw = before(id, i);
            const SE3d Swi = Siw.inverse();
            // Sji (6dof) or Sij (4dof) from the poses before the correction
            auto rel = [&](KeyFrameId other, int j) {
                return four_dof ? Siw * before(other, j).inverse() : before(other, j) * Swi;
            };

            if (!four_dof && kf.parent != map::kNone) {
                const int j = vertex(kf.parent);
                if (j >= 0) add(i, j, rel(kf.parent, j));
            }
            // 4dof: the previous keyframe, whatever bImu (6dof: last, when bImu)
            if (four_dof && kf.prev != map::kNone) {
                const int j = vertex(kf.prev);
                if (j >= 0) add(i, j, rel(kf.prev, j));
            }
            for (KeyFrameId l : kf.loop_edges) {
                if (l >= id) continue;
                const int j = vertex(l);
                if (j >= 0) add(i, j, rel(l, j));
            }
            for (KeyFrameId n : map::covisiblesByWeight(kf, kMinFeat)) {
                if (four_dof) {
                    if (n == kf.prev || n == kf.next || kf.children.count(n) || kf.loop_edges.count(n)) continue;
                } else {
                    if (n == kf.parent || kf.children.count(n)) continue;
                }
                const map::KeyFrame* kn = a_.goodKeyFrame(n);
                if (!kn || n >= id) continue;
                if (inserted.count({std::min(id, n), std::max(id, n)})) continue;
                const int j = vertex(n);
                if (j >= 0) add(i, j, rel(n, j));
            }
            if (!four_dof && kf.imu && kf.prev != map::kNone) {
                const int j = vertex(kf.prev);
                if (j >= 0) add(i, j, rel(kf.prev, j));
            }
        }
    }

private:
    const map::Atlas& a_;
    ba::Problem& p_;
    const std::unordered_map<KeyFrameId, int>& index_;
    const std::map<KeyFrameId, SE3d>& non_corrected_;
};

std::vector<KeyFrameId> connectedKeyFrames(const map::KeyFrame& kf) {  // GetConnectedKeyFrames
    std::vector<KeyFrameId> v;
    for (const auto& c : kf.connected) v.push_back(c.first);
    return v;
}

}  // namespace

std::optional<ba::Problem> beginCorrectLoop(map::Atlas& a, local_mapping::PendingLoop& loop) {
    const KeyFrameId cur_id = loop.keyframe;
    if (!a.goodKeyFrame(cur_id) || !a.goodKeyFrame(loop.matched_kf)) return std::nullopt;
    const map::MapId map_id = a.keyframe(cur_id)->map;
    const map::Map* m = a.findMap(map_id);
    if (!m) return std::nullopt;
    const bool imu_init = m->imu_initialized;
    const bool four_dof = m->inertial && m->imu_initialized;

    map::updateConnections(a, cur_id);

    // The current keyframe and its covisibles get the loop's correction
    std::vector<KeyFrameId> connected = a.keyframe(cur_id)->ordered_connected;
    connected.push_back(cur_id);

    std::map<KeyFrameId, SE3d> corrected, non_corrected;  // CorrectedSim3, NonCorrectedSim3
    {
        const map::KeyFrame& cur = *a.keyframe(cur_id);
        const SE3f Twc = cur.poseInverse();
        corrected[cur_id] = loop.Tcw;
        non_corrected[cur_id] = cur.pose().cast<double>();
        a.editKeyFrame(cur_id)->setPose(loop.Tcw.cast<float>());
        for (KeyFrameId id : connected) {
            if (id == cur_id) continue;
            const map::KeyFrame* k = a.keyframe(id);
            if (!k) continue;
            const SE3f Tiw = k->pose();
            const SE3d Tic = (Tiw * Twc).cast<double>();
            const SE3d Ciw = Tic * loop.Tcw;
            corrected[id] = Ciw;
            a.editKeyFrame(id)->setPose(Ciw.cast<float>());
            non_corrected[id] = Tiw.cast<double>();
        }
    }

    // Their points, from the old pose to the new one
    loop.corrected_reference.clear();
    for (const auto& [id, Ciw] : corrected) {
        const SE3d Cwi = Ciw.inverse();
        const SE3d& Siw = non_corrected[id];
        const std::vector<MapPointId> pts = a.keyframe(id)->points;
        for (MapPointId pid : pts) {
            const map::MapPoint* p = a.point(pid);
            if (!p || p->bad || loop.corrected_reference.count(pid)) continue;
            const Eigen::Vector3d P = p->pos.cast<double>();
            a.editPoint(pid)->pos = (Cwi * (Siw * P)).cast<float>();
            loop.corrected_reference[pid] = id;
            map::updateNormalAndDepth(a, pid);
        }
        if (imu_init) {
            const Eigen::Quaternionf Rcor = (Ciw.unitQuaternion().inverse() * Siw.unitQuaternion()).cast<float>();
            map::KeyFrame* k = a.editKeyFrame(id);
            k->setVelocity(Rcor * k->velocity());
        }
        map::updateConnections(a, id);
    }
    ++a.editMap(map_id)->change_index;

    // Loop fusion: the matched points replace the current keyframe's or are added
    for (std::size_t i = 0; i < loop.matched_points.size(); ++i) {
        const MapPointId lp = loop.matched_points[i];
        if (lp == map::kNone || !a.goodPoint(lp)) continue;
        const MapPointId cp = a.keyframe(cur_id)->points[i];
        if (cp != map::kNone) {
            map::replace(a, cp, lp);
        } else {
            a.editKeyFrame(cur_id)->points[i] = lp;
            map::addObservation(a, lp, cur_id, int(i));
            map::computeDistinctiveDescriptors(a, lp);
        }
    }

    // SearchAndFuse: the loop's region into each corrected keyframe
    std::vector<MapPointId> rep;
    for (const auto& [id, Ciw] : corrected) {
        fuseWithPose(a, id, Ciw.cast<float>(), loop.region_points, 4, rep);
        for (std::size_t i = 0; i < rep.size(); ++i)
            if (rep[i] != map::kNone) map::replace(a, rep[i], loop.region_points[i]);
    }

    // The new links
    std::map<KeyFrameId, std::set<KeyFrameId>> connections;
    for (KeyFrameId id : connected) {
        if (!a.keyframe(id)) continue;
        const std::vector<KeyFrameId> previous = a.keyframe(id)->ordered_connected;
        map::updateConnections(a, id);
        std::set<KeyFrameId>& c = connections[id];
        for (KeyFrameId k : connectedKeyFrames(*a.keyframe(id))) c.insert(k);
        for (KeyFrameId k : previous) c.erase(k);
        for (KeyFrameId k : connected) c.erase(k);
    }

    // The essential graph
    ba::Problem p;
    p.kind = four_dof ? ba::Kind::pose_graph_4dof : ba::Kind::pose_graph;
    p.map = map_id;
    p.main_keyframe = cur_id;
    p.iterations = 20;
    p.initial_lambda = four_dof ? 0 : 1e-16;
    std::unordered_map<KeyFrameId, int> index;
    for (KeyFrameId id : a.findMap(map_id)->keyframes) {
        const map::KeyFrame* k = a.goodKeyFrame(id);
        if (!k) continue;
        ba::KeyFrameState s;
        s.id = id;
        s.fixed = four_dof ? id == loop.matched_kf : id == m->init_kf_id;
        s.Tcw = k->pose();
        if (k->data->has_imu_calib) s.T_cb = k->data->T_cb;
        index[id] = int(p.keyframes.size());
        p.keyframes.push_back(s);
        const auto it = corrected.find(id);
        if (it != corrected.end())
            p.pose_graph.push_back({it->second, true});
        else
            p.pose_graph.push_back({k->pose().cast<double>(), false});
    }
    EdgeBuilder(a, p, index, non_corrected).build(cur_id, loop.matched_kf, connections, four_dof);

    loop.map = map_id;
    loop.four_dof = four_dof;
    loop.vertices.clear();
    loop.vertex_poses.clear();
    for (std::size_t i = 0; i < p.keyframes.size(); ++i) {
        loop.vertices.push_back(p.keyframes[i].id);
        loop.vertex_poses.push_back(p.pose_graph[i].Tcw);
    }
    return p;
}

void endCorrectLoop(map::Atlas& a, const local_mapping::PendingLoop& loop, const ba::Result& r) {
    std::unordered_map<KeyFrameId, std::size_t> index;
    std::vector<SE3d> corrected_wc(loop.vertices.size());
    for (std::size_t i = 0; i < loop.vertices.size() && i < r.poses.size(); ++i) {
        index[loop.vertices[i]] = i;
        corrected_wc[i] = r.poses[i].inverse();
        if (map::KeyFrame* k = a.editKeyFrame(loop.vertices[i])) k->setPose(r.poses[i].cast<float>());
    }

    // Points: from their reference keyframe's pose before to its optimized one
    for (MapPointId pid : map::mapPoints(a, loop.map)) {
        const map::MapPoint* p = a.point(pid);
        if (!p || p->bad) continue;
        KeyFrameId ref = p->ref_kf;
        if (!loop.four_dof) {
            const auto it = loop.corrected_reference.find(pid);
            if (it != loop.corrected_reference.end()) ref = it->second;
        }
        const auto it = index.find(ref);
        if (it != index.end()) {
            const Eigen::Vector3d P = p->pos.cast<double>();
            a.editPoint(pid)->pos = (corrected_wc[it->second] * (loop.vertex_poses[it->second] * P)).cast<float>();
        }
        map::updateNormalAndDepth(a, pid);
    }
    if (map::Map* m = a.editMap(loop.map)) {
        ++m->change_index;
        ++m->big_change_index;  // InformNewBigChange
    }
    map::addLoopEdge(a, loop.matched_kf, loop.keyframe);
    map::addLoopEdge(a, loop.keyframe, loop.matched_kf);
}

}  // namespace orb_loop_closing
