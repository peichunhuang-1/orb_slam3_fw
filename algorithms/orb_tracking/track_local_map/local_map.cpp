#include "algorithms/orb_tracking/track_local_map/local_map.hpp"

#include <algorithm>
#include <cstdint>
#include <unordered_set>
#include <vector>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "types/orb/hamming.hpp"

namespace orb_tracking {

namespace {

constexpr int kThHigh = 100;  // TH_HIGH

void updateLocalKeyFrames(const map::Atlas& a, tracking::Tracker& t) {
    tracking::Frame& f = t.current;
    // Each point votes for the keyframes observing it (the current frame's points; with the IMU initialized and
    // no recent relocalization the last frame's, as the current one has no matches yet). Bad points leave.
    // (counted as a sorted list of votes: the keyframes in increasing id, as a std::map would give them)
    std::vector<map::KeyFrameId> votes;
    tracking::Frame& voter = (!a.imuInitialized() || f.id < t.last_reloc_frame_id + 2) ? f : t.last;
    for (map::MapPointId& id : voter.points) {
        if (id == map::kNone) continue;
        const map::MapPoint* p = a.point(id);
        if (p && !p->bad) {
            for (const auto& [k, obs] : p->observations) votes.push_back(k);
        } else {
            id = map::kNone;
        }
    }
    std::sort(votes.begin(), votes.end());
    std::vector<std::pair<map::KeyFrameId, int>> counter;
    for (map::KeyFrameId k : votes) {
        if (counter.empty() || counter.back().first != k) counter.push_back({k, 0});
        ++counter.back().second;
    }

    std::unordered_set<map::KeyFrameId> marked;  // mnTrackReferenceForFrame == this frame
    int max = 0;
    map::KeyFrameId kf_max = map::kNone;
    std::vector<map::KeyFrameId>& local = t.local_keyframes;
    local.clear();
    local.reserve(3 * counter.size());
    for (const auto& [k, n] : counter) {
        const map::KeyFrame* kf = a.keyframe(k);
        if (!kf || kf->bad) continue;
        if (n > max) {
            max = n;
            kf_max = k;
        }
        local.push_back(k);
        marked.insert(k);
    }

    // Neighbours of the keyframes so far: the first new of the 10 best covisibles, of the children, the parent.
    // As the original: only the keyframes there before the loop, stopping above 80, and a new parent ends it all.
    const std::size_t n0 = local.size();
    for (std::size_t j = 0; j < n0; ++j) {
        if (local.size() > 80) break;
        const map::KeyFrame& kf = *a.keyframe(local[j]);
        for (map::KeyFrameId n : kf.bestCovisibles(10)) {
            const map::KeyFrame* nk = a.keyframe(n);
            if (nk && !nk->bad && !marked.count(n)) {
                local.push_back(n);
                marked.insert(n);
                break;
            }
        }
        for (map::KeyFrameId c : kf.children) {
            const map::KeyFrame* ck = a.keyframe(c);
            if (ck && !ck->bad && !marked.count(c)) {
                local.push_back(c);
                marked.insert(c);
                break;
            }
        }
        if (kf.parent != map::kNone && a.keyframe(kf.parent) && !marked.count(kf.parent)) {
            local.push_back(kf.parent);
            marked.insert(kf.parent);
            break;
        }
    }

    // The last 20 keyframes in time (inertial). As the original, an already included one stops the walk (the
    // loop goes on without moving).
    if (t.inertial && local.size() < 80) {
        map::KeyFrameId k = f.last_keyframe;
        for (int i = 0; i < 20; ++i) {
            const map::KeyFrame* kf = a.keyframe(k);
            if (!kf) break;
            if (!marked.count(k)) {
                local.push_back(k);
                marked.insert(k);
                k = kf->prev;
            }
        }
    }

    if (kf_max != map::kNone) {
        t.reference_kf = kf_max;
        f.reference_kf = kf_max;
    }
}

void updateLocalPoints(const map::Atlas& a, tracking::Tracker& t) {
    t.local_points.clear();
    std::vector<std::uint8_t> marked(std::size_t(std::max(a.next_point_id, a.next_tracking_point_id)) + 1, 0);  // by id
    for (auto it = t.local_keyframes.rbegin(); it != t.local_keyframes.rend(); ++it) {
        const map::KeyFrame& kf = *a.keyframe(*it);
        for (map::MapPointId id : kf.points) {
            if (id == map::kNone) continue;
            if (std::size_t(id) >= marked.size()) marked.resize(std::size_t(id) + 1, 0);
            if (marked[std::size_t(id)]) continue;
            if (a.goodPoint(id)) {
                t.local_points.push_back(id);
                marked[std::size_t(id)] = 1;
            }
        }
    }
}

// Frame::isInFrustumChecks for one camera (right: through Trl, the right camera's model)
bool inFrustum(const tracking::Frame& f, const camera::StereoRig& rig, const map::MapPoint& p, float cos_limit,
               bool right, tracking::PointScratch& s) {
    const orb::FrameFeatures& ff = *f.features;
    Eigen::Matrix3f R;
    Eigen::Vector3f tr, twc;
    if (right) {
        const Eigen::Matrix3f Rrl = f.Trl.rotationMatrix();
        R = Rrl * f.Rcw();
        tr = Rrl * f.tcw() + f.Trl.translation();
        twc = f.Rwc() * f.Tlr.translation() + f.cameraCenter();
    } else {
        R = f.Rcw();
        tr = f.tcw();
        twc = f.cameraCenter();
    }
    const Eigen::Vector3f Pc = R * p.pos + tr;
    const float Pc_dist = Pc.norm();
    if (Pc(2) < 0.0f) return false;
    const Eigen::Vector2f uv = rig.camera(right ? 1 : 0).project_orb(Pc);
    if (uv(0) < 0 || uv(0) > float(ff.stereo->width) || uv(1) < 0 || uv(1) > float(ff.stereo->height)) return false;

    const Eigen::Vector3f PO = p.pos - twc;
    const float dist = PO.norm();
    if (dist < p.minDistanceInvariance() || dist > p.maxDistanceInvariance()) return false;
    const float view_cos = PO.dot(p.normal) / dist;
    if (view_cos < cos_limit) return false;
    const int level = p.predictScale(dist, ff.log_scale_factor, ff.levels);
    if (right) {
        s.proj_xr = uv(0);
        s.proj_yr = uv(1);
        s.level_r = level;
        s.view_cos_r = view_cos;
    } else {
        s.proj_x = uv(0);
        s.proj_y = uv(1);
        s.level = level;
        s.view_cos = view_cos;
        s.depth = Pc_dist;
    }
    return true;
}

// Frame::isInFrustum
bool isInFrustum(const tracking::Frame& f, const camera::StereoRig& rig, const map::MapPoint& p, float cos_limit,
                 tracking::PointScratch& s) {
    s.level = -1;
    s.level_r = -1;
    s.in_view = inFrustum(f, rig, p, cos_limit, false, s);
    s.in_view_r = inFrustum(f, rig, p, cos_limit, true, s);
    return s.in_view || s.in_view_r;
}

float radiusByViewingCos(float view_cos) { return view_cos > 0.998f ? 2.5f : 4.0f; }

// The best and second best free feature of one view for a descriptor (view 1: right indices)
struct Best {
    int dist = 256, level = -1, dist2 = 256, level2 = -1, idx = -1;
};

// A candidate feature of a view (view 1: right index) and its descriptor distance to the point, computed before
// the matching (in parallel): whether it is still free is checked when the point's turn comes
struct Candidate {
    int idx, dist, octave;
};
using Candidates = std::vector<Candidate>;

Candidates candidates(const orb::FrameFeatures& ff, const std::vector<std::size_t>& in_area, int view,
                      const std::uint8_t* d) {
    const int offset = view == 0 ? 0 : ff.n_left;
    Candidates out;
    out.reserve(in_area.size());
    for (std::size_t c : in_area) {
        const int i = int(c) + offset;
        out.push_back({int(c), orb::hamming256(d, ff.descriptor(i)), ff.keypoint(i).octave});
    }
    return out;
}

Best bestTwo(const tracking::Frame& f, const map::Atlas& a, const Candidates& cands, int view) {
    const orb::FrameFeatures& ff = *f.features;
    const int offset = view == 0 ? 0 : ff.n_left;
    Best b;
    for (const Candidate& c : cands) {
        const int i = c.idx + offset;
        if (const map::MapPoint* q = a.point(f.points[std::size_t(i)]); q && q->n_obs > 0) continue;
        const int dist = c.dist;
        const int octave = c.octave;
        if (dist < b.dist) {
            b.dist2 = b.dist;
            b.dist = dist;
            b.level2 = b.level;
            b.level = octave;
            b.idx = c.idx;
        } else if (dist < b.dist2) {
            b.level2 = octave;
            b.dist2 = dist;
        }
    }
    return b;
}

// ORBmatcher(0.8)::SearchByProjection(Frame&, vector<MapPoint*>, th, bFarPoints, thFarPoints)
// Phase 1 on par: per local point and view, the features in its search area and their descriptor distances
// (independent of the matches). Phase 2 in the points' order: the matches, as the original's single loop.
int searchByProjection(tracking::Tracker& t, const map::Atlas& a, float th, bool far_points, float th_far_points,
                       const ParallelFor& par) {
    constexpr float kRatio = 0.8f;
    tracking::Frame& f = t.current;
    const orb::FrameFeatures& ff = *f.features;
    const std::vector<int>& left_to_right = ff.stereo->left_to_right;
    const std::vector<int>& right_to_left = ff.stereo->right_to_left;
    const bool factor = th != 1.0f;
    int nmatches = 0;

    // the points that are searched: in view, near enough, good
    auto searched = [&](map::MapPointId id) -> const map::MapPoint* {
        const tracking::PointScratch& s = t.point_scratch[std::size_t(id)];
        if (!s.in_view && !s.in_view_r) return nullptr;
        if (far_points && s.depth > th_far_points) return nullptr;
        const map::MapPoint* p = a.point(id);
        return p && !p->bad ? p : nullptr;
    };
    const int n = int(t.local_points.size());
    std::vector<Candidates> cands[2];
    cands[0].resize(std::size_t(n));
    cands[1].resize(std::size_t(n));
    runParallel(par, 0, kParallelChunks, [&](int c) {
        for (int k = chunkBegin(n, c); k < chunkBegin(n, c + 1); ++k) {
            const map::MapPointId id = t.local_points[std::size_t(k)];
            const map::MapPoint* p = searched(id);
            if (!p) continue;
            const tracking::PointScratch& s = t.point_scratch[std::size_t(id)];
            const std::uint8_t* d = p->descriptor.data();
            if (s.in_view) {
                const int level = s.level;
                float r = radiusByViewingCos(s.view_cos);
                if (factor) r *= th;
                cands[0][std::size_t(k)] = candidates(
                    ff, ff.inArea(0, s.proj_x, s.proj_y, r * ff.scale_factors[std::size_t(level)], level - 1, level),
                    0, d);
            }
            if (s.in_view_r && s.level_r != -1) {
                const int level = s.level_r;
                const float r = radiusByViewingCos(s.view_cos_r);  // no th factor, as the original
                cands[1][std::size_t(k)] = candidates(
                    ff, ff.inArea(1, s.proj_xr, s.proj_yr, r * ff.scale_factors[std::size_t(level)], level - 1, level),
                    1, d);
            }
        }
    });

    for (int k = 0; k < n; ++k) {
        const map::MapPointId id = t.local_points[std::size_t(k)];
        if (!searched(id)) continue;
        const tracking::PointScratch& s = t.point_scratch[std::size_t(id)];

        if (s.in_view) {
            const Candidates& cl = cands[0][std::size_t(k)];
            if (!cl.empty()) {
                const Best b = bestTwo(f, a, cl, 0);
                if (b.dist <= kThHigh) {
                    // As the original: a failed ratio test on the same level skips the right view too
                    if (b.level == b.level2 && float(b.dist) > kRatio * float(b.dist2)) continue;
                    if (b.level != b.level2 || float(b.dist) <= kRatio * float(b.dist2)) {
                        f.points[std::size_t(b.idx)] = id;
                        if (left_to_right[std::size_t(b.idx)] != -1) {
                            f.points[std::size_t(left_to_right[std::size_t(b.idx)] + ff.n_left)] = id;
                            ++nmatches;
                        }
                        ++nmatches;
                    }
                }
            }
        }

        if (s.in_view_r) {
            if (s.level_r == -1) continue;
            const Candidates& cr = cands[1][std::size_t(k)];
            if (cr.empty()) continue;
            const Best b = bestTwo(f, a, cr, 1);
            if (b.dist <= kThHigh) {
                if (b.level == b.level2 && float(b.dist) > kRatio * float(b.dist2)) continue;
                if (right_to_left[std::size_t(b.idx)] != -1) {
                    f.points[std::size_t(right_to_left[std::size_t(b.idx)])] = id;
                    ++nmatches;
                }
                f.points[std::size_t(b.idx + ff.n_left)] = id;
                ++nmatches;
            }
        }
    }
    return nmatches;
}

}  // namespace

void updateLocalMap(const map::Atlas& a, tracking::Tracker& t) {
    updateLocalKeyFrames(a, t);
    updateLocalPoints(a, t);
}

void searchLocalPoints(const map::Atlas& a, tracking::Tracker& t, tracking::State state, const tracking::MapperStatus& mapper,
                       const ParallelFor& par) {
    tracking::Frame& f = t.current;
    // The frame's points are not searched again
    for (map::MapPointId& id : f.points) {
        if (id == map::kNone) continue;
        const map::MapPoint* p = a.point(id);
        if (!p || p->bad) {
            id = map::kNone;
            continue;
        }
        ++t.counts[id].visible;  // IncreaseVisible
        tracking::PointScratch& s = pointScratch(t, id);
        s.last_frame_seen = f.id;
        s.in_view = false;
        s.in_view_r = false;
    }

    // The local points in the frustum (the tests on par: each point's scratch is its own; the scratch is sized for
    // the largest id first)
    const camera::StereoRig& rig = rigOf(a, f);
    const int n = int(t.local_points.size());
    if (n > 0) pointScratch(t, *std::max_element(t.local_points.begin(), t.local_points.end()));
    std::vector<std::uint8_t> visible(std::size_t(n), 0);
    runParallel(par, 0, kParallelChunks, [&](int c) {
        for (int k = chunkBegin(n, c); k < chunkBegin(n, c + 1); ++k) {
            const map::MapPointId id = t.local_points[std::size_t(k)];
            tracking::PointScratch& s = t.point_scratch[std::size_t(id)];
            if (s.last_frame_seen == f.id) continue;
            const map::MapPoint* p = a.point(id);
            if (!p || p->bad) continue;
            visible[std::size_t(k)] = isInFrustum(f, rig, *p, 0.5f, s);
        }
    });
    int to_match = 0;
    for (int k = 0; k < n; ++k)
        if (visible[std::size_t(k)]) {
            ++t.counts[t.local_points[std::size_t(k)]].visible;  // IncreaseVisible
            ++to_match;
        }
    if (to_match == 0) return;

    int th = 1;
    if (a.imuInitialized()) th = a.inertialBA2() ? 2 : 6;
    else if (t.inertial) th = 10;
    if (f.id < t.last_reloc_frame_id + 2) th = 5;  // relocalized recently: coarser
    if (state == tracking::State::LOST || state == tracking::State::RECENTLY_LOST) th = 15;
    searchByProjection(t, a, float(th), mapper.far_points, mapper.th_far_points, par);
}

}  // namespace orb_tracking
