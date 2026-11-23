#include "algorithms/orb_loop_closing/keyframe_matching/keyframe_matching.hpp"

#include <unordered_set>

#include "algorithms/orb_tracking/shared/rotation_histogram.hpp"
#include "types/orb/hamming.hpp"

namespace orb_loop_closing {

namespace {

constexpr int kThLow = 50;  // ORBmatcher::TH_LOW

enum class Projection { kb8, pinhole };

int project(const map::Atlas& a, const map::KeyFrame& kf, const geometry::SE3f& Tcw,
            const std::vector<map::MapPointId>& points, const std::vector<map::KeyFrameId>* point_kfs,
            std::vector<map::MapPointId>& matched, std::vector<map::KeyFrameId>* matched_kfs, int th, float ratio,
            Projection model) {
    const orb::FrameFeatures& f = *kf.data->features;
    const camera::KannalaBrandt8& cam = a.rig(kf.data->rig).camera(0);
    const float fx = cam.fx(), fy = cam.fy(), cx = cam.cx(), cy = cam.cy();
    const Eigen::Vector3f Ow = Tcw.inverse().translation();

    std::unordered_set<map::MapPointId> already(matched.begin(), matched.end());
    already.erase(map::kNone);

    int nmatches = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const map::MapPoint* p = a.point(points[i]);
        if (!p || p->bad || already.count(points[i])) continue;

        const Eigen::Vector3f Pw = p->pos;
        const Eigen::Vector3f Pc = Tcw * Pw;
        if (Pc(2) < 0.0f) continue;
        Eigen::Vector2f uv;
        if (model == Projection::kb8) {
            uv = cam.project_orb(Pc);
        } else {
            const float invz = 1 / Pc(2);
            uv = Eigen::Vector2f(fx * Pc(0) * invz + cx, fy * Pc(1) * invz + cy);
        }
        if (!f.inImage(uv(0), uv(1))) continue;

        // Depth inside the scale invariance region, viewing angle under 60°
        const Eigen::Vector3f PO = Pw - Ow;
        const float dist = PO.norm();
        if (dist < p->minDistanceInvariance() || dist > p->maxDistanceInvariance()) continue;
        if (PO.dot(p->normal) < 0.5 * dist) continue;

        const int level = p->predictScale(dist, f.log_scale_factor, f.levels);
        const float radius = float(th) * f.scale_factors[std::size_t(level)];
        const std::vector<std::size_t> near = f.inArea(0, uv(0), uv(1), radius);
        if (near.empty()) continue;

        int best_dist = 256, best_idx = -1;
        for (std::size_t j : near) {
            if (matched[j] != map::kNone) continue;
            const int kp_level = f.keys(0)[j].octave;
            if (kp_level < level - 1 || kp_level > level) continue;
            const int d = orb::hamming256(p->descriptor.data(), f.descriptor(int(j)));
            if (d < best_dist) {
                best_dist = d;
                best_idx = int(j);
            }
        }
        if (float(best_dist) <= float(kThLow) * ratio) {
            matched[std::size_t(best_idx)] = points[i];
            if (matched_kfs) (*matched_kfs)[std::size_t(best_idx)] = (*point_kfs)[i];
            ++nmatches;
        }
    }
    return nmatches;
}

}  // namespace

int searchByBoW(const map::Atlas& a, const map::KeyFrame& kf1, const map::KeyFrame& kf2, float nn_ratio,
                bool check_orientation, std::vector<map::MapPointId>& matches12) {
    const orb::FrameFeatures& f1 = *kf1.data->features;
    const orb::FrameFeatures& f2 = *kf2.data->features;
    const unsigned n_left1 = unsigned(f1.n_left), n_left2 = unsigned(f2.n_left);
    const std::vector<map::MapPointId> points1 = kf1.points, points2 = kf2.points;
    matches12.assign(points1.size(), map::kNone);
    std::vector<bool> matched2(points2.size(), false);
    orb_tracking::RotationHistogram hist;
    int nmatches = 0;

    auto it1 = kf1.data->feat.begin(), it2 = kf2.data->feat.begin();
    const auto end1 = kf1.data->feat.end(), end2 = kf2.data->feat.end();
    while (it1 != end1 && it2 != end2) {
        if (it1->first == it2->first) {
            for (unsigned idx1 : it1->second) {
                if (idx1 >= n_left1) continue;
                if (!a.goodPoint(points1[idx1])) continue;
                const std::uint8_t* d1 = f1.descriptor(int(idx1));
                int best1 = 256, best2 = 256, best_idx2 = -1;
                for (unsigned idx2 : it2->second) {
                    if (idx2 >= n_left2) continue;
                    if (matched2[idx2] || points2[idx2] == map::kNone) continue;
                    if (!a.goodPoint(points2[idx2])) continue;
                    const int dist = orb::hamming256(d1, f2.descriptor(int(idx2)));
                    if (dist < best1) {
                        best2 = best1;
                        best1 = dist;
                        best_idx2 = int(idx2);
                    } else if (dist < best2) {
                        best2 = dist;
                    }
                }
                if (best1 < kThLow && float(best1) < nn_ratio * float(best2)) {
                    matches12[idx1] = points2[std::size_t(best_idx2)];
                    matched2[std::size_t(best_idx2)] = true;
                    if (check_orientation)
                        hist.add(f1.keypoint(int(idx1)).angle, f2.keypoint(best_idx2).angle, int(idx1));
                    ++nmatches;
                }
            }
            ++it1;
            ++it2;
        } else if (it1->first < it2->first) {
            it1 = kf1.data->feat.lower_bound(it2->first);
        } else {
            it2 = kf2.data->feat.lower_bound(it1->first);
        }
    }

    if (check_orientation)
        hist.forEachRejected([&](int i) {
            matches12[std::size_t(i)] = map::kNone;
            --nmatches;
        });
    return nmatches;
}

int searchByProjection(const map::Atlas& a, const map::KeyFrame& kf, const geometry::SE3f& Tcw,
                       const std::vector<map::MapPointId>& points, std::vector<map::MapPointId>& matched, int th,
                       float ratio_hamming) {
    return project(a, kf, Tcw, points, nullptr, matched, nullptr, th, ratio_hamming, Projection::kb8);
}

int searchByProjection(const map::Atlas& a, const map::KeyFrame& kf, const geometry::SE3f& Tcw,
                       const std::vector<map::MapPointId>& points, const std::vector<map::KeyFrameId>& point_kfs,
                       std::vector<map::MapPointId>& matched, std::vector<map::KeyFrameId>& matched_kfs, int th,
                       float ratio_hamming) {
    return project(a, kf, Tcw, points, &point_kfs, matched, &matched_kfs, th, ratio_hamming, Projection::pinhole);
}

}  // namespace orb_loop_closing
