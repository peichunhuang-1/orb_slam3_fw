#include "algorithms/orb_loop_closing/place_recognition/place_recognition.hpp"

#include <cstdio>
#include <string>
#include <unordered_set>

#include "algorithms/orb_loop_closing/keyframe_matching/keyframe_matching.hpp"
#include "algorithms/orb_loop_closing/optimize_relative_pose/optimize_relative_pose.hpp"
#include "algorithms/orb_loop_closing/relative_pose_ransac/relative_pose_ransac.hpp"

namespace orb_loop_closing {

namespace {

using geometry::SE3d;
using geometry::SE3f;

SE3d toD(const SE3f& T) { return T.cast<double>(); }
SE3f toF(const SE3d& T) { return T.cast<float>(); }

void notErase(PlaceRecognition::Flags& flags, map::KeyFrameId k) {
    if (k != map::kNone) flags.push_back({k, true});
}
void erase(PlaceRecognition::Flags& flags, map::KeyFrameId k) {
    if (k != map::kNone) flags.push_back({k, false});
}

bool isGood(const map::Atlas& a, map::KeyFrameId k) { return a.goodKeyFrame(k) != nullptr; }

}  // namespace

void PlaceRecognition::clear() {
    loop_ = CommonRegion();
    merge_ = CommonRegion();
    scratch_.clear();
}

void PlaceRecognition::resetRegion(CommonRegion& r, Flags& flags) {
    erase(flags, r.last_current_kf);
    erase(flags, r.matched_kf);
    r.coincidences = 0;
    r.matched_points.clear();
    r.points.clear();
    r.not_found = 0;
    r.detected = false;
}

void PlaceRecognition::checkRegion(CommonRegion& r) {
    if (r.coincidences <= 0) return;
    if (isGood(*atlas_, r.matched_kf) && isGood(*atlas_, r.last_current_kf)) return;
    ++stats_.regions_lost;
    r = CommonRegion();
}

bool PlaceRecognition::detect(const map::Atlas& a, map::KeyFrameId current, Flags& flags) {
    atlas_ = &a;
    current_ = a.goodKeyFrame(current);
    ++stats_.keyframes;
    if (!current_) return false;
    const map::KeyFrame& cur = *current_;
    const map::Map* last_map = a.findMap(cur.map);
    if (!last_map) return false;

    // Early outs (the keyframe is in the BoW index already)
    const std::size_t map_kfs = last_map->keyframes.size();
    if ((last_map->inertial && !last_map->imu_ba2) || (!last_map->inertial && map_kfs < 5) || map_kfs < 12) {
        ++stats_.skipped;
        erase(flags, current);
        return false;
    }

    checkRegion(loop_);
    checkRegion(merge_);

    // The last regions, from the last current keyframe's pose
    bool loop_in_kf = false;
    if (loop_.coincidences > 0) {
        const map::KeyFrame& last = *a.keyframe(loop_.last_current_kf);
        const map::KeyFrame& matched = *a.keyframe(loop_.matched_kf);
        const SE3d Tcl = toD(cur.pose() * last.poseInverse());
        SE3d Tcw = Tcl * loop_.Tlw;
        int n_proj = 0;
        std::vector<map::MapPointId> matched_points;
        const bool common = refinePoseFromLastKeyFrame(cur, matched, Tcw, n_proj, loop_.points, matched_points);
        if (debug)
            std::fprintf(stderr, "[LC] kf %lu refine against kf %lu: %s, projection matches %d, coincidences %d\n",
                         (unsigned long)cur.id, (unsigned long)matched.id, common ? "confirmed" : "not confirmed",
                         n_proj, loop_.coincidences);
        if (common) {
            loop_in_kf = true;
            ++loop_.coincidences;
            erase(flags, loop_.last_current_kf);
            loop_.last_current_kf = current;
            loop_.Tlw = Tcw;
            loop_.matched_points = std::move(matched_points);
            loop_.detected = loop_.coincidences >= 3;
            loop_.not_found = 0;
        } else {
            if (++loop_.not_found >= 2) {
                erase(flags, loop_.last_current_kf);
                erase(flags, loop_.matched_kf);
                loop_.coincidences = 0;
                loop_.matched_points.clear();
                loop_.points.clear();
                loop_.not_found = 0;
            }
        }
    }

    bool merge_in_kf = false;
    if (merge_.coincidences > 0) {
        const map::KeyFrame& last = *a.keyframe(merge_.last_current_kf);
        const map::KeyFrame& matched = *a.keyframe(merge_.matched_kf);
        const SE3d Tcl = toD(cur.pose() * last.poseInverse());
        SE3d Tcw = Tcl * merge_.Tlw;
        int n_proj = 0;
        std::vector<map::MapPointId> matched_points;
        const bool common = refinePoseFromLastKeyFrame(cur, matched, Tcw, n_proj, merge_.points, matched_points);
        if (common) {
            merge_in_kf = true;
            ++merge_.coincidences;
            erase(flags, merge_.last_current_kf);
            merge_.last_current_kf = current;
            merge_.Tlw = Tcw;
            merge_.matched_points = std::move(matched_points);
            merge_.detected = merge_.coincidences >= 3;
        } else {
            merge_.detected = false;
            if (++merge_.not_found >= 2) {
                erase(flags, merge_.last_current_kf);
                erase(flags, merge_.matched_kf);
                merge_.coincidences = 0;
                merge_.matched_points.clear();
                merge_.points.clear();
                merge_.not_found = 0;
            }
        }
    }

    if (merge_.detected || loop_.detected) return true;

    // BoW candidates
    std::vector<map::KeyFrameId> merge_candidates, loop_candidates;
    if (!merge_in_kf || !loop_in_kf) {
        detectNBestCandidates(a, scratch_, current, 3, loop_candidates, merge_candidates);
        stats_.candidates_loop += long(loop_candidates.size());
        stats_.candidates_merge += long(merge_candidates.size());
        if (debug)
            std::fprintf(stderr,
                         "[LC] kf %lu frame %ld map KFs %zu covisible %zu loop candidates %zu merge candidates %zu "
                         "coincidences %d\n",
                         (unsigned long)cur.id, cur.data->frame_id, map_kfs, cur.ordered_connected.size(),
                         loop_candidates.size(), merge_candidates.size(), loop_.coincidences);
    }
    if (!loop_in_kf && !loop_candidates.empty()) loop_.detected = detectFromBoW(loop_candidates, loop_, flags);
    if (!merge_in_kf && !merge_candidates.empty()) merge_.detected = detectFromBoW(merge_candidates, merge_, flags);

    if (merge_.detected || loop_.detected) return true;
    erase(flags, current);
    return false;
}

// DetectAndReffineSim3FromLastKF. As the original, the pose it confirms is the one it was given: the optimized
// one only gates (gScw_estimation is built from gScw, not from the optimized gScm).
bool PlaceRecognition::refinePoseFromLastKeyFrame(const map::KeyFrame& cur, const map::KeyFrame& matched, SE3d& Tcw,
                                                  int& n_proj, std::vector<map::MapPointId>& points,
                                                  std::vector<map::MapPointId>& matched_points) {
    n_proj = findMatchesByProjection(cur, matched, Tcw, points, matched_points);
    constexpr int kProjMatches = 30, kProjOptMatches = 50, kProjMatchesRep = 100;
    if (n_proj < kProjMatches) return false;

    SE3d Tcm = Tcw * toD(matched.poseInverse());
    const int n_opt = optimizeRelativePose(*atlas_, cur, matched, matched_points, Tcm, 10, true);
    if (n_opt <= kProjOptMatches) return false;

    const SE3d Tcw_estimation = Tcw;
    n_proj = findMatchesByProjection(cur, matched, Tcw_estimation, points, matched_points);
    if (n_proj >= kProjMatchesRep) {
        Tcw = Tcw_estimation;
        return true;
    }
    return false;
}

bool PlaceRecognition::detectFromBoW(const std::vector<map::KeyFrameId>& candidates, CommonRegion& r, Flags& flags) {
    constexpr int kBoWMatches = 20, kBoWInliers = 15, kSim3Inliers = 20, kProjMatches = 50, kProjOptMatches = 80;
    constexpr int kCovisibles = 10;
    const map::Atlas& a = *atlas_;
    const map::KeyFrame& cur = *current_;
    const std::size_t N = cur.points.size();

    map::KeyFrameId best_kf = map::kNone;
    int best_matches_reproj = 0, best_coincidences = 0;
    SE3d best_Tcw;
    std::vector<map::MapPointId> best_points, best_matched_points;

    for (map::KeyFrameId cand : candidates) {
        const map::KeyFrame* kfi = a.goodKeyFrame(cand);
        if (!kfi) continue;
        int d_bow = -1, d_ransac = -1, d_proj = -1, d_opt = -1, d_proj_opt = -1, d_kfs = -1;
        auto report = [&](const char* stage) {
            if (!debug) return;
            const auto it = scratch_.find(cand);
            std::fprintf(stderr,
                         "[LC]   cand kf %lu (score %g): %s | bow %d/%d sim3 inliers %d/%d proj %d/%d opt %d/%d "
                         "projopt %d/%d neighbour KFs %d/3\n",
                         (unsigned long)cand, it == scratch_.end() ? 0.0 : double(it->second.score), stage, d_bow,
                         kBoWMatches, d_ransac, kBoWInliers, d_proj, kProjMatches, d_opt, kSim3Inliers, d_proj_opt,
                         kProjOptMatches, d_kfs);
        };

        // The candidate and its covisibles (the candidate first, the best covisible moved last)
        std::vector<map::KeyFrameId> cov = kfi->bestCovisibles(kCovisibles);
        if (cov.empty()) {
            cov.push_back(cand);
        } else {
            cov.push_back(cov[0]);
            cov[0] = cand;
        }
        bool near_current = false;
        for (map::KeyFrameId k : cov)
            if (cur.connected.count(k)) {
                near_current = true;
                break;
            }
        if (near_current) {
            report("skipped: covisible with the current KF");
            continue;
        }

        // BoW matches against each, merged: a point once, its feature's last keyframe kept
        std::vector<std::vector<map::MapPointId>> bow_matches(cov.size());
        for (std::size_t j = 0; j < cov.size(); ++j) {
            const map::KeyFrame* kj = a.goodKeyFrame(cov[j]);
            if (!kj) continue;
            searchByBoW(a, cur, *kj, 0.9f, true, bow_matches[j]);
        }
        std::unordered_set<map::MapPointId> seen;
        int n_bow = 0;
        std::vector<map::MapPointId> matched_points(N, map::kNone);
        std::vector<map::KeyFrameId> matched_kfs(N, map::kNone);
        for (std::size_t j = 0; j < cov.size(); ++j)
            for (std::size_t k = 0; k < bow_matches[j].size(); ++k) {
                const map::MapPointId p = bow_matches[j][k];
                if (!a.goodPoint(p)) continue;
                if (seen.insert(p).second) {
                    ++n_bow;
                    matched_points[k] = p;
                    matched_kfs[k] = cov[j];
                }
            }

        d_bow = n_bow;
        const map::KeyFrame& most = *kfi;  // pMostBoWMatchesKF: the candidate (its update is commented out)
        if (n_bow >= kBoWMatches) {
            RelativePoseRansac solver(a, cur, most, matched_points, matched_kfs, seed_++);
            solver.setRansacParameters(0.99, kBoWInliers, 300);
            bool no_more = false, converged = false;
            std::vector<bool> inliers;
            int n_inliers = 0;
            while (!converged && !no_more) solver.iterate(20, no_more, inliers, n_inliers, converged);
            d_ransac = converged ? n_inliers : 0;

            if (converged) {
                // The points of the matched keyframe's neighbourhood
                std::vector<map::KeyFrameId> cov_m = most.bestCovisibles(kCovisibles);
                cov_m.push_back(most.id);
                std::unordered_set<map::MapPointId> seen_points;
                std::vector<map::MapPointId> points;
                std::vector<map::KeyFrameId> point_kfs;
                for (map::KeyFrameId k : cov_m) {
                    const map::KeyFrame* kk = a.keyframe(k);
                    if (!kk) continue;
                    for (map::MapPointId p : kk->points) {
                        if (!a.goodPoint(p)) continue;
                        if (seen_points.insert(p).second) {
                            points.push_back(p);
                            point_kfs.push_back(k);
                        }
                    }
                }

                SE3d Tcm(solver.bestRotation().cast<double>(), solver.bestTranslation().cast<double>());
                const SE3d Tmw = toD(most.pose());
                SE3d Tcw = Tcm * Tmw;
                std::vector<map::MapPointId> matched_mp(N, map::kNone);
                std::vector<map::KeyFrameId> matched_kf(N, map::kNone);
                const int n_proj = searchByProjection(a, cur, toF(Tcw), points, point_kfs, matched_mp, matched_kf, 8, 1.5f);
                d_proj = n_proj;

                if (n_proj >= kProjMatches) {
                    const int n_opt = optimizeRelativePose(a, cur, *kfi, matched_mp, Tcm, 10, true);
                    d_opt = n_opt;
                    if (n_opt >= kSim3Inliers) {
                        Tcw = Tcm * Tmw;
                        std::vector<map::MapPointId> matched_opt(N, map::kNone);
                        const int n_proj_opt = searchByProjection(a, cur, toF(Tcw), points, matched_opt, 5, 1.0f);
                        d_proj_opt = n_proj_opt;

                        if (n_proj_opt >= kProjOptMatches) {
                            // The pose checked from the current keyframe's covisibles (as the original, each check
                            // replaces `points` with that keyframe's neighbourhood points)
                            int n_kfs = 0;
                            const std::vector<map::KeyFrameId> cur_cov = cur.bestCovisibles(kCovisibles);
                            for (std::size_t j = 0; n_kfs < 3 && j < cur_cov.size(); ++j) {
                                const map::KeyFrame* kj = a.keyframe(cur_cov[j]);
                                if (!kj) continue;
                                const SE3d Tjc = toD(kj->pose() * cur.poseInverse());
                                const SE3d Tjw = Tjc * Tcw;
                                int n_proj_j = 0;
                                std::vector<map::MapPointId> matched_j;
                                if (detectFromLastKeyFrame(*kj, most, Tjw, n_proj_j, points, matched_j)) ++n_kfs;
                            }
                            d_kfs = n_kfs;

                            if (best_matches_reproj < n_proj_opt) {
                                best_matches_reproj = n_proj_opt;
                                best_coincidences = n_kfs;
                                best_kf = most.id;
                                best_Tcw = Tcw;
                                best_points = points;
                                best_matched_points = matched_opt;
                            }
                        }
                    }
                }
            }
        }
        report(d_bow < kBoWMatches         ? "rejected: BoW matches"
               : d_ransac <= 0             ? "rejected: Sim3 RANSAC"
               : d_proj < kProjMatches     ? "rejected: projection"
               : d_opt < kSim3Inliers      ? "rejected: Sim3 optimisation"
               : d_proj_opt < kProjOptMatches ? "rejected: projection after optimisation"
               : d_kfs < 3                 ? "accepted, waiting for more coincidences"
                                           : "accepted: loop");
    }

    if (best_matches_reproj > 0) {
        r.last_current_kf = cur.id;
        r.coincidences = best_coincidences;
        r.matched_kf = best_kf;
        notErase(flags, best_kf);
        r.Tlw = best_Tcw;
        r.points = std::move(best_points);
        r.matched_points = std::move(best_matched_points);
        return r.coincidences >= 3;
    }
    return false;
}

bool PlaceRecognition::detectFromLastKeyFrame(const map::KeyFrame& cur, const map::KeyFrame& matched, const SE3d& Tcw,
                                              int& n_proj, std::vector<map::MapPointId>& points,
                                              std::vector<map::MapPointId>& matched_points) {
    n_proj = findMatchesByProjection(cur, matched, Tcw, points, matched_points);
    return n_proj >= 30;
}

int PlaceRecognition::findMatchesByProjection(const map::KeyFrame& cur, const map::KeyFrame& matched, const SE3d& Tcw,
                                              std::vector<map::MapPointId>& points,
                                              std::vector<map::MapPointId>& matched_points) {
    constexpr int kCovisibles = 10;
    const map::Atlas& a = *atlas_;
    std::vector<map::KeyFrameId> cov = matched.bestCovisibles(kCovisibles);
    const std::size_t n_initial = cov.size();
    cov.push_back(matched.id);
    std::unordered_set<map::KeyFrameId> check(cov.begin(), cov.end());
    if (int(n_initial) < kCovisibles) {
        for (std::size_t i = 0; i < n_initial; ++i) {
            const map::KeyFrame* ki = a.keyframe(cov[i]);
            if (!ki) continue;
            const std::vector<map::KeyFrameId> kfs = ki->bestCovisibles(kCovisibles);
            int inserted = 0;
            for (std::size_t j = 0; j < kfs.size() && inserted < kCovisibles; ++j)
                if (!check.count(kfs[j]) && !cur.connected.count(kfs[j])) {
                    check.insert(kfs[j]);
                    ++inserted;
                }
            cov.insert(cov.end(), kfs.begin(), kfs.end());  // all of them, as the original
        }
    }
    std::unordered_set<map::MapPointId> seen;
    points.clear();
    matched_points.clear();
    for (map::KeyFrameId k : cov) {
        const map::KeyFrame* kk = a.keyframe(k);
        if (!kk) continue;
        for (map::MapPointId p : kk->points)
            if (a.goodPoint(p) && seen.insert(p).second) points.push_back(p);
    }
    matched_points.assign(cur.points.size(), map::kNone);
    return searchByProjection(a, cur, toF(Tcw), points, matched_points, 3, 1.5f);
}

}  // namespace orb_loop_closing
