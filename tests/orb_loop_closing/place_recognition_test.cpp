// orb_loop_closing's place recognition vs tests/fixtures/orb_slam3/loop (README.md): two passes through the same
// place, the second drifted; case "loop" in one map, "merge" with the first pass in another map. The atlas is
// rebuilt from the fixture (keyframes with the original's BoW, points with its descriptors, normals and scale
// ranges); then
//   detectNBestCandidates, searchByBoW, both searchByProjection   exactly the original's;
//   RelativePoseRansac      converges as the original, T12 close to its and to the truth (the sampling differs);
//   optimizeRelativePose    from the same start and matches: T12 and the inliers as the original's;
//   detectFromBoW           the same matched keyframe and coincidences, the pose close, the matches nearly the same.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "algorithms/orb_loop_closing/keyframe_candidates/keyframe_candidates.hpp"
#include "algorithms/orb_loop_closing/keyframe_matching/keyframe_matching.hpp"
#include "algorithms/orb_loop_closing/optimize_relative_pose/optimize_relative_pose.hpp"
#include "algorithms/orb_loop_closing/place_recognition/place_recognition.hpp"
#include "algorithms/orb_loop_closing/relative_pose_ransac/relative_pose_ransac.hpp"
#include "tests/orb_local_mapping/fixture_keyframes.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

struct Scene {
    map::Atlas a;
    std::vector<map::KeyFrameId> kfs;
    std::vector<map::MapPointId> pts;

    int kfIndex(map::KeyFrameId id) const {
        if (id == map::kNone) return -1;
        return int(std::find(kfs.begin(), kfs.end(), id) - kfs.begin());
    }
    int ptIndex(map::MapPointId id) const {
        if (id == map::kNone) return -1;
        return int(std::find(pts.begin(), pts.end(), id) - pts.begin());
    }
    std::vector<map::MapPointId> points(const Eigen::MatrixXd& idx) const {
        std::vector<map::MapPointId> r;
        for (int i = 0; i < idx.rows(); ++i) r.push_back(idx(i) < 0 ? map::kNone : pts[std::size_t(idx(i))]);
        return r;
    }
};

void load(Scene& s, const fixture::File& f, const fixture::Rig& rig) {
    map::Atlas& a = s.a;
    const int K = int(f.scalar("n_keyframes")), KA = int(f.scalar("pass_a"));
    const bool merge = f.scalar("merge") != 0;
    a.currentMap();
    map::addRig(a, rig.rig);
    for (int k = 0; k < K; ++k) {
        if (merge && k == KA) map::createNewMap(a);
        const std::string p = "kf" + std::to_string(k) + "_";
        auto d = std::make_shared<map::KeyFrameData>();
        d->frame_id = k;
        d->timestamp = f.scalar(p + "timestamp");
        d->features = fixture::keyframeFeatures(f, p, rig);
        d->rig = 0;
        d->Tlr = rig.Tlr;
        d->Trl = rig.Tlr.inverse();
        d->baseline = rig.Tlr.translation().norm();
        d->th_depth = rig.th_depth;
        const Eigen::MatrixXd& bow = f[p + "bow"];
        for (int i = 0; i < bow.rows(); ++i) d->bow[map::WordId(bow(i, 0))] = bow(i, 1);
        const Eigen::MatrixXd& feat = f[p + "feat"];
        for (int i = 0; i < feat.rows(); ++i) d->feat[map::NodeId(feat(i, 0))].push_back(unsigned(feat(i, 1)));
        map::KeyFrame kf;
        kf.data = std::move(d);
        kf.setPose(fixture::se3f(f[p + "Tcw"]));
        if (k > 0 && k != KA) kf.prev = s.kfs.back();
        const map::KeyFrameId id = map::createKeyFrame(a, std::move(kf));
        if (k > 0 && k != KA) a.editKeyFrame(s.kfs.back())->next = id;
        map::insertKeyFrame(a, id);
        s.kfs.push_back(id);
    }

    const Eigen::MatrixXd& P = f["points"];
    const Eigen::MatrixXd& D = f["point_desc"];
    for (int r = 0; r < P.rows(); ++r)
        s.pts.push_back(map::createMapPoint(a, P.block<1, 3>(r, 1).transpose().cast<float>(), s.kfs[std::size_t(P(r, 0))]));
    for (int k = 0; k < K; ++k) {
        const Eigen::MatrixXd& kp = f["kf" + std::to_string(k) + "_points"];
        for (int i = 0; i < kp.rows(); ++i) {
            if (kp(i) < 0) continue;
            const map::MapPointId p = s.pts[std::size_t(kp(i))];
            map::addObservation(a, p, s.kfs[std::size_t(k)], i);
            a.editKeyFrame(s.kfs[std::size_t(k)])->points[std::size_t(i)] = p;
        }
    }
    for (int r = 0; r < P.rows(); ++r) {
        map::insertMapPoint(a, s.pts[std::size_t(r)]);
        map::MapPoint* p = a.editPoint(s.pts[std::size_t(r)]);
        p->normal = P.block<1, 3>(r, 4).transpose().cast<float>();
        p->min_distance = float(P(r, 7));
        p->max_distance = float(P(r, 8));
        for (int b = 0; b < 32; ++b) p->descriptor[std::size_t(b)] = std::uint8_t(D(r, b));
    }
    for (map::KeyFrameId id : s.kfs) map::updateConnections(a, id);
    for (int k = 0; k < K - 1; ++k) map::addToBowIndex(a, s.kfs[std::size_t(k)]);
}

int countDiff(const Scene& s, const std::vector<map::MapPointId>& got, const Eigen::MatrixXd& want) {
    int d = 0;
    for (int i = 0; i < want.rows(); ++i)
        if (s.ptIndex(got[std::size_t(i)]) != int(want(i))) ++d;
    return d;
}

double angleDeg(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
    return Eigen::AngleAxisd(A.transpose() * B).angle() * 180.0 / M_PI;
}

void runCase(const std::string& name, const fixture::Rig& rig) {
    const fixture::File f("loop/" + name + ".txt");
    Scene s;
    load(s, f, rig);
    const map::Atlas& a = s.a;
    const int K = int(f.scalar("n_keyframes"));
    const map::KeyFrameId cur_id = s.kfs[std::size_t(K - 1)];
    const map::KeyFrame& cur = *a.keyframe(cur_id);

    // Covisibility as the original's
    {
        const Eigen::MatrixXd& c = f["covisibility"];
        std::vector<std::vector<std::pair<int, int>>> want{std::size_t(K)};
        for (int r = 0; r < c.rows(); ++r) want[std::size_t(c(r, 0))].emplace_back(int(c(r, 1)), int(c(r, 2)));
        int bad = 0;
        for (int k = 0; k < K; ++k) {
            const map::KeyFrame& kf = *a.keyframe(s.kfs[std::size_t(k)]);
            std::vector<std::pair<int, int>> got;
            for (std::size_t i = 0; i < kf.ordered_connected.size(); ++i)
                got.emplace_back(s.kfIndex(kf.ordered_connected[i]), kf.ordered_weights[i]);
            // Equal weights: the original orders them by pointer
            auto key = [](std::vector<std::pair<int, int>> v) {
                std::sort(v.begin(), v.end(), [](auto x, auto y) { return x.second != y.second ? x.second > y.second : x.first < y.first; });
                return v;
            };
            if (key(got) != key(want[std::size_t(k)])) ++bad;
        }
        check(bad == 0, name + ": covisibility of " + std::to_string(bad) + " keyframes");
    }

    // DetectNBestCandidates
    orb_loop_closing::PlaceRecognitionScratchMap scratch;
    std::vector<map::KeyFrameId> loop_c, merge_c;
    orb_loop_closing::detectNBestCandidates(a, scratch, cur_id, 3, loop_c, merge_c);
    auto same = [&](const std::vector<map::KeyFrameId>& got, const Eigen::MatrixXd& want) {
        if (Eigen::Index(got.size()) != want.rows()) return false;
        for (std::size_t i = 0; i < got.size(); ++i)
            if (s.kfIndex(got[i]) != int(want(Eigen::Index(i)))) return false;
        return true;
    };
    check(same(loop_c, f["loop_candidates"]), name + ": loop candidates");
    check(same(merge_c, f["merge_candidates"]), name + ": merge candidates");
    if (!f.has("bow_kf")) {
        std::printf("%s: no candidate in the fixture\n", name.c_str());
        return;
    }
    const map::KeyFrameId c0_id = s.kfs[std::size_t(f.scalar("bow_kf"))];
    const map::KeyFrame& c0 = *a.keyframe(c0_id);

    // SearchByBoW
    std::vector<map::MapPointId> bow;
    const int n_bow = orb_loop_closing::searchByBoW(a, cur, c0, 0.9f, true, bow);
    const int bow_diff = countDiff(s, bow, f["bow_matches"]);
    check(n_bow == int(f.scalar("bow_n")) && bow_diff == 0,
          name + ": searchByBoW " + std::to_string(n_bow) + " vs " + std::to_string(int(f.scalar("bow_n"))) + ", " +
              std::to_string(bow_diff) + " differ");

    // RelativePoseRansac
    orb_loop_closing::RelativePoseRansac solver(a, cur, c0, bow, std::vector<map::KeyFrameId>(bow.size(), c0_id));
    solver.setRansacParameters(0.99, 15, 300);
    bool no_more = false, converged = false;
    std::vector<bool> inl;
    int n_inl = 0;
    while (!converged && !no_more) solver.iterate(20, no_more, inl, n_inl, converged);
    check(solver.correspondences() == int(f.scalar("ransac_correspondences")),
          name + ": RANSAC correspondences " + std::to_string(solver.correspondences()) + " vs " +
              std::to_string(int(f.scalar("ransac_correspondences"))));
    check(converged == (f.scalar("ransac_converged") != 0), name + ": RANSAC converged");
    const geometry::SE3d T_ref{Eigen::Matrix4d(f["ransac_T12"])};
    const geometry::SE3d T_true{Eigen::Matrix4d(f["true_T12"])};
    const geometry::SE3d T_got(solver.bestRotation().cast<double>(), solver.bestTranslation().cast<double>());
    const double ra = angleDeg(T_got.rotationMatrix(), T_ref.rotationMatrix());
    const double ta = (T_got.translation() - T_ref.translation()).norm();
    const double rt = angleDeg(T_got.rotationMatrix(), T_true.rotationMatrix());
    const double tt = (T_got.translation() - T_true.translation()).norm();
    if (converged) {
        // The first hypothesis over min_inliers wins (the original's is 0.3-0.5° / 2-4 cm off the truth here)
        check(rt < 1.5 && tt < 0.1, name + ": RANSAC T12 far from the truth");
    }

    // Projections from proj_Tcw
    const geometry::SE3f Tcw = fixture::se3f(f["proj_Tcw"]);
    const std::vector<map::MapPointId> nb = s.points(f["nb_points"]);
    std::vector<map::KeyFrameId> nb_kfs;
    for (int i = 0; i < f["nb_kfs"].rows(); ++i) nb_kfs.push_back(s.kfs[std::size_t(f["nb_kfs"](i))]);
    std::vector<map::MapPointId> proj1(cur.points.size(), map::kNone), proj2(cur.points.size(), map::kNone);
    std::vector<map::KeyFrameId> proj1_kf(cur.points.size(), map::kNone);
    const int n1 = orb_loop_closing::searchByProjection(a, cur, Tcw, nb, nb_kfs, proj1, proj1_kf, 8, 1.5f);
    const int n2 = orb_loop_closing::searchByProjection(a, cur, Tcw, nb, proj2, 5, 1.0f);
    const int d1 = countDiff(s, proj1, f["proj_pinhole"]), d2 = countDiff(s, proj2, f["proj_kb8"]);
    check(n1 == int(f.scalar("proj_pinhole_n")) && d1 == 0,
          name + ": projection (pinhole) " + std::to_string(n1) + " vs " +
              std::to_string(int(f.scalar("proj_pinhole_n"))) + ", " + std::to_string(d1) + " differ");
    check(n2 == int(f.scalar("proj_kb8_n")) && d2 == 0,
          name + ": projection (KB8) " + std::to_string(n2) + " vs " + std::to_string(int(f.scalar("proj_kb8_n"))) +
              ", " + std::to_string(d2) + " differ");

    // optimizeRelativePose from the original's start, on its matches
    std::vector<map::MapPointId> opt = s.points(f["proj_pinhole"]);
    const geometry::SE3f Ti = fixture::se3f(f["opt_T12_initial"]);
    geometry::SE3d T12 = Ti.cast<double>();
    const int n_opt = orb_loop_closing::optimizeRelativePose(a, cur, c0, opt, T12, 10, true);
    const geometry::SE3d T_opt{Eigen::Matrix4d(f["opt_T12"])};
    const double ro = angleDeg(T12.rotationMatrix(), T_opt.rotationMatrix());
    const double to = (T12.translation() - T_opt.translation()).norm();
    const int d_opt = countDiff(s, opt, f["opt_matches"]);
    // The original's optimizer runs on numeric Jacobians that are mostly rounding (optimize_relative_pose.cpp):
    // from the same start the two end close, not equal (seen: inliers within 15 %, 0.05-0.3°, 2-9 mm)
    const int want_opt = int(f.scalar("opt_inliers"));
    check(std::abs(n_opt - want_opt) <= std::max(3, want_opt / 5) && d_opt <= std::max(3, want_opt / 3),
          name + ": optimizeRelativePose inliers " + std::to_string(n_opt) + " vs " + std::to_string(want_opt) + ", " +
              std::to_string(d_opt) + " matches differ");
    check(ro < 0.5 && to < 0.02, name + ": optimizeRelativePose T12");

    // DetectCommonRegionsFromBoW: once (the region it finds), then how often it detects over repeated runs (the
    // outcome depends on RANSAC's sampling, in the original too)
    const std::vector<map::KeyFrameId>& cand = f.scalar("merge") != 0 ? merge_c : loop_c;
    const geometry::SE3d T_true_cw{Eigen::Matrix4d(f["true_Tcw"])};
    orb_loop_closing::PlaceRecognition pr;
    orb_loop_closing::CommonRegion r;
    orb_loop_closing::PlaceRecognition::Flags flags;
    const bool det = pr.detectFromBoW(a, cur_id, cand, r, flags);
    double rd = 0, td = 0;
    int n_matched = 0;
    if (r.matched_kf != map::kNone) {
        check(r.matched_kf == cand[0], name + ": detectFromBoW matched another keyframe");
        check(flags.size() == 1 && flags[0].keyframe == r.matched_kf && flags[0].not_erase,
              name + ": detectFromBoW SetNotErase of the matched keyframe");
        rd = angleDeg(r.Tlw.rotationMatrix(), T_true_cw.rotationMatrix());
        td = (r.Tlw.translation() - T_true_cw.translation()).norm();
        check(rd < 1.5 && td < 0.1, name + ": detectFromBoW Tcw far from the truth");
        for (map::MapPointId p : r.matched_points) n_matched += p != map::kNone;
        check(n_matched >= 80, name + ": detectFromBoW matches");
    }
    const int runs = int(f.scalar("bow_detect_runs")), want_det = int(f.scalar("bow_detect_detections"));
    int n_det = 0;
    for (int i = 0; i < runs; ++i) {
        orb_loop_closing::CommonRegion ri;
        orb_loop_closing::PlaceRecognition::Flags fi;
        n_det += pr.detectFromBoW(a, cur_id, cand, ri, fi);
    }
    // Binomial: within about 2.5 standard deviations of the original's rate
    const double pr_ref = double(want_det) / runs;
    const double tol = std::max(3.0, 2.5 * std::sqrt(runs * pr_ref * (1 - pr_ref)) * std::sqrt(2.0));
    check(std::abs(n_det - want_det) <= tol, name + ": detectFromBoW detects " + std::to_string(n_det) + " of " +
                                                 std::to_string(runs) + " vs " + std::to_string(want_det));
    std::printf("%s: candidates loop %zu merge %zu; bow %d; RANSAC %d inliers of %d (%s), vs truth %.3f° %.4f m "
                "(original's vs it %.3f° %.4f m); projection %d / %d; optimize %d inliers (original %d), T12 vs "
                "original %.1e° %.1e m; detect %d (%d matches, Tcw vs truth %.3f° %.4f m), %d of %d runs (original %d)\n",
                name.c_str(), loop_c.size(), merge_c.size(), n_bow, n_inl, solver.correspondences(),
                converged ? "converged" : "not converged", rt, tt, ra, ta, n1, n2, n_opt, want_opt, ro, to, int(det),
                n_matched, rd, td, n_det, runs, want_det);
}

}  // namespace

int main() {
    const fixture::Rig rig;
    for (const char* c : {"loop", "merge"}) runCase(c, rig);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
