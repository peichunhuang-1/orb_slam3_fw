// orb_loop_closing's merge_maps (MergeLocal2) vs tests/fixtures/orb_slam3/loop/merge_inertial*.txt (README.md):
// the original's MergeLocal2 of pass B's map into pass A's, both stereo-inertial. The atlas is rebuilt from the
// fixture (the inertial state, the spanning tree per pass, the points the current keyframe's last local BA took);
// then alignMergeInertial, ba_solver's inertial_init kind and applyMergeBias when the current map has no BA2
// ("merge_inertial_bias"), joinMaps, the merge_inertial kind, applyMergeInertialBA. Compared with the original's
// maps after:
//   which map each keyframe and point is in, the spanning tree, mvpMergeConnectedKFs   exactly;
//   the keyframes' points (SearchAndFuse), the points' bad flags                          exactly;
//   poses, velocities, biases (the keyframes' and their preintegrations'), positions      within the float noise
//                                                                                         of two LM solvers
//                                                                                         taking the same steps.
// The tolerances are local_inertial_ba_test's (8 LM iterations stop short of convergence, so that noise grows),
// the biases' relative to how far the BA moved them (1e-3 of it: the current window's chain shares one offset).
// That with joinMaps(original_points): the original's MergeInertialBA leaves out the points the current
// keyframe's last local BA took. Then again as this code runs (those points taken): the same up to the BA (maps,
// tree, mvpMergeConnectedKFs, SearchAndFuse), the BA's problem has them, and after it the current side's
// keyframes agree with their own points as well as with the fused ones (the reprojection errors, left image,
// of the pass-B keyframes the BA moved; printed for both).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "algorithms/ba_solver/inertial.hpp"
#include "algorithms/ba_solver/merge_inertial.hpp"
#include "algorithms/orb_loop_closing/merge_maps/merge.hpp"
#include "tests/orb_loop_closing/loop_scene.hpp"
#include "types/camera/stereo_rig.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

void near(double err, double tol, const std::string& what) {
    if (!(err <= tol)) {
        ++failures;
        std::printf("FAIL %s: %.3g > %.3g\n", what.c_str(), err, tol);
    }
}

double angleDeg(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
    return Eigen::AngleAxisd(A.transpose() * B).angle() * 180.0 / M_PI;
}

imu::Noise noise() {
    const fixture::File r("rig.txt");
    const Eigen::MatrixXd& d = r["imu_noise_density"];
    return imu::Noise::fromDensities(float(d(0)), float(d(1)), float(d(2)), float(d(3))).atFixedRate(r.scalar("imu_hz"));
}

imu::Bias biasOf(const Eigen::MatrixXd& v) {
    imu::Bias b;
    b.g = Eigen::Vector3f(float(v(0)), float(v(1)), float(v(2)));
    b.a = Eigen::Vector3f(float(v(3)), float(v(4)), float(v(5)));
    return b;
}

double biasErr(const imu::Bias& b, const Eigen::MatrixXd& v) {
    return std::max((b.g - biasOf(v).g).norm(), (b.a - biasOf(v).a).norm());
}

double median(std::vector<double> v) {
    if (v.empty()) return 0;
    std::nth_element(v.begin(), v.begin() + long(v.size() / 2), v.end());
    return v[v.size() / 2];
}

// The median reprojection errors (px, left image) of the BA-optimized pass-B keyframes' left observations: of
// the points fused from pass A, of pass B's own
struct Reproj {
    double fused = 0, own = 0;
};

Reproj reproj_after[2];

Reproj reprojection(const map::Atlas& a, const fixture::Scene& s, const ba::Problem& p, int KA,
                    const std::vector<bool>& from_a) {
    std::vector<double> fused, own;
    const camera::KannalaBrandt8& cam = a.rig(0).camera(0);
    for (const ba::KeyFrameState& k : p.keyframes) {
        if (k.fixed || s.kfIndex(k.id) < KA) continue;
        const map::KeyFrame& kf = *a.keyframe(k.id);
        const orb::FrameFeatures& f = *kf.data->features;
        for (std::size_t i = 0; i < kf.points.size(); ++i) {
            const map::MapPointId pid = kf.points[i];
            if (pid == map::kNone || !a.goodPoint(pid)) continue;
            const auto it = a.point(pid)->observations.find(k.id);
            if (it == a.point(pid)->observations.end() || it->second.left != int(i)) continue;
            Eigen::Vector2f uv;
            if (!cam.project(kf.pose() * a.point(pid)->pos, uv)) continue;
            const cv::KeyPoint& kp = f.keypoint(int(i));
            const double e = (uv - Eigen::Vector2f(kp.pt.x, kp.pt.y)).norm();
            (from_a[std::size_t(s.ptIndex(pid))] ? fused : own).push_back(e);
        }
    }
    return {median(fused), median(own)};
}

void runCase(const std::string& name, const fixture::Rig& rig, bool original) {
    const std::string tag = name + (original ? " (original)" : " (current side taken)");
    const fixture::File f("loop/" + name + ".txt");
    fixture::Scene s;
    fixture::load(s, f, rig);
    map::Atlas& a = s.a;
    const int K = int(f.scalar("n_keyframes")), KA = int(f.scalar("pass_a"));
    const map::MapId mapA = a.keyframe(s.kfs[0])->map, mapB = a.keyframe(s.kfs[std::size_t(K - 1)])->map;
    for (const map::MapId id : {mapA, mapB}) {
        map::Map* M = a.editMap(id);
        M->inertial = true;
        M->imu_initialized = true;
        M->imu_ba1 = true;
        M->imu_ba2 = id == mapA || f.scalar("ba2_current") != 0;
    }
    const imu::Noise n = noise();
    for (int k = 0; k < K; ++k) {
        const std::string p = "kf" + std::to_string(k) + "_";
        map::KeyFrame* kf = a.editKeyFrame(s.kfs[std::size_t(k)]);
        auto pre = std::make_shared<imu::Preintegrated>(biasOf(f[p + "lin_bias"]), n);
        const Eigen::MatrixXd& m = f[p + "meas"];
        for (int i = 0; i < m.rows(); ++i)
            pre->integrate(Eigen::Vector3f(float(m(i, 0)), float(m(i, 1)), float(m(i, 2))),
                           Eigen::Vector3f(float(m(i, 3)), float(m(i, 4)), float(m(i, 5))), float(m(i, 6)));
        kf->preintegrated = std::move(pre);
        kf->setNewBias(biasOf(f[p + "bias"]));
        kf->imu = true;
    }
    // The spanning tree per pass: B's first keyframe is its map's root
    check(a.keyframe(s.kfs[std::size_t(KA)])->parent == map::kNone, tag + ": pass B's root");

    local_mapping::LocalMapper m;
    m.inertial = true;
    const map::KeyFrameId cur = s.kfs[std::size_t(K - 1)];
    for (const map::MapPointId p : s.points(f["lba_points"])) {
        if (p >= m.point_scratch.size()) m.point_scratch.resize(std::size_t(p) + 1);
        m.point_scratch[std::size_t(p)].ba_local_for = cur;
    }
    m.current = cur;
    m.merge.pending = true;
    m.merge.keyframe = cur;
    m.merge.matched_kf = s.kfs[std::size_t(f.scalar("merge_matched"))];
    m.merge.T_old_new = fixture::se3f(f["merge_T_old_new"]).cast<double>();
    m.merge.last_kf = s.kfs[std::size_t(f.scalar("merge_last_kf"))];
    const int change0 = a.findMap(mapB)->change_index;

    const std::optional<ba::Problem> pb = orb_loop_closing::alignMergeInertial(a, m);
    const bool bias_case = f.scalar("ba2_current") == 0;
    check(pb.has_value() == bias_case, tag + ": the bias optimization when BA2 is missing");
    if (pb) {
        check(pb->kind == ba::Kind::inertial_init && pb->fixed_gravity && pb->iterations == 200, tag + ": bias problem");
        const ba::Result rb = ba_solver::solveInertialInit(*pb);
        orb_loop_closing::applyMergeBias(a, m, rb);
        std::printf("%s: bias optimization %d iterations\n", tag.c_str(), rb.iterations);
    }
    check(m.imu_update_seq == (bias_case ? 2u : 1u), tag + ": UpdateFrameIMU");
    const std::optional<ba::Problem> p = orb_loop_closing::joinMaps(a, m, original);
    check(p.has_value(), tag + ": MergeInertialBA's problem");
    if (!p) return;
    int n_opt = 0, n_fixed = 0;
    for (const ba::KeyFrameState& k : p->keyframes) (k.fixed ? n_fixed : n_opt)++;
    std::printf("%s: MergeInertialBA %d keyframes optimized, %d fixed, %zu points, %zu observations\n", tag.c_str(),
                n_opt, n_fixed, p->points.size(), p->observations.size());
    {
        // The last local BA's points seen by an optimized keyframe: in the problem unless original
        std::set<map::MapPointId> in, seen;
        for (const ba::PointState& q : p->points) in.insert(q.id);
        for (const ba::KeyFrameState& k : p->keyframes)
            if (!k.fixed && s.kfIndex(k.id) >= 0)
                for (const map::MapPointId q : a.keyframe(k.id)->points)
                    if (q != map::kNone && a.goodPoint(q)) seen.insert(q);
        int lba_in = 0, lba_all = 0;
        for (const map::MapPointId q : s.points(f["lba_points"]))
            if (seen.count(q)) {
                ++lba_all;
                lba_in += int(in.count(q));
            }
        std::printf("%s: %d of the last local BA's %d points (seen by an optimized keyframe) in the problem\n",
                    tag.c_str(), lba_in, lba_all);
        check(lba_all > 0 && lba_in == (original ? 0 : lba_all), tag + ": the last local BA's points");
    }
    std::vector<bool> from_a;
    for (int i = 0; i < f["points"].rows(); ++i) from_a.push_back(int(f["points"](i, 0)) < KA);
    const Reproj before = reprojection(a, s, *p, KA, from_a);
    const ba::Result r = ba_solver::solveMergeInertial(*p);
    check(r.ok, tag + ": MergeInertialBA finite");
    orb_loop_closing::applyMergeInertialBA(a, mapB, r);
    const Reproj after = reprojection(a, s, *p, KA, from_a);
    std::printf("%s: current side's reprojection (median px) fused %.2f -> %.2f, own %.2f -> %.2f\n", tag.c_str(),
                before.fused, after.fused, before.own, after.own);
    reproj_after[original] = after;
    check(a.findMap(mapB)->change_index - change0 == int(f.scalar("after_change_index")), tag + ": change index");
    check(a.findMap(mapB)->imu_ba2 == (f.scalar("after_ba2") != 0), tag + ": BA2");
    // The merge map: empty, set bad (the original leaves it among the maps)
    const map::Map* bad = a.bad_maps.find(mapA);
    check(!a.findMap(mapA) && bad && bad->bad && bad->keyframes.empty() && bad->n_points == 0 && a.maps.size() == 1,
          tag + ": the merge map empty and bad");

    // mvpMergeConnectedKFs
    {
        const Eigen::MatrixXd& want = f["after_merge_connected"];
        bool same = Eigen::Index(m.merge_connected_kfs.size()) == want.size();
        for (Eigen::Index i = 0; same && i < want.size(); ++i)
            same = s.kfIndex(m.merge_connected_kfs[std::size_t(i)]) == int(want(i));
        check(same, tag + ": mvpMergeConnectedKFs");
    }

    // Keyframes
    const Eigen::MatrixXd& par = f["after_parents"];
    const Eigen::MatrixXd& kmap = f["after_map"];
    const Eigen::MatrixXd& vel = f["after_velocity"];
    double rot = 0, trans = 0, verr = 0, berr = 0, perr = 0, bmoved = 0;
    int point_diff = 0;
    for (int k = 0; k < K; ++k) {
        const map::KeyFrame& kf = *a.keyframe(s.kfs[std::size_t(k)]);
        const std::string pf = "after_k" + std::to_string(k) + "_";
        check(s.kfIndex(kf.parent) == int(par(k)), tag + ": kf" + std::to_string(k) + " parent");
        check(kf.map == (int(kmap(k)) == 0 ? mapA : mapB), tag + ": kf" + std::to_string(k) + " map");
        const geometry::SE3f T = fixture::se3f(f[pf + "Tcw"]);
        rot = std::max(rot, angleDeg(kf.pose().rotationMatrix().cast<double>(), T.rotationMatrix().cast<double>()));
        trans = std::max(trans, double((kf.cameraCenter() - T.inverse().translation()).norm()));
        verr = std::max(verr, double((kf.velocity() - vel.row(k).transpose().cast<float>()).norm()));
        berr = std::max(berr, biasErr(kf.bias, f[pf + "bias"]));
        bmoved = std::max(bmoved, biasErr(biasOf(f[pf + "bias"]), f["kf" + std::to_string(k) + "_bias"]));
        perr = std::max(perr, biasErr(kf.preintegrated->updatedBias(), f[pf + "preint_bias"]));
        const Eigen::MatrixXd& pts = f[pf + "points"];
        for (int i = 0; i < pts.rows(); ++i)
            if (s.ptIndex(kf.points[std::size_t(i)]) != int(pts(i))) ++point_diff;
    }
    std::printf("%s: keyframes %.2g deg, %.2g m, velocity %.2g m/s, bias %.2g (moved up to %.2g), preintegration "
                "bias %.2g\n", tag.c_str(), rot, trans, verr, berr, bmoved, perr);
    if (!original) return;  // the BA's values (and the outliers it erases) are not the original's
    check(point_diff == 0, tag + ": keyframes' points (" + std::to_string(point_diff) + " differ)");
    near(rot, 1e-5 * 180 / M_PI, tag + ": keyframe rotations (deg)");
    near(trans, 1e-5, tag + ": keyframe centres (m)");
    near(verr, 5e-5, tag + ": velocities (m/s)");
    near(berr, 1e-3 * bmoved, tag + ": biases");
    near(perr, 1e-3 * bmoved, tag + ": preintegration biases");

    // Points
    const Eigen::MatrixXd& P = f["after_points"];
    double pos = 0;  // relative to its tolerance, 3e-5·max(10, d²) m at d m from the current keyframe
    int bad_diff = 0, map_diff = 0;
    const Eigen::Vector3f O = a.keyframe(cur)->cameraCenter();
    for (int i = 0; i < P.rows(); ++i) {
        const map::MapPoint& mp = *a.point(s.pts[std::size_t(i)]);
        if (mp.bad != (P(i, 3) != 0)) ++bad_diff;
        if (mp.bad) continue;
        if (mp.map != (int(P(i, 4)) == 0 ? mapA : mapB)) ++map_diff;
        const double d = (mp.pos - O).norm();
        pos = std::max(pos, double((mp.pos - P.block<1, 3>(i, 0).transpose().cast<float>()).norm()) /
                                (3e-5 * std::max(10.0, d * d)));
    }
    std::printf("%s: points %.2g of their tolerance\n", tag.c_str(), pos);
    check(bad_diff == 0, tag + ": points' bad flags (" + std::to_string(bad_diff) + " differ)");
    check(map_diff == 0, tag + ": points' maps (" + std::to_string(map_diff) + " differ)");
    near(pos, 1, tag + ": point positions (of their tolerance)");
}

}  // namespace

int main() {
    const fixture::Rig rig;
    for (const char* c : {"merge_inertial", "merge_inertial_bias"}) {
        runCase(c, rig, true);
        runCase(c, rig, false);
        // The original moves the current side onto the fused points away from its own; taken, both agree
        const Reproj& o = reproj_after[1];
        const Reproj& t = reproj_after[0];
        check(o.own > o.fused * 1.3, std::string(c) + ": the original leaves the current side's own points behind");
        check(t.own < o.own && t.own < t.fused * 1.1, std::string(c) + ": the current side agrees with its own points");
    }
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
