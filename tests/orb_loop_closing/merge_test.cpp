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

void runCase(const std::string& name, const fixture::Rig& rig) {
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
    check(a.keyframe(s.kfs[std::size_t(KA)])->parent == map::kNone, name + ": pass B's root");

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
    check(pb.has_value() == bias_case, name + ": the bias optimization when BA2 is missing");
    if (pb) {
        check(pb->kind == ba::Kind::inertial_init && pb->fixed_gravity && pb->iterations == 200, name + ": bias problem");
        const ba::Result rb = ba_solver::solveInertialInit(*pb);
        orb_loop_closing::applyMergeBias(a, m, rb);
        std::printf("%s: bias optimization %d iterations\n", name.c_str(), rb.iterations);
    }
    check(m.imu_update_seq == (bias_case ? 2u : 1u), name + ": UpdateFrameIMU");
    const std::optional<ba::Problem> p = orb_loop_closing::joinMaps(a, m);
    check(p.has_value(), name + ": MergeInertialBA's problem");
    if (!p) return;
    int n_opt = 0, n_fixed = 0;
    for (const ba::KeyFrameState& k : p->keyframes) (k.fixed ? n_fixed : n_opt)++;
    std::printf("%s: MergeInertialBA %d keyframes optimized, %d fixed, %zu points, %zu observations\n", name.c_str(),
                n_opt, n_fixed, p->points.size(), p->observations.size());
    const ba::Result r = ba_solver::solveMergeInertial(*p);
    check(r.ok, name + ": MergeInertialBA finite");
    orb_loop_closing::applyMergeInertialBA(a, mapB, r);
    check(a.findMap(mapB)->change_index - change0 == int(f.scalar("after_change_index")), name + ": change index");
    check(a.findMap(mapB)->imu_ba2 == (f.scalar("after_ba2") != 0), name + ": BA2");
    check(a.findMap(mapA) && a.findMap(mapA)->keyframes.empty() && a.findMap(mapA)->n_points == 0,
          name + ": the merge map left empty");

    // mvpMergeConnectedKFs
    {
        const Eigen::MatrixXd& want = f["after_merge_connected"];
        bool same = Eigen::Index(m.merge_connected_kfs.size()) == want.size();
        for (Eigen::Index i = 0; same && i < want.size(); ++i)
            same = s.kfIndex(m.merge_connected_kfs[std::size_t(i)]) == int(want(i));
        check(same, name + ": mvpMergeConnectedKFs");
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
        check(s.kfIndex(kf.parent) == int(par(k)), name + ": kf" + std::to_string(k) + " parent");
        check(kf.map == (int(kmap(k)) == 0 ? mapA : mapB), name + ": kf" + std::to_string(k) + " map");
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
                "bias %.2g\n", name.c_str(), rot, trans, verr, berr, bmoved, perr);
    check(point_diff == 0, name + ": keyframes' points (" + std::to_string(point_diff) + " differ)");
    near(rot, 1e-5 * 180 / M_PI, name + ": keyframe rotations (deg)");
    near(trans, 1e-5, name + ": keyframe centres (m)");
    near(verr, 5e-5, name + ": velocities (m/s)");
    near(berr, 1e-3 * bmoved, name + ": biases");
    near(perr, 1e-3 * bmoved, name + ": preintegration biases");

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
    std::printf("%s: points %.2g of their tolerance\n", name.c_str(), pos);
    check(bad_diff == 0, name + ": points' bad flags (" + std::to_string(bad_diff) + " differ)");
    check(map_diff == 0, name + ": points' maps (" + std::to_string(map_diff) + " differ)");
    near(pos, 1, name + ": point positions (of their tolerance)");
}

}  // namespace

int main() {
    const fixture::Rig rig;
    for (const char* c : {"merge_inertial", "merge_inertial_bias"}) runCase(c, rig);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
