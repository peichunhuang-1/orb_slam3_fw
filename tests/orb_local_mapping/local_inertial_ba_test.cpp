// local_bundle_adjustment/local_ba (localInertialBAProblem, applyLocalBA) + ba_solver's `local_inertial` vs
// tests/fixtures/orb_slam3/local_inertial_ba (README.md, Optimizer::LocalInertialBA): the atlas before the BA is
// rebuilt from the fixture; the window (optimized and fixed keyframes, local points, edge count) and the
// convergence verdict must match exactly, and so must the observations erased, the points gone bad and their
// observation counts; poses, velocities, biases (the keyframes' and their preintegrations' updated ones), points,
// normals and scale ranges within the noise of a different summation order (4-10 LM iterations stop short of
// convergence, so that noise grows; a point's depth error grows as its distance squared, so the points' tolerance
// is 3e-5·max(10, d²) m at d m from the current keyframe (points with two or three observations come closest), over d for normals and scale ranges). A point whose reference keyframe
// lost its observation gets a new reference (by KeyFrame* address in the original, by id here), so its scale
// range is not compared.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <string>
#include <unordered_map>

#include "algorithms/ba_solver/local_inertial.hpp"
#include "algorithms/orb_local_mapping/local_bundle_adjustment/local_ba.hpp"
#include "tests/orb_local_mapping/fixture_keyframes.hpp"

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

bool sameList(std::vector<int> got, const Eigen::MatrixXd& want) {
    std::sort(got.begin(), got.end());
    if (Eigen::Index(got.size()) != want.size()) return false;
    for (std::size_t i = 0; i < got.size(); ++i)
        if (got[i] != int(want(Eigen::Index(i)))) return false;
    return true;
}

imu::Noise noise() {
    const fixture::File r("rig.txt");
    const Eigen::MatrixXd& d = r["imu_noise_density"];
    // discretized at the rig's rate for every step, as the original (not at each step's dt as the pipeline)
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
    const fixture::File f("local_inertial_ba/" + name + ".txt");
    const imu::Noise n = noise();
    const int K = int(f.scalar("n_keyframes")), cur = K - 1;
    const bool large = f.scalar("large") != 0, rec_init = f.scalar("rec_init") != 0;

    map::Atlas a;
    const std::vector<map::KeyFrameId> ids = fixture::addKeyFrames(a, f, rig, K);
    {
        map::Map& M = a.editCurrentMap();
        M.inertial = true;
        M.imu_initialized = true;
        M.imu_ba1 = true;
        M.imu_ba2 = !rec_init;
    }
    for (int k = 0; k < K; ++k) {
        const std::string p = "kf" + std::to_string(k) + "_";
        map::KeyFrame* kf = a.editKeyFrame(ids[std::size_t(k)]);
        auto d = std::make_shared<map::KeyFrameData>(*kf->data);
        d->has_imu_calib = true;
        d->T_cb = rig.T_bc.inverse();
        kf->data = std::move(d);
        kf->setPose(kf->pose());
        auto pre = std::make_shared<imu::Preintegrated>(biasOf(f[p + "lin_bias"]), n);
        const Eigen::MatrixXd& m = f[p + "meas"];
        for (int i = 0; i < m.rows(); ++i)
            pre->integrate(Eigen::Vector3f(float(m(i, 0)), float(m(i, 1)), float(m(i, 2))),
                           Eigen::Vector3f(float(m(i, 3)), float(m(i, 4)), float(m(i, 5))), float(m(i, 6)));
        kf->preintegrated = std::move(pre);
        kf->setNewBias(biasOf(f[p + "bias"]));
        kf->setVelocity(f.f<3, 1>(p + "velocity"));
        kf->imu = true;
    }
    auto kfIndex = [&](map::KeyFrameId id) { return int(std::find(ids.begin(), ids.end(), id) - ids.begin()); };

    const Eigen::MatrixXd& before = f["before_points"];
    std::vector<map::MapPointId> pts;
    std::unordered_map<map::MapPointId, int> row_of;
    for (int r = 0; r < before.rows(); ++r) {
        const map::MapPointId p = map::createMapPoint(a, before.block<1, 3>(r, 1).transpose().cast<float>(),
                                                      ids[std::size_t(before(r, 0))]);
        pts.push_back(p);
        row_of[p] = r;
    }
    for (int k = 0; k < K; ++k) {
        const Eigen::MatrixXd& kp = f["before_kf" + std::to_string(k) + "_points"];
        for (int i = 0; i < kp.rows(); ++i) {
            if (kp(i) < 0) continue;
            const map::MapPointId p = pts[std::size_t(kp(i))];
            map::addObservation(a, p, ids[std::size_t(k)], i);
            a.editKeyFrame(ids[std::size_t(k)])->points[std::size_t(i)] = p;
        }
    }
    for (const map::MapPointId p : pts) {
        map::computeDistinctiveDescriptors(a, p);
        map::updateNormalAndDepth(a, p);
        map::insertMapPoint(a, p);
    }
    for (map::KeyFrameId id : ids) map::updateConnections(a, id);

    // ---- window
    const std::optional<ba::Problem> p = orb_local_mapping::localInertialBAProblem(a, ids[std::size_t(cur)], large, rec_init);
    check(p.has_value(), name + ": a problem");
    if (!p) return;
    std::vector<int> opt, fixed, local_points;
    std::set<int> in_problem;
    for (const ba::KeyFrameState& k : p->keyframes) {
        (k.fixed ? fixed : opt).push_back(kfIndex(k.id));
        in_problem.insert(kfIndex(k.id));
    }
    for (const ba::PointState& q : p->points) local_points.push_back(row_of.at(q.id));
    check(sameList(opt, f["optimized"]), name + ": optimized keyframes");
    check(sameList(fixed, f["fixed"]), name + ": fixed keyframes");
    check(sameList(local_points, f["local_points"]), name + ": local points");
    int close_bad = 0;
    for (const ba::PointState& q : p->points) close_bad += q.close != (before(row_of.at(q.id), 4) < 10.0);
    check(close_bad == 0, name + ": " + std::to_string(close_bad) + " points with another close flag");
    // Edges: every feature of a problem keyframe that sees a local point
    {
        std::set<int> local(local_points.begin(), local_points.end());
        int edges = 0;
        for (const int k : in_problem) {
            const Eigen::MatrixXd& kp = f["before_kf" + std::to_string(k) + "_points"];
            for (int i = 0; i < kp.rows(); ++i) edges += kp(i) >= 0 && local.count(int(kp(i)));
        }
        check(int(p->observations.size()) == edges, name + ": edges " + std::to_string(p->observations.size()) +
                                                         " vs " + std::to_string(edges));
    }

    // ---- solve, write back
    const ba::Result r = ba_solver::solveLocalInertial(*p);
    check(r.ok == (f.scalar("ok") != 0), name + ": convergence verdict");
    if (r.ok) orb_local_mapping::applyLocalBA(a, p->map, r);
    // FAIL LOCAL-INERTIAL BA: no fixture hits it (LM only takes steps that lower the cost, so only a NaN can);
    // one NaN measurement makes the error NaN, which fails unless bLarge
    if (!p->observations.empty()) {
        ba::Problem bad = *p;
        bad.observations[0].uv.x() = std::numeric_limits<float>::quiet_NaN();
        const ba::Result rb = ba_solver::solveLocalInertial(bad);
        check(rb.ok == large, name + ": a NaN error " + (large ? "passes (bLarge)" : "fails"));
        check(rb.ok || (rb.keyframes.empty() && rb.points.empty() && rb.outliers.empty()),
              name + ": a failed result carries nothing to write back");
    }

    int slots_bad = 0, erased = 0;
    for (int k = 0; k < K; ++k) {
        const Eigen::MatrixXd& want = f["after_kf" + std::to_string(k) + "_points"];
        const Eigen::MatrixXd& was = f["before_kf" + std::to_string(k) + "_points"];
        const map::KeyFrame& kf = *a.keyframe(ids[std::size_t(k)]);
        for (int i = 0; i < want.rows(); ++i) {
            const map::MapPointId q = kf.points[std::size_t(i)];
            const int got = q == map::kNone ? -1 : row_of.at(q);
            if (got != int(want(i))) ++slots_bad;
            erased += was(i) >= 0 && want(i) < 0;
        }
    }
    check(slots_bad == 0, name + ": " + std::to_string(slots_bad) + " keyframe features with another point");

    double pose_err = 0, rot_err = 0, vel_err = 0, bias_err = 0, pre_err = 0;
    for (int k = 0; k < K; ++k) {
        const std::string pf = "after_kf" + std::to_string(k) + "_";
        const map::KeyFrame& kf = *a.keyframe(ids[std::size_t(k)]);
        const geometry::SE3f want = fixture::se3f(f[pf + "Tcw"]);
        pose_err = std::max(pose_err, double((kf.pose().translation() - want.translation()).norm()));
        rot_err = std::max(rot_err, double(kf.pose().unitQuaternion().angularDistance(want.unitQuaternion())));
        vel_err = std::max(vel_err, double((kf.velocity() - f.f<3, 1>(pf + "velocity")).norm()));
        bias_err = std::max(bias_err, biasErr(kf.bias, f[pf + "bias"]));
        pre_err = std::max(pre_err, biasErr(kf.preintegrated->updatedBias(), f[pf + "preint_bias"]));
    }
    near(pose_err, 1e-5, name + ": positions");
    near(rot_err, 1e-5, name + ": rotations");
    near(vel_err, 5e-5, name + ": velocities");
    near(bias_err, 1e-5, name + ": biases");
    near(pre_err, 1e-5, name + ": preintegration biases");

    const Eigen::MatrixXd& after = f["after_points"];
    const Eigen::Vector3d cur_centre = fixture::se3f(f["after_kf" + std::to_string(cur) + "_Tcw"]).inverse().translation().cast<double>();
    int state_bad = 0, geom_bad = 0, n_bad = 0, n_new_ref = 0;
    double pos_err = 0, normal_err = 0, dist_err = 0;
    for (int row = 0; row < after.rows(); ++row) {
        const int k = int(before(row, 0));
        const Eigen::MatrixXd &b = f["before_kf" + std::to_string(k) + "_points"],
                              &w = f["after_kf" + std::to_string(k) + "_points"];
        const bool ref_erased = (b.array() == row).any() && !(w.array() == row).any();
        const map::MapPoint& q = *a.point(pts[std::size_t(row)]);
        if (int(q.bad) != int(after(row, 0)) || q.n_obs != int(after(row, 1))) ++state_bad;
        n_bad += q.bad;
        // Tolerance: depth is ill-conditioned (its error grows as d²); relative to 3e-5·max(10, d²) from here on
        const double d = (after.block<1, 3>(row, 2).transpose() - cur_centre).norm();
        const double tol = 3e-5 * std::max(10.0, d * d);
        const double e_pos = (q.pos.cast<double>() - after.block<1, 3>(row, 2).transpose()).norm() / tol;
        pos_err = std::max(pos_err, e_pos);
        double e = e_pos;
        if (!q.bad) {
            const double e_n = (q.normal.cast<double>() - after.block<1, 3>(row, 5).transpose()).norm() /
                               (tol / std::max(1.0, d));
            double e_d = 0;
            if (ref_erased)
                ++n_new_ref;
            else
                e_d = std::max(std::abs(q.min_distance - after(row, 8)) / std::max(1.0, after(row, 8)),
                               std::abs(q.max_distance - after(row, 9)) / std::max(1.0, after(row, 9))) /
                      (tol / std::max(1.0, d));
            normal_err = std::max(normal_err, e_n);
            dist_err = std::max(dist_err, e_d);
            e = std::max({e, e_n, e_d});
        }
        if (e > 1) ++geom_bad;
    }
    check(state_bad == 0, name + ": " + std::to_string(state_bad) + " points with another bad flag / count");
    check(geom_bad == 0, name + ": " + std::to_string(geom_bad) + " points with another position / normal / range");
    std::printf("%s: %zu optimized, %zu fixed, %zu points, %zu edges, %d iterations, ok %d, %d erased, %d bad, %d "
                "new references; max error: pose %.2e m %.2e rad, velocity %.2e, bias %.2e (preint %.2e); points, "
                "relative to their tolerance: position %.2f, normal %.2f, range %.2f\n",
                name.c_str(), opt.size(), fixed.size(), p->points.size(), p->observations.size(), r.iterations,
                int(r.ok), erased, n_bad, n_new_ref, pose_err, rot_err, vel_err, bias_err, pre_err, pos_err,
                normal_err, dist_err);
}

}  // namespace

int main() {
    const fixture::Rig rig;
    for (const char* c : {"large", "small", "rec_init"}) runCase(c, rig);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
