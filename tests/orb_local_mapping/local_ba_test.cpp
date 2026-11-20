// local_bundle_adjustment/local_ba + ba_solver's `local` vs tests/fixtures/orb_slam3/local_ba (README.md,
// Optimizer::LocalBundleAdjustment): the atlas before the BA is rebuilt from the fixture; the window (local and
// fixed keyframes, edge count) must match exactly, and so must the observations erased, the points gone bad
// and their observation counts; poses, points, normals and scale ranges within the noise of a different
// summation order. A point whose reference keyframe lost its observation gets a new reference: the original's
// first observation by KeyFrame* address, the port's by id, so its scale range is not compared.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <unordered_map>

#include "algorithms/ba_solver/local_ba.hpp"
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

std::vector<int> sorted(std::vector<int> v) {
    std::sort(v.begin(), v.end());
    return v;
}

bool sameList(const std::vector<int>& got, const Eigen::MatrixXd& want) {
    if (Eigen::Index(got.size()) != want.rows()) return false;
    for (std::size_t i = 0; i < got.size(); ++i)
        if (got[i] != int(want(Eigen::Index(i)))) return false;
    return true;
}

void runCase(const std::string& name, const fixture::Rig& rig) {
    const fixture::File f("local_ba/" + name + ".txt");
    const int K = int(f.scalar("n_keyframes")), cur = K - 1;
    map::Atlas a;
    const std::vector<map::KeyFrameId> ids = fixture::addKeyFrames(a, f, rig, K);
    a.editCurrentMap().inertial = f.scalar("inertial") != 0;
    auto kfIndex = [&](map::KeyFrameId id) { return int(std::find(ids.begin(), ids.end(), id) - ids.begin()); };
    check(kfIndex(a.currentMap().init_kf_id) == int(f.scalar("init_kf")), name + ": init keyframe");

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

    // Window
    const std::optional<ba::Problem> p = orb_local_mapping::localBAProblem(a, ids[std::size_t(cur)]);
    check(p.has_value(), name + ": a problem");
    if (!p) return;
    std::vector<int> local, fixed;
    int n_fixed = 0;
    // Fixed in the problem: the fixed cameras and an init keyframe among the local ones
    for (const ba::KeyFrameState& k : p->keyframes) {
        const bool init = k.id == a.currentMap().init_kf_id;
        if (!k.fixed || init) local.push_back(kfIndex(k.id));
        else fixed.push_back(kfIndex(k.id));
        if (k.fixed) ++n_fixed;
    }
    check(sameList(sorted(local), f["local"]), name + ": local keyframes");
    check(sameList(sorted(fixed), f["fixed"]), name + ": fixed keyframes");
    check(n_fixed == int(f.scalar("n_fixed")), name + ": fixed count");
    check(int(p->observations.size()) == int(f.scalar("n_edges")), name + ": edges " +
                                                                        std::to_string(p->observations.size()));

    const ba::Result r = ba_solver::solveLocal(*p);
    orb_local_mapping::applyLocalBA(a, p->map, r);

    // Erased observations
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

    // Poses
    double pose_err = 0, rot_err = 0;
    for (int k = 0; k < K; ++k) {
        const geometry::SE3f want = fixture::se3f(f["after_kf" + std::to_string(k) + "_Tcw"]);
        const geometry::SE3f& got = a.keyframe(ids[std::size_t(k)])->pose();
        pose_err = std::max(pose_err, double((got.translation() - want.translation()).norm()));
        rot_err = std::max(rot_err, double(got.unitQuaternion().angularDistance(want.unitQuaternion())));
    }
    check(pose_err < 1e-5 && rot_err < 1e-5, name + ": poses");

    // Points
    const Eigen::MatrixXd& after = f["after_points"];
    std::vector<bool> ref_erased(std::size_t(after.rows()), false);
    for (int row = 0; row < after.rows(); ++row) {
        const int k = int(before(row, 0));
        const Eigen::MatrixXd &b = f["before_kf" + std::to_string(k) + "_points"],
                              &w = f["after_kf" + std::to_string(k) + "_points"];
        ref_erased[std::size_t(row)] = (b.array() == row).any() && !(w.array() == row).any();
    }
    int state_bad = 0, geom_bad = 0, n_bad = 0, n_new_ref = 0;
    double pos_err = 0, normal_err = 0, dist_err = 0;
    for (int row = 0; row < after.rows(); ++row) {
        const map::MapPoint& q = *a.point(pts[std::size_t(row)]);
        if (int(q.bad) != int(after(row, 0)) || q.n_obs != int(after(row, 1))) ++state_bad;
        n_bad += q.bad;
        const double e_pos = (q.pos.cast<double>() - after.block<1, 3>(row, 2).transpose()).norm();
        pos_err = std::max(pos_err, e_pos);
        double e = e_pos;
        if (!q.bad) {
            const double e_n = (q.normal.cast<double>() - after.block<1, 3>(row, 5).transpose()).norm();
            double e_d = 0;
            if (ref_erased[std::size_t(row)])
                ++n_new_ref;
            else
                e_d = std::max(std::abs(q.min_distance - after(row, 8)) / std::max(1.0, after(row, 8)),
                               std::abs(q.max_distance - after(row, 9)) / std::max(1.0, after(row, 9)));
            normal_err = std::max(normal_err, e_n);
            dist_err = std::max(dist_err, e_d);
            e = std::max({e, e_n, e_d});
        }
        if (e > 1e-4) ++geom_bad;
    }
    check(state_bad == 0, name + ": " + std::to_string(state_bad) + " points with another bad flag / count");
    check(geom_bad == 0, name + ": " + std::to_string(geom_bad) + " points with another position / normal / range");
    std::printf("%s: %zu local+fixed keyframes, %zu points, %zu edges, %d iterations, %d erased, %d bad, %d new "
                "references; max error: pose %.2e m %.2e rad, point %.2e m, normal %.2e, range %.2e\n",
                name.c_str(), p->keyframes.size(), p->points.size(), p->observations.size(), r.iterations, erased,
                n_bad, n_new_ref, pose_err, rot_err, pos_err, normal_err, dist_err);
}

}  // namespace

int main() {
    const fixture::Rig rig;
    for (const char* c : {"stereo", "inertial"}) runCase(c, rig);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
