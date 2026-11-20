// create_new_map_points/triangulation vs tests/fixtures/orb_slam3/new_map_points (README.md): the covisibility
// order, SearchForTriangulation with each other keyframe (bCoarse false and true), and CreateNewMapPoints' new
// points in order (keyframes and feature indices exactly, positions within float noise).
#include <cmath>
#include <cstdio>
#include <string>

#include "algorithms/orb_local_mapping/create_new_map_points/triangulation.hpp"
#include "tests/orb_local_mapping/fixture_keyframes.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

void runCase(const std::string& name, const fixture::Rig& rig) {
    const fixture::File f("new_map_points/" + name + ".txt");
    const int K = int(f.scalar("n_keyframes")), cur = K - 1;
    const bool inertial = f.scalar("inertial") != 0;

    map::Atlas a;
    const std::vector<map::KeyFrameId> ids = fixture::addKeyFrames(a, f, rig, K);
    if (f.scalar("imu_ba2") != 0) a.editCurrentMap().imu_ba2 = true;

    // The tracked points, made from the current keyframe, observed where the fixture has them
    const Eigen::MatrixXd& tw = f["tracked_world"];
    std::vector<map::MapPointId> tracked;
    for (int i = 0; i < tw.rows(); ++i)
        tracked.push_back(map::createMapPoint(a, tw.row(i).transpose().cast<float>(), ids[std::size_t(cur)]));
    for (int k = 0; k < K; ++k) {
        const Eigen::MatrixXd& pts = f["kf" + std::to_string(k) + "_points"];
        for (int i = 0; i < pts.rows(); ++i) {
            const int t = int(pts(i));
            if (t < 0) continue;
            map::addObservation(a, tracked[std::size_t(t)], ids[std::size_t(k)], i);
            a.editKeyFrame(ids[std::size_t(k)])->points[std::size_t(i)] = tracked[std::size_t(t)];
        }
    }
    for (map::MapPointId p : tracked) map::insertMapPoint(a, p);
    for (map::KeyFrameId id : ids) map::updateConnections(a, id);

    // Covisibility order
    const map::KeyFrame& kc = *a.keyframe(ids[std::size_t(cur)]);
    const Eigen::MatrixXd& cov = f["covisibles"];
    const std::vector<map::KeyFrameId> best = kc.bestCovisibles(10);
    bool cov_ok = Eigen::Index(best.size()) == cov.rows();
    for (std::size_t i = 0; cov_ok && i < best.size(); ++i) cov_ok = best[i] == ids[std::size_t(cov(Eigen::Index(i)))];
    check(cov_ok, name + ": covisibility order");

    // SearchForTriangulation
    for (int k = 0; k < cur; ++k)
        for (int coarse = 0; coarse < 2; ++coarse) {
            const std::string key = "search_kf" + std::to_string(k) + (coarse ? "_coarse" : "");
            const Eigen::MatrixXd& want = f[key];
            std::vector<std::pair<int, int>> pairs;
            orb_local_mapping::searchForTriangulation(a, kc, *a.keyframe(ids[std::size_t(k)]), coarse != 0, pairs);
            bool ok = Eigen::Index(pairs.size()) == want.rows();
            int first_bad = -1;
            for (std::size_t i = 0; i < pairs.size() && i < std::size_t(want.rows()); ++i)
                if (pairs[i].first != int(want(Eigen::Index(i), 0)) || pairs[i].second != int(want(Eigen::Index(i), 1))) {
                    ok = false;
                    if (first_bad < 0) first_bad = int(i);
                }
            if (!ok)
                std::printf("  %s %s: %zu pairs, want %ld (first difference at %d)\n", name.c_str(), key.c_str(),
                            pairs.size(), long(want.rows()), first_bad);
            check(ok, name + ": " + key);
        }

    // CreateNewMapPoints
    local_mapping::LocalMapper m;
    m.inertial = inertial;
    m.current = ids[std::size_t(cur)];
    m.far_points = f.scalar("far_points") != 0;
    m.th_far_points = float(f.scalar("th_far_points"));
    const int n = orb_local_mapping::createNewMapPoints(a, m, f.scalar("recently_lost") != 0);
    const Eigen::MatrixXd& want = f["created"];
    check(n == want.rows() && Eigen::Index(m.recent_points.size()) == want.rows(),
          name + ": " + std::to_string(n) + " points created, want " + std::to_string(want.rows()));
    int row = 0, idx_bad = 0, pos_bad = 0;
    double max_err = 0;
    for (map::MapPointId pid : m.recent_points) {
        if (row >= want.rows()) break;
        const map::MapPoint& p = *a.point(pid);
        const map::KeyFrameId other = ids[std::size_t(want(row, 0))];
        const int idx1 = int(want(row, 1)), idx2 = int(want(row, 2));
        auto at = [&](map::KeyFrameId k) {
            const map::Observation o = p.indexIn(k);
            return o.left >= 0 ? o.left : o.right;
        };
        if (p.observations.size() != 2 || at(m.current) != idx1 || at(other) != idx2) ++idx_bad;
        const Eigen::Vector3d w = want.block<1, 3>(row, 3).transpose();
        const double err = (p.pos.cast<double>() - w).norm() / std::max(1.0, w.norm());
        max_err = std::max(max_err, err);
        if (err > 1e-4) ++pos_bad;
        if (p.first_kf != m.current || !p.in_map || p.n_obs != 2) ++idx_bad;
        ++row;
    }
    check(idx_bad == 0, name + ": " + std::to_string(idx_bad) + " created points with other keyframes / indices");
    check(pos_bad == 0, name + ": " + std::to_string(pos_bad) + " created points off (relative > 1e-4)");
    std::printf("%s: %d created, position max relative error %.2e\n", name.c_str(), n, max_err);
}

}  // namespace

int main() {
    const fixture::Rig rig;
    for (const char* c : {"stereo", "inertial", "inertial_coarse", "far_points"}) runCase(c, rig);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
