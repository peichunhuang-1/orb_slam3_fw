// search_in_neighbors/fuse vs tests/fixtures/orb_slam3/fuse (README.md): the atlas before SearchInNeighbors is
// rebuilt from the fixture (points with the original's descriptors, normals and scale ranges); the targets, every
// keyframe's points, every point's bad / replaced / observation counts / descriptor and the current keyframe's
// covisibility after it must match exactly, normals and scale ranges within float noise.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <unordered_map>

#include "algorithms/orb_local_mapping/search_in_neighbors/fuse.hpp"
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
    const fixture::File f("fuse/" + name + ".txt");
    const int K = int(f.scalar("n_keyframes")), cur = K - 1;
    map::Atlas a;
    const std::vector<map::KeyFrameId> ids = fixture::addKeyFrames(a, f, rig, K);
    auto kfIndex = [&](map::KeyFrameId id) { return int(std::find(ids.begin(), ids.end(), id) - ids.begin()); };

    // Points as the original had them
    const Eigen::MatrixXd& before = f["before_points"];
    const Eigen::MatrixXd& before_desc = f["before_point_desc"];
    const Eigen::MatrixXd& refs = f["point_ref"];
    std::vector<map::MapPointId> pts;
    std::unordered_map<map::MapPointId, int> row_of;
    for (int r = 0; r < before.rows(); ++r) {
        const map::MapPointId p = map::createMapPoint(a, before.block<1, 3>(r, 5).transpose().cast<float>(),
                                                      ids[std::size_t(refs(r))]);
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
    int obs_bad = 0;
    for (int r = 0; r < before.rows(); ++r) {
        map::insertMapPoint(a, pts[std::size_t(r)]);
        map::MapPoint* p = a.editPoint(pts[std::size_t(r)]);
        if (p->n_obs != int(before(r, 2))) ++obs_bad;
        p->visible = int(before(r, 3));
        p->found = int(before(r, 4));
        p->normal = before.block<1, 3>(r, 8).transpose().cast<float>();
        p->min_distance = float(before(r, 11));
        p->max_distance = float(before(r, 12));
        for (int b = 0; b < 32; ++b) p->descriptor[std::size_t(b)] = std::uint8_t(before_desc(r, b));
    }
    check(obs_bad == 0, name + ": observation counts before");
    for (map::KeyFrameId id : ids) map::updateConnections(a, id);

    auto covisiblesMatch = [&](const std::string& key) {
        const Eigen::MatrixXd& want = f[key];
        const map::KeyFrame& kc = *a.keyframe(ids[std::size_t(cur)]);
        bool ok = Eigen::Index(kc.ordered_connected.size()) == want.rows();
        for (std::size_t i = 0; ok && i < kc.ordered_connected.size(); ++i)
            ok = kfIndex(kc.ordered_connected[i]) == int(want(Eigen::Index(i), 0)) &&
                 kc.ordered_weights[i] == int(want(Eigen::Index(i), 1));
        return ok;
    };
    check(covisiblesMatch("before_covisibles"), name + ": covisibility before");

    // Targets (the original's marks show the set)
    local_mapping::LocalMapper m;
    m.inertial = f.scalar("inertial") != 0;
    m.current = ids[std::size_t(cur)];
    std::vector<int> targets;
    for (map::KeyFrameId id : orb_local_mapping::fuseTargets(a, m.current, m.inertial, false))
        targets.push_back(kfIndex(id));
    std::sort(targets.begin(), targets.end());
    const Eigen::MatrixXd& want_targets = f["targets"];
    bool targets_ok = Eigen::Index(targets.size()) == want_targets.rows();
    for (std::size_t i = 0; targets_ok && i < targets.size(); ++i)
        targets_ok = targets[i] == int(want_targets(Eigen::Index(i)));
    check(targets_ok, name + ": targets");

    orb_local_mapping::searchInNeighbors(a, m);

    // Keyframe points
    int slots_bad = 0;
    for (int k = 0; k < K; ++k) {
        const Eigen::MatrixXd& want = f["after_kf" + std::to_string(k) + "_points"];
        const map::KeyFrame& kf = *a.keyframe(ids[std::size_t(k)]);
        for (int i = 0; i < want.rows(); ++i) {
            const map::MapPointId p = kf.points[std::size_t(i)];
            const int got = p == map::kNone ? -1 : row_of.at(p);
            if (got != int(want(i))) ++slots_bad;
        }
    }
    check(slots_bad == 0, name + ": " + std::to_string(slots_bad) + " keyframe features with another point");

    // Points
    const Eigen::MatrixXd& after = f["after_points"];
    const Eigen::MatrixXd& after_desc = f["after_point_desc"];
    int state_bad = 0, desc_bad = 0, geom_bad = 0, n_replaced = 0;
    double max_err = 0;
    for (int r = 0; r < after.rows(); ++r) {
        const map::MapPoint& p = *a.point(pts[std::size_t(r)]);
        const int replaced = p.replaced == map::kNone ? -1 : row_of.at(p.replaced);
        if (int(p.bad) != int(after(r, 0)) || replaced != int(after(r, 1)) || p.n_obs != int(after(r, 2)) ||
            p.visible != int(after(r, 3)) || p.found != int(after(r, 4)))
            ++state_bad;
        n_replaced += replaced >= 0;
        if (p.bad) continue;  // the original leaves a bad point's derived values as they were
        bool d_ok = true;
        for (int b = 0; b < 32; ++b) d_ok = d_ok && p.descriptor[std::size_t(b)] == std::uint8_t(after_desc(r, b));
        if (!d_ok) ++desc_bad;
        const Eigen::Vector3d n = after.block<1, 3>(r, 8).transpose();
        const double e = std::max({(p.normal.cast<double>() - n).norm(),
                                   std::abs(p.min_distance - after(r, 11)) / std::max(1.0, after(r, 11)),
                                   std::abs(p.max_distance - after(r, 12)) / std::max(1.0, after(r, 12)),
                                   (p.pos.cast<double>() - after.block<1, 3>(r, 5).transpose()).norm()});
        max_err = std::max(max_err, e);
        if (e > 1e-5) ++geom_bad;
    }
    check(state_bad == 0, name + ": " + std::to_string(state_bad) + " points with other bad / replaced / counts");
    check(desc_bad == 0, name + ": " + std::to_string(desc_bad) + " points with another descriptor");
    check(geom_bad == 0, name + ": " + std::to_string(geom_bad) + " points with another normal / scale range");
    check(covisiblesMatch("after_covisibles"), name + ": covisibility after");
    std::printf("%s: %zu targets, %d points replaced, normal / scale max error %.2e\n", name.c_str(), targets.size(),
                n_replaced, max_err);
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
