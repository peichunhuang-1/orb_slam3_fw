// orb_loop_closing's CorrectLoop vs tests/fixtures/orb_slam3/loop/correct*.txt (README.md): the original's
// CorrectLoop on the place recognition scene with the drift growing along the second pass, the loop its
// DetectCommonRegionsFromBoW found. "correct": a visual map (6DoF essential graph); "correct_4dof": inertial
// (4DoF). The atlas is rebuilt from the fixture with the original's spanning tree; then beginCorrectLoop, ba_solver's
// pose graph, endCorrectLoop. "gba": a visual map with the IMU not initialized, so the original's CorrectLoop also
// ran its global BA and wrote it back: then globalBAProblem, the `global` kind solved in steps (as gba_solver),
// applyGlobalBA. Compared with the original's map after:
//   the keyframes' points (the loop fusion and SearchAndFuse)   exactly;
//   the points' bad flags                                       exactly;
//   the keyframes' poses, velocities, the points' positions     within the float rounding of two pose graph
//                                                               solvers taking the same steps;
//   the loop edges                                              exactly.
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "algorithms/ba_solver/global_ba.hpp"
#include "algorithms/ba_solver/pose_graph.hpp"
#include "algorithms/orb_loop_closing/correct_loop/correction.hpp"
#include "algorithms/orb_loop_closing/global_ba/global_ba_ops.hpp"
#include "tests/orb_loop_closing/loop_scene.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

double angleDeg(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
    return Eigen::AngleAxisd(A.transpose() * B).angle() * 180.0 / M_PI;
}

void runCase(const std::string& name, const fixture::Rig& rig) {
    const fixture::File f("loop/" + name + ".txt");
    fixture::Scene s;
    fixture::load(s, f, rig);
    map::Atlas& a = s.a;
    const int K = int(f.scalar("n_keyframes"));

    local_mapping::PendingLoop loop;
    loop.pending = true;
    loop.keyframe = s.kfs[std::size_t(K - 1)];
    loop.matched_kf = s.kfs[std::size_t(f.scalar("loop_matched"))];
    loop.Tcw = fixture::se3f(f["loop_Tcw"]).cast<double>();
    loop.matched_points = s.points(f["loop_matched_points"]);
    loop.region_points = s.points(f["loop_points"]);

    const std::optional<ba::Problem> p = orb_loop_closing::beginCorrectLoop(a, loop);
    check(p.has_value(), name + ": beginCorrectLoop");
    if (!p) return;
    const bool four_dof = f.scalar("inertial") != 0;
    check((p->kind == ba::Kind::pose_graph_4dof) == four_dof, name + ": the pose graph's kind");
    const ba::Result r = four_dof ? ba_solver::solvePoseGraph4DoF(*p) : ba_solver::solvePoseGraph(*p);
    orb_loop_closing::endCorrectLoop(a, loop, r);
    double graph_moved = 0;  // what the essential graph changed
    for (std::size_t i = 0; i < r.poses.size(); ++i)
        graph_moved = std::max(graph_moved, (r.poses[i].inverse().translation() -
                                             p->pose_graph[i].Tcw.inverse().translation()).norm());

    // The global BA the original started (RunGlobalBundleAdjustment)
    double gba_moved = 0;
    int gba_iterations = 0;
    if (f.has("gba_kf")) {
        const map::MapId M = a.keyframe(loop.keyframe)->map;
        ba::Problem gp = orb_loop_closing::globalBAProblem(a, M);
        check(gp.kind == ba::Kind::global && gp.iterations == 10, name + ": the visual global BA");
        int fixed = 0;
        for (const ba::KeyFrameState& k : gp.keyframes) fixed += k.fixed;
        check(fixed == 1 && gp.keyframes[0].fixed, name + ": the init keyframe fixed");
        std::vector<geometry::SE3f> before;
        for (int k = 0; k < K; ++k) before.push_back(a.keyframe(s.kfs[std::size_t(k)])->pose());
        const std::unique_ptr<ba_solver::SteppedSolve> st = ba_solver::startGlobal(gp);
        while (!st->finished()) st->step();
        local_mapping::LocalMapper m;
        local_mapping::PendingGlobalBA g;
        g.state = local_mapping::PendingGlobalBA::State::ready;
        g.map = M;
        g.loop_kf = s.kfs[std::size_t(f.scalar("gba_kf"))];
        g.imu_initialized = false;
        g.result = st->result();
        gba_iterations = g.result.iterations;
        {  // !bImuInit && isImuInitialized(): not written back
            map::Atlas b = a;
            b.editMap(M)->imu_initialized = true;
            local_mapping::LocalMapper mb;
            check(!orb_loop_closing::applyGlobalBA(b, mb, g), name + ": a visual BA dropped once the IMU is initialized");
        }
        const int big = a.findMap(M)->big_change_index;
        check(orb_loop_closing::applyGlobalBA(a, m, g), name + ": the global BA written back");
        check(a.findMap(M)->big_change_index == big + 1, name + ": InformNewBigChange");
        for (int k = 0; k < K; ++k)
            gba_moved = std::max(gba_moved, double((a.keyframe(s.kfs[std::size_t(k)])->poseInverse().translation() -
                                                    before[std::size_t(k)].inverse().translation()).norm()));
        check(gba_moved > 1e-3, name + ": the global BA moved nothing (the test would not check it)");
    }

    // The keyframes' points and loop edges
    int pts_diff = 0, edges_diff = 0;
    for (int k = 0; k < K; ++k) {
        const std::string pre = "after_k" + std::to_string(k) + "_";
        const map::KeyFrame& kf = *a.keyframe(s.kfs[std::size_t(k)]);
        const Eigen::MatrixXd& want = f[pre + "points"];
        for (int i = 0; i < want.rows(); ++i)
            if (s.ptIndex(kf.points[std::size_t(i)]) != int(want(i))) ++pts_diff;
        std::vector<int> le;
        for (map::KeyFrameId l : kf.loop_edges) le.push_back(s.kfIndex(l));
        std::sort(le.begin(), le.end());
        const Eigen::MatrixXd& want_le = f[pre + "loop_edges"];
        std::vector<int> wl;
        for (int i = 0; i < want_le.rows(); ++i) wl.push_back(int(want_le(i)));
        if (le != wl) ++edges_diff;
    }
    check(pts_diff == 0, name + ": " + std::to_string(pts_diff) + " keyframe point slots differ");
    check(edges_diff == 0, name + ": loop edges of " + std::to_string(edges_diff) + " keyframes differ");

    // Poses and velocities
    double max_r = 0, max_t = 0, max_v = 0;
    const Eigen::MatrixXd& vel = f["after_velocity"];
    for (int k = 0; k < K; ++k) {
        const map::KeyFrame& kf = *a.keyframe(s.kfs[std::size_t(k)]);
        const geometry::SE3d want{Eigen::Matrix4d(f["after_k" + std::to_string(k) + "_Tcw"])};
        const geometry::SE3d got = kf.pose().cast<double>();
        max_r = std::max(max_r, angleDeg(got.rotationMatrix(), want.rotationMatrix()));
        max_t = std::max(max_t, (got.translation() - want.translation()).norm());
        max_v = std::max(max_v, (kf.velocity().cast<double>() - vel.row(k).transpose()).norm());
    }
    // Points
    const Eigen::MatrixXd& P = f["after_points"];
    int bad_diff = 0;
    double max_p = 0;
    for (int i = 0; i < P.rows(); ++i) {
        const map::MapPoint& mp = *a.point(s.pts[std::size_t(i)]);
        if (mp.bad != (P(i, 3) != 0)) ++bad_diff;
        if (mp.bad) continue;
        const double d = (mp.pos.cast<double>() - P.block<1, 3>(i, 0).transpose()).norm();
        max_p = std::max(max_p, d);
    }
    check(bad_diff == 0, name + ": " + std::to_string(bad_diff) + " points' bad flags differ");
    check(max_r < 1e-3 && max_t < 1e-4, name + ": keyframe poses");
    check(max_v < 1e-4, name + ": velocities");
    check(max_p < 1e-4, name + ": point positions");
    check(graph_moved > 1e-3, name + ": the essential graph moved nothing (the test would not check it)");

    // How much the correction moved the map (that the comparison is not trivial)
    double moved = 0;
    for (int k = 0; k < K; ++k) {
        const geometry::SE3d before = fixture::se3f(f["kf" + std::to_string(k) + "_Tcw"]).cast<double>();
        const geometry::SE3d want{Eigen::Matrix4d(f["after_k" + std::to_string(k) + "_Tcw"])};
        moved = std::max(moved, (before.inverse().translation() - want.inverse().translation()).norm());
    }
    std::printf("%s: %s, %zu vertices, %zu edges, %d iterations; vs the original: poses %.1e° %.1e m, velocities "
                "%.1e, points %.1e m (keyframes moved up to %.3f m, %.4f m by the graph); %d point slots, %d bad flags differ\n",
                name.c_str(), four_dof ? "4DoF" : "6DoF", p->keyframes.size(), p->edges.size(), r.iterations, max_r,
                max_t, max_v, max_p, moved, graph_moved, pts_diff, bad_diff);
    if (f.has("gba_kf"))
        std::printf("%s: global BA %d iterations, moved keyframes up to %.4f m\n", name.c_str(), gba_iterations,
                    gba_moved);
}

}  // namespace

int main() {
    const fixture::Rig rig;
    for (const char* c : {"correct", "correct_4dof", "gba"}) runCase(c, rig);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
