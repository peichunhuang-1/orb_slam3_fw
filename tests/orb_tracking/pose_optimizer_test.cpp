// orb_tracking/shared/pose_optimizer vs ORB-SLAM3's Optimizer::PoseOptimization
// (tests/fixtures/orb_slam3/pose_optimization): the pose, the outlier flag of every feature with a map point,
// and the inlier count. The same again on a 4-thread ParallelFor: identical to the sequential result.
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "algorithms/orb_tracking/shared/pose_optimizer.hpp"
#include "tests/orb_tracking/fixture_frame.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

}  // namespace

int main() {
    const fixture::Rig rig;
    const char* cases[] = {"nominal", "dense", "sparse", "heavy_outliers", "large_init_error", "few_edges", "left_only", "far"};
    for (const char* name : cases) {
        const fixture::File f(std::string("pose_optimization/") + name + ".txt");
        fixture::Observations o = fixture::frameOf(f, rig);
        o.frame.setPose(fixture::se3f(f["Tcw_init"]));
        const int n = orb_tracking::optimizePose(o.frame, rig.rig, o.world);

        const geometry::SE3f expected = fixture::se3f(f["out_Tcw"]);
        const geometry::SE3f& got = o.frame.pose();
        const double rot = Eigen::AngleAxisd((expected.rotationMatrix().transpose() * got.rotationMatrix()).cast<double>()).angle();
        const double trans = (expected.translation() - got.translation()).cast<double>().norm();
        const Eigen::MatrixXd& out = f["out_outlier"];
        const Eigen::MatrixXd& of = f["mp_of_index"];
        int flag_diff = 0, with_point = 0;
        for (int i = 0; i < o.frame.size(); ++i) {
            if (of(i) < 0) continue;
            ++with_point;
            if (o.frame.outlier[std::size_t(i)] != (out(i) != 0)) ++flag_diff;
        }
        const int n_expected = int(f.scalar("out_n_inliers"));
        std::printf("%-17s edges %3d  inliers %3d (orb %3d)  flags differ %d  rot %.2e rad  trans %.2e m\n", name,
                    with_point, n, n_expected, flag_diff, rot, trans);
        check(n == n_expected, std::string(name) + ": inlier count");
        check(flag_diff == 0, std::string(name) + ": outlier flags");
        check(rot < 1e-5 && trans < 1e-5, std::string(name) + ": pose");
    }
    // Fixed chunks summed in chunk order: the thread count does not change the result (bit for bit)
    const orb_tracking::ParallelFor threaded = [](int begin, int end, const std::function<void(int)>& fn) {
        std::vector<std::thread> th;
        for (int i = begin; i < end; ++i) th.emplace_back(fn, i);
        for (std::thread& t : th) t.join();
    };
    for (const char* name : cases) {
        const fixture::File f(std::string("pose_optimization/") + name + ".txt");
        fixture::Observations a = fixture::frameOf(f, rig), b = fixture::frameOf(f, rig);
        a.frame.setPose(fixture::se3f(f["Tcw_init"]));
        b.frame.setPose(fixture::se3f(f["Tcw_init"]));
        const int na = orb_tracking::optimizePose(a.frame, rig.rig, a.world);
        const int nb = orb_tracking::optimizePose(b.frame, rig.rig, b.world, threaded);
        const bool same = na == nb && a.frame.outlier == b.frame.outlier &&
                          a.frame.pose().translation() == b.frame.pose().translation() &&
                          a.frame.pose().unitQuaternion().coeffs() == b.frame.pose().unitQuaternion().coeffs();
        check(same, std::string(name) + ": threaded == sequential");
    }
    std::printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
