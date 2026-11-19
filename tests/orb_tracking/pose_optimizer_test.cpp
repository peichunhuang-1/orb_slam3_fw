// orb_tracking/shared/pose_optimizer vs ORB-SLAM3's Optimizer::PoseOptimization
// (tests/fixtures/orb_slam3/pose_optimization): the pose, the outlier flag of every feature with a map point,
// and the inlier count.
#include <cmath>
#include <cstdio>
#include <string>

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
    std::printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
