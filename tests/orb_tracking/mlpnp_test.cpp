// orb_tracking/relocalize/mlpnp_solver vs ORB-SLAM3's MLPnPsolver (tests/fixtures/orb_slam3/mlpnp):
//   - the bearings (unproject at z = 1) and computePose on fixed correspondences (the first 6, all): exact up to
//     rounding;
//   - the RANSAC settings SetRansacParameters derives: exact;
//   - RANSAC as Relocalization runs it (iterate(5) until a pose or no more): the draws differ from the original's,
//     so both are measured against the truth: the pose error and the share of true inliers.
#include <cmath>
#include <cstdio>
#include <string>

#include "algorithms/orb_tracking/relocalize/mlpnp_solver.hpp"
#include "tests/orb_tracking/fixture_frame.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

void poseError(const Eigen::MatrixXd& expected, const Eigen::Matrix<double, 3, 4>& got, double& rot, double& trans) {
    const Eigen::Matrix3d R = expected.block<3, 3>(0, 0);
    rot = Eigen::AngleAxisd(Eigen::Matrix3d(R.transpose() * got.leftCols<3>())).angle();
    trans = (expected.block<3, 1>(0, 3) - got.col(3)).norm();
}

}  // namespace

int main() {
    const fixture::Rig rig;
    const char* cases[] = {"nominal", "noise_free", "heavy_outliers", "few", "too_few"};
    for (const char* name : cases) {
        const fixture::File f(std::string("mlpnp/") + name + ".txt");
        fixture::Observations o = fixture::frameOf(f, rig);
        const std::string n(name);

        // Bearings and computePose
        const Eigen::MatrixXd& idx = f["cp_index"];
        const Eigen::MatrixXd& cb = f["cp_bearings"];
        std::vector<Eigen::Vector3d> bearings, points;
        double bearing_err = 0;
        for (int k = 0; k < idx.rows(); ++k) {
            const cv::KeyPoint& kp = o.frame.features->keypoint(int(idx(k)));
            const Eigen::Vector3f b = rig.rig.camera(0).unproject_orb(Eigen::Vector2f(kp.pt.x, kp.pt.y));
            bearing_err = std::max(bearing_err, (b.cast<double>() - cb.row(k).transpose()).norm());
            bearings.push_back(cb.row(k).transpose());
            points.push_back(o.world[std::size_t(idx(k))]->cast<double>());
        }
        check(bearing_err < 1e-6, n + ": bearings");
        if (f.has("cp6_Tcw")) {
            double r6, t6, ra, ta;
            const std::vector<Eigen::Vector3d> b6(bearings.begin(), bearings.begin() + 6), p6(points.begin(), points.begin() + 6);
            poseError(f["cp6_Tcw"], orb_tracking::MLPnPSolver::computePose(b6, p6), r6, t6);
            poseError(f["cpall_Tcw"], orb_tracking::MLPnPSolver::computePose(bearings, points), ra, ta);
            std::printf("%-15s computePose: 6 points R %.1e t %.1e, all R %.1e t %.1e\n", name, r6, t6, ra, ta);
            check(r6 < 1e-6 && t6 < 1e-6 && ra < 1e-8 && ta < 1e-8, n + ": computePose");
        }

        // RANSAC
        orb_tracking::MLPnPSolver solver(o.frame, rig.rig.camera(0), o.world);
        solver.setRansacParameters(0.99, 10, 300, 6, 0.5f, 5.991f);
        check(solver.minInliers() == int(f.scalar("ransac_min_inliers")) &&
                  solver.maxIterations() == int(f.scalar("ransac_max_iterations")),
              n + ": RANSAC settings");
        const Eigen::MatrixXd& me = f["ransac_max_error"];
        bool same_error = int(solver.maxError().size()) == me.rows();
        for (int i = 0; same_error && i < me.rows(); ++i) same_error = solver.maxError()[std::size_t(i)] == float(me(i));
        check(same_error, n + ": max errors");

        bool ok = false, no_more = false;
        std::vector<bool> inliers;
        int n_inliers = 0, calls = 0;
        Eigen::Matrix4f T;
        while (!ok && !no_more && calls < 1000) {
            ok = solver.iterate(5, no_more, inliers, n_inliers, T);
            ++calls;
        }
        const bool ok_expected = f.scalar("ransac_ok") != 0;
        check(ok == ok_expected, n + ": RANSAC result");
        if (ok && ok_expected) {
            // Against the truth, as the original's own draw is: pose error and the share of true inliers
            const Eigen::MatrixXd Ttrue = f["Tcw_true"].topRows(3);
            const Eigen::MatrixXd& truth = f["outlier_truth"];
            const Eigen::MatrixXd& ei = f["ransac_inliers"];
            const auto precision = [&](auto inlier) {
                int in = 0, good = 0;
                for (int i = 0; i < o.frame.size(); ++i)
                    if (inlier(i)) {
                        ++in;
                        good += truth(i) == 0;
                    }
                return in ? double(good) / in : 0.0;
            };
            const double p_fw = precision([&](int i) { return bool(inliers[std::size_t(i)]); });
            const double p_orb = precision([&](int i) { return ei(i) != 0; });
            double rot, trans, rot_orb, trans_orb;
            poseError(Ttrue, Eigen::Matrix<double, 3, 4>(T.topRows<3>().cast<double>()), rot, trans);
            poseError(Ttrue, Eigen::Matrix<double, 3, 4>(f["ransac_Tcw"].topRows(3)), rot_orb, trans_orb);
            std::printf("%-15s RANSAC: inliers %d (orb %d), true inliers %.2f (orb %.2f), vs truth R %.1e t %.1e "
                        "(orb R %.1e t %.1e)\n",
                        name, n_inliers, int(f.scalar("ransac_n_inliers")), p_fw, p_orb, rot, trans, rot_orb, trans_orb);
            check(p_fw >= p_orb - 0.05 && n_inliers >= solver.minInliers(), n + ": RANSAC inliers");
            check(rot < std::max(3 * rot_orb, 1e-4) && trans < std::max(3 * trans_orb, 1e-3), n + ": RANSAC pose");
        } else {
            std::printf("%-15s RANSAC: no pose (orb: %s)\n", name, ok_expected ? "pose" : "no pose");
        }
    }
    std::printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
