// orb_tracking/track_local_map/pose_inertial_optimizer vs ORB-SLAM3's PoseInertialOptimizationLastKeyFrame and
// PoseInertialOptimizationLastFrame (tests/fixtures/orb_slam3/pose_inertial): the body state and bias, the
// outlier flags, the inlier count and the prior left on the frame. The preintegrations are made from the
// fixtures' measurements at the original's biases, as it had them; the last-keyframe case is also run with the
// fw design's preintegration (at zero bias, corrected by the bias Jacobians) to measure that difference.
#include <cmath>
#include <cstdio>
#include <string>

#include "algorithms/orb_tracking/track_local_map/pose_inertial_optimizer.hpp"
#include "tests/orb_tracking/fixture_frame.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

imu::Noise noise() {
    const fixture::File r("rig.txt");
    const Eigen::MatrixXd& d = r["imu_calib_discrete"];
    return imu::Noise::fromDiscrete(float(d(0)), float(d(1)), float(d(2)), float(d(3)));
}

imu::Bias bias(const fixture::File& f, const std::string& p) {
    imu::Bias b;
    b.g = f.f<3, 1>(p + "bg");
    b.a = f.f<3, 1>(p + "ba");
    return b;
}

std::shared_ptr<imu::Preintegrated> integrate(const fixture::File& f, const std::string& meas, const imu::Bias& b) {
    auto p = std::make_shared<imu::Preintegrated>(b, noise());
    const Eigen::MatrixXd& m = f[meas];
    for (int i = 0; i < m.rows(); ++i)
        p->integrate(Eigen::Vector3f(float(m(i, 0)), float(m(i, 1)), float(m(i, 2))),
                     Eigen::Vector3f(float(m(i, 3)), float(m(i, 4)), float(m(i, 5))), float(m(i, 6)));
    return p;
}

std::vector<float> trackDepth(const fixture::File& f, const fixture::Observations& o) {
    const geometry::SE3f T = fixture::se3f(f["Tcw_true"]);
    std::vector<float> d(o.world.size(), 100.f);
    for (std::size_t i = 0; i < d.size(); ++i)
        if (o.world[i]) d[i] = (T * *o.world[i]).z();
    return d;
}

// The frame at the optimization's start (init_*: PredictStateIMU's result)
void setInit(tracking::Frame& frame, const fixture::File& f) {
    frame.setPose(fixture::se3f(f["init_Tcw"]));
    frame.setVelocity(f.f<3, 1>("init_vwb"));
    frame.bias = bias(f, "init_");
}

double rotationError(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
    return Eigen::AngleAxisd(a.transpose() * b).angle();
}

void compare(const std::string& name, const fixture::File& f, const fixture::Observations& o, int n) {
    const tracking::Frame& fr = o.frame;
    const double rot = rotationError(f["out_Rwb"], fr.imuRotation().cast<double>());
    const double trans = (f["out_twb"] - fr.imuPosition().cast<double>()).norm();
    const double vel = (f["out_vwb"] - fr.velocity().cast<double>()).norm();
    const double bg = (f["out_bg"] - fr.bias.g.cast<double>()).norm();
    const double ba = (f["out_ba"] - fr.bias.a.cast<double>()).norm();
    const Eigen::MatrixXd& out = f["out_outlier"];
    const Eigen::MatrixXd& of = f["mp_of_index"];
    int flag_diff = 0;
    for (int i = 0; i < fr.size(); ++i)
        if (of(i) >= 0 && fr.outlier[std::size_t(i)] != (out(i) != 0)) ++flag_diff;
    const int n_expected = int(f.scalar("out_n_inliers"));
    const Eigen::MatrixXd& H = f["out_prior_H"];
    const double h_rel = fr.prior ? (fr.prior->H - H).norm() / H.norm() : 1.0;
    const double prior_state = fr.prior ? (fr.prior->twb - f["out_prior_twb"]).norm() + (fr.prior->bg - f["out_prior_bg"]).norm()
                                        : 1.0;
    std::printf("%-26s inliers %3d (orb %3d) flags %d  R %.1e  t %.1e  v %.1e  bg %.1e  ba %.1e  H %.1e\n", name.c_str(), n,
                n_expected, flag_diff, rot, trans, vel, bg, ba, h_rel);
    check(n == n_expected, name + ": inlier count");
    check(flag_diff == 0, name + ": outlier flags");
    check(rot < 1e-5 && trans < 1e-5 && vel < 1e-5, name + ": body state");
    check(bg < 1e-6 && ba < 1e-5, name + ": bias");
    check(h_rel < 1e-4 && prior_state < 1e-5, name + ": prior");
}

}  // namespace

int main() {
    const fixture::Rig rig;
    const char* cases[] = {"nominal", "bias_error", "few_points", "outliers", "fast", "next_frame"};
    for (const char* name : cases) {
        {
            const fixture::File f(std::string("pose_inertial/") + name + "_last_keyframe.txt");
            fixture::Observations o = fixture::frameOf(f, rig);
            setInit(o.frame, f);
            orb_tracking::ImuState kf;
            kf.Rwb = f.f<3, 3>("kf_Rwb");
            kf.twb = f.f<3, 1>("kf_twb");
            kf.vwb = f.f<3, 1>("kf_vwb");
            kf.bias = bias(f, "kf_");
            o.frame.preintegrated = integrate(f, "meas_kf", kf.bias);
            check((o.frame.preintegrated->C.cast<double>() - f["pre_kf_C"]).norm() < 1e-6 * f["pre_kf_C"].norm(),
                  std::string(name) + ": keyframe preintegration");
            const int n = orb_tracking::optimizePoseInertialLastKeyFrame(o.frame, kf, rig.rig, o.world, trackDepth(f, o));
            compare(std::string(name) + " last_keyframe", f, o, n);

            // The fw design: the preintegration made at zero bias, corrected to the keyframe's bias by its Jacobians
            fixture::Observations z = fixture::frameOf(f, rig);
            setInit(z.frame, f);
            z.frame.preintegrated = integrate(f, "meas_kf", imu::Bias());
            z.frame.preintegrated->setNewBias(kf.bias);
            const int nz = orb_tracking::optimizePoseInertialLastKeyFrame(z.frame, kf, rig.rig, z.world, trackDepth(f, z));
            const double dr = rotationError(f["out_Rwb"], z.frame.imuRotation().cast<double>());
            const double dt = (f["out_twb"] - z.frame.imuPosition().cast<double>()).norm();
            std::printf("  zero-bias preintegration: inliers %3d, R %.1e rad, t %.1e m\n", nz, dr, dt);
            check(std::abs(nz - n) <= 2 && dr < 1e-4 && dt < 1e-3, std::string(name) + ": zero-bias preintegration");
        }
        {
            const fixture::File f(std::string("pose_inertial/") + name + "_last_frame.txt");
            fixture::Observations o = fixture::frameOf(f, rig);
            setInit(o.frame, f);
            tracking::Frame prev;
            prev.T_cb = o.frame.T_cb;
            prev.Trl = o.frame.Trl;
            prev.setImuPoseVelocity(f.f<3, 3>("prev_Rwb"), f.f<3, 1>("prev_twb"), f.f<3, 1>("prev_vwb"));
            prev.bias = bias(f, "prev_");
            tracking::PoseImuPrior c;
            c.Rwb = f["prior_Rwb"];
            c.twb = f["prior_twb"];
            c.vwb = f["prior_vwb"];
            c.bg = f["prior_bg"];
            c.ba = f["prior_ba"];
            c.H = f["prior_H"];
            prev.prior = c;
            o.frame.preintegrated_frame = integrate(f, "meas_frame", bias(f, "pre_frame_"));
            o.frame.preintegrated = integrate(f, "meas_kf", bias(f, "pre_kf_"));
            const int n = orb_tracking::optimizePoseInertialLastFrame(o.frame, prev, rig.rig, o.world, trackDepth(f, o));
            compare(std::string(name) + " last_frame", f, o, n);
            check(!prev.prior, std::string(name) + ": previous prior dropped");
        }
    }
    std::printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
