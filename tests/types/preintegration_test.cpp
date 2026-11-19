// types/imu/preintegrated + algorithms/imu_preintegration/interval.hpp against ORB-SLAM3
// (tests/fixtures/orb_slam3/preintegration, README.md there):
//   interval     PreintegrateIMU's sample selection and interpolation: meas_a, meas_b from imu
//   a_           integrate at b0;  a_q_: delta*(b2);  a_u_: after setNewBias(b1)
//   z_, z_u_     integrate at zero bias, then corrected to b1
//   m_           mergePrevious;  r_: reintegrate
// and the fw design's own path, which has no counterpart in the original:
//   append       z(meas_a) + z(meas_b) chained = z(meas_a, meas_b) integrated in sequence (rounding only)
//   zero → b1    z_u_ (first-order correction from 0) against r_ (re-integrated at b1): the design's error
#include <cstdio>
#include <string>
#include <vector>

#include "algorithms/imu_preintegration/interval.hpp"
#include "tests/common/orb_slam3_fixture.hpp"
#include "types/calib/stereo_imu_rig.hpp"
#include "types/geometry/so3.hpp"
#include "types/imu/preintegrated.hpp"

namespace {

int failures = 0;
void check(bool ok, const std::string& what, double value) {
    std::printf("  %-66s %10.3g  %s\n", what.c_str(), value, ok ? "ok" : "FAIL");
    if (!ok) ++failures;
}

// max |x − y| / max |y|
template <typename A, typename B> double rel(const A& x, const B& y) {
    const Eigen::MatrixXd d = x.template cast<double>() - y.template cast<double>();
    const double s = y.template cast<double>().cwiseAbs().maxCoeff();
    return d.cwiseAbs().maxCoeff() / (s > 0 ? s : 1.0);
}
// rotation angle between two rotations (rad)
double angle(const Eigen::Matrix3f& R1, const Eigen::Matrix3f& R2) {
    return geometry::log(geometry::normalizeRotation(Eigen::Matrix3d((R1.transpose() * R2).cast<double>()))).norm();
}

std::vector<imu_preintegration::Sample> samples(const fixture::File& f) {
    const Eigen::MatrixXd& m = f["imu"];
    std::vector<imu_preintegration::Sample> s(m.rows());
    for (int i = 0; i < m.rows(); ++i) {
        s[i].t = m(i, 0);
        s[i].a = m.block<1, 3>(i, 1).transpose().cast<float>();
        s[i].w = m.block<1, 3>(i, 4).transpose().cast<float>();
    }
    return s;
}

std::vector<imu::Measurement> measurements(const fixture::File& f, const std::string& key) {
    const Eigen::MatrixXd& m = f[key];
    std::vector<imu::Measurement> v(m.rows());
    for (int i = 0; i < m.rows(); ++i) {
        v[i].a = m.block<1, 3>(i, 0).transpose().cast<float>();
        v[i].w = m.block<1, 3>(i, 3).transpose().cast<float>();
        v[i].dt = float(m(i, 6));
    }
    return v;
}

imu::Bias bias(const fixture::File& f, const std::string& p) {
    imu::Bias b;
    b.g = f.f<3, 1>(p + "_g");
    b.a = f.f<3, 1>(p + "_a");
    return b;
}

imu::Preintegrated integrate(const std::vector<imu::Measurement>& m, const imu::Bias& b, const imu::Noise& n) {
    imu::Preintegrated p(b, n);
    for (const imu::Measurement& x : m) p.integrate(x);
    return p;
}

struct Worst {
    double v = 0;
    void operator()(double x) { v = std::max(v, x); }
};

// Largest relative difference over everything a Preintegrated holds, against fixture prefix p
double compare(const imu::Preintegrated& x, const fixture::File& f, const std::string& p, double* rotation = nullptr) {
    Worst w;
    w(std::abs(x.dT - f.scalar(p + "dT")) / f.scalar(p + "dT"));
    w(rel(x.dV, f[p + "dV"]));
    w(rel(x.dP, f[p + "dP"]));
    w(rel(x.JRg, f[p + "JRg"]));
    w(rel(x.JVg, f[p + "JVg"]));
    w(rel(x.JVa, f[p + "JVa"]));
    w(rel(x.JPg, f[p + "JPg"]));
    w(rel(x.JPa, f[p + "JPa"]));
    w(rel(x.C.block<9, 9>(0, 0), f[p + "C"].block(0, 0, 9, 9)));
    w(rel(x.C.block<6, 6>(9, 9), f[p + "C"].block(9, 9, 6, 6)));
    w(rel(x.C.block<9, 6>(0, 9), f[p + "C"].block(0, 9, 9, 6)));  // zero in both
    w(rel(x.avgA, f[p + "avgA"]));
    w(rel(x.avgW, f[p + "avgW"]));
    w(rel(x.bias().g, f[p + "bg"]));
    w(rel(x.bias().a, f[p + "ba"]));
    if (rotation) *rotation = angle(x.dR, f.f<3, 3>(p + "dR"));
    return w.v;
}

}  // namespace

int main() {
    const fixture::File rig("rig.txt");
    calib::ImuNoise cn;
    cn.rate_hz = rig.scalar("imu_hz");
    cn.gyro_noise = rig["imu_noise_density"](0);
    cn.accel_noise = rig["imu_noise_density"](1);
    cn.gyro_walk = rig["imu_noise_density"](2);
    cn.accel_walk = rig["imu_noise_density"](3);
    const imu::Noise noise = imu::Noise::fromContinuous(cn);
    {
        const Eigen::MatrixXd d = rig["imu_calib_discrete"];
        const imu::Noise ref = imu::Noise::fromDiscrete(float(d(0)), float(d(1)), float(d(2)), float(d(3)));
        std::printf("noise\n");
        check(rel(noise.cov, ref.cov) < 1e-6 && rel(noise.cov_walk, ref.cov_walk) < 1e-6,
              "fromContinuous = Tracking's IMU::Calib values", std::max(rel(noise.cov, ref.cov), rel(noise.cov_walk, ref.cov_walk)));
    }

    const char* cases[] = {"frame_20hz_a",   "frame_20hz_b", "frame_20hz_aligned", "frame_20hz_zero_bias", "frame_20hz_big_dbias",
                           "keyframe_0p5s",  "keyframe_1p5s", "short_4ms",         "short_4ms_empty"};
    Worst wSel, wInt, wRot, wQ, wU, wZ, wM, wR, wApp, wAppRot;
    for (const char* name : cases) {
        const fixture::File f(std::string("preintegration/") + name + ".txt");
        std::printf("%s\n", name);
        const std::vector<imu_preintegration::Sample> q = samples(f);
        const double tPrev = f.scalar("t_prev"), tCur = f.scalar("t_cur"), tNext = f.scalar("t_next");

        // Selection and interpolation (exact: same float operations)
        const imu_preintegration::Interval ia = imu_preintegration::intervalMeasurements(q, tPrev, tCur);
        const std::vector<imu_preintegration::Sample> rest(q.begin() + long(ia.consumed), q.end());
        const imu_preintegration::Interval ib = imu_preintegration::intervalMeasurements(rest, tCur, tNext);
        const std::vector<imu::Measurement> mA = measurements(f, "meas_a"), mB = measurements(f, "meas_b");
        double dSel = (ia.steps.size() == mA.size() && ib.steps.size() == mB.size()) ? 0 : 1;
        for (std::size_t i = 0; dSel == 0 && i < mA.size(); ++i)
            dSel = std::max({dSel, rel(ia.steps[i].a, mA[i].a), rel(ia.steps[i].w, mA[i].w),
                             double(std::abs(ia.steps[i].dt - mA[i].dt) / mA[i].dt)});
        for (std::size_t i = 0; dSel < 1 && i < mB.size(); ++i)
            dSel = std::max({dSel, rel(ib.steps[i].a, mB[i].a), rel(ib.steps[i].w, mB[i].w),
                             double(std::abs(ib.steps[i].dt - mB[i].dt) / mB[i].dt)});
        check(dSel == 0, "interval: meas_a, meas_b (" + std::to_string(mA.size()) + ", " + std::to_string(mB.size()) + " steps)", dSel);
        wSel(dSel);
        if (!f.has("a_dT")) {
            check(ia.steps.empty(), "interval: no preintegration", double(ia.steps.size()));
            continue;
        }

        const imu::Bias b0 = bias(f, "b0"), b1 = bias(f, "b1"), b2 = bias(f, "b2");

        // a_: integrated at b0
        imu::Preintegrated A = integrate(mA, b0, noise);
        double rot;
        const double dA = compare(A, f, "a_", &rot);
        check(dA < 2e-5 && rot < 1e-5, "a_: integrate at b0 (rel, dR rad)", std::max(dA, rot));
        wInt(dA);
        wRot(rot);

        // a_q_: first-order correction to b2
        const double dQ = std::max({angle(A.deltaRotation(b2), f.f<3, 3>("a_q_dR")), rel(A.deltaVelocity(b2), f["a_q_dV"]),
                                    rel(A.deltaPosition(b2), f["a_q_dP"])});
        check(dQ < 2e-5, "a_q_: delta*(b2)", dQ);
        wQ(dQ);

        // a_u_: after setNewBias(b1)
        A.setNewBias(b1);
        const double dU = std::max({angle(A.updatedDeltaRotation(), f.f<3, 3>("a_u_dR")),
                                    rel(A.updatedDeltaVelocity(), f["a_u_dV"]), rel(A.updatedDeltaPosition(), f["a_u_dP"]),
                                    rel(A.deltaBias(), f["a_u_db"])});
        check(dU < 2e-5, "a_u_: setNewBias(b1), updatedDelta*, deltaBias", dU);
        wU(dU);

        // z_, z_u_: integrated at zero, corrected to b1
        imu::Preintegrated Z = integrate(mA, imu::Bias(), noise);
        double zrot;
        const double dZ0 = compare(Z, f, "z_", &zrot);
        Z.setNewBias(b1);
        const double dZ = std::max({dZ0, zrot, angle(Z.updatedDeltaRotation(), f.f<3, 3>("z_u_dR")),
                                    rel(Z.updatedDeltaVelocity(), f["z_u_dV"]), rel(Z.updatedDeltaPosition(), f["z_u_dP"])});
        check(dZ < 2e-5, "z_, z_u_: integrate at 0, correct to b1", dZ);
        wZ(dZ);

        // m_: B at b0, setNewBias(b1), mergePrevious(A)
        if (f.has("m_dT")) {
            imu::Preintegrated B = integrate(mB, b0, noise);
            B.setNewBias(b1);
            B.mergePrevious(A);
            double mrot;
            const double dM = compare(B, f, "m_", &mrot);
            check(dM < 2e-5 && mrot < 1e-5, "m_: mergePrevious", std::max(dM, mrot));
            wM(dM);

            // append: chained at zero = integrated in sequence at zero
            imu::Preintegrated Za = integrate(mA, imu::Bias(), noise), Zb = integrate(mB, imu::Bias(), noise);
            std::vector<imu::Measurement> mAB = mA;
            mAB.insert(mAB.end(), mB.begin(), mB.end());
            const imu::Preintegrated Zab = integrate(mAB, imu::Bias(), noise);
            Za.append(Zb);
            Worst w;
            w(std::abs(Za.dT - Zab.dT) / Zab.dT);
            for (auto [x, y] : {std::pair{&Za.dV, &Zab.dV}, {&Za.dP, &Zab.dP}, {&Za.avgA, &Zab.avgA}, {&Za.avgW, &Zab.avgW}})
                w(rel(*x, *y));
            for (auto [x, y] : {std::pair{&Za.JRg, &Zab.JRg}, {&Za.JVg, &Zab.JVg}, {&Za.JVa, &Zab.JVa}, {&Za.JPg, &Zab.JPg},
                                {&Za.JPa, &Zab.JPa}})
                w(rel(*x, *y));
            w(rel(Za.C.block<9, 9>(0, 0), Zab.C.block<9, 9>(0, 0)));
            w(rel(Za.C.block<6, 6>(9, 9), Zab.C.block<6, 6>(9, 9)));
            const double arot = angle(Za.dR, Zab.dR);
            check(w.v < 2e-5 && arot < 1e-5 && Za.measurements().size() == mAB.size(),
                  "append: chained = integrated in sequence (rel, dR rad)", std::max(w.v, arot));
            wApp(w.v);
            wAppRot(arot);
        }

        // r_: reintegrate at b1
        A.reintegrate();
        double rrot;
        const double dR = compare(A, f, "r_", &rrot);
        check(dR < 2e-5 && rrot < 1e-5, "r_: reintegrate", std::max(dR, rrot));
        wR(dR);

        // The design's error: corrected from 0 against re-integrated at b1 (information, no threshold)
        std::printf("    zero -> b1 first-order vs reintegrated: dR %.3g rad, dV %.3g m/s, dP %.3g m (dT %.3f s)\n",
                    angle(Z.updatedDeltaRotation(), A.dR), (Z.updatedDeltaVelocity() - A.dV).norm(),
                    (Z.updatedDeltaPosition() - A.dP).norm(), A.dT);
    }

    std::printf("worst: interval %.3g, integrate %.3g (dR %.3g rad), delta(b) %.3g, updated %.3g, zero %.3g, merge %.3g, "
                "reintegrate %.3g, append %.3g (dR %.3g rad)\n",
                wSel.v, wInt.v, wRot.v, wQ.v, wU.v, wZ.v, wM.v, wR.v, wApp.v, wAppRot.v);
    {   // append rejects different linearization points
        imu::Bias b;
        b.g.x() = 0.01f;
        imu::Preintegrated x(imu::Bias(), noise), y(b, noise);
        y.integrate(Eigen::Vector3f(0, 0, 9.81f), Eigen::Vector3f::Zero(), 0.005f);
        bool threw = false;
        try { x.append(y); } catch (const std::invalid_argument&) { threw = true; }
        check(threw, "append: different b̄ rejected", threw ? 0 : 1);
    }
    std::printf("%s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
