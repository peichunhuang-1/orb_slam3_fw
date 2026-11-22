// types/imu/preintegrated + algorithms/imu_preintegration/interval.hpp against ORB-SLAM3
// (tests/fixtures/orb_slam3/preintegration, README.md there):
//   interval     the samples spanning each frame interval, by their timestamps only (interval.hpp): equal to
//                PreintegrateIMU's meas_a, meas_b where frames fall on samples (frame_20hz_aligned); elsewhere
//                the dts add up to the interval and the difference from the original's is printed. On jittered
//                stamps with linear signals Σ a·dt is the exact integral (interpolation and trapezoid exact)
//   a_           integrate at b0;  a_q_: delta*(b2);  a_u_: after setNewBias(b1)
//   z_, z_u_     integrate at zero bias, then corrected to b1
//   m_           mergePrevious;  r_: reintegrate
// and the fw design's own path, which has no counterpart in the original:
//   append       z(meas_a) + z(meas_b) chained = z(meas_a, meas_b) integrated in sequence (rounding only)
//   zero → b1    z_u_ (first-order correction from 0) against r_ (re-integrated at b1): the design's error
//   split        the covariance with every step cut in two halves ≈ the covariance as is: the noise is
//                discretized at each step's dt, not at a fixed IMU rate
// The covariance C is checked against the original only where every step is 1/imu_hz long: ORB-SLAM3 uses
// the noise discretized at the rate for every step, here each step's own dt (types/imu/preintegrated.hpp).
// Elsewhere its difference from the original is printed, not checked.
#include <cmath>
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

// To whole nanoseconds, as the pipeline's stamps (fw::Time) are: the generator's frame times are off its sample
// times by ~1e-15 s of accumulated rounding where they are meant to coincide
double ns(double t) { return std::round(t * 1e9) * 1e-9; }

std::vector<imu_preintegration::Sample> samples(const fixture::File& f) {
    const Eigen::MatrixXd& m = f["imu"];
    std::vector<imu_preintegration::Sample> s(m.rows());
    for (int i = 0; i < m.rows(); ++i) {
        s[i].t = ns(m(i, 0));
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

double sumDt(const std::vector<imu::Measurement>& m) {
    double s = 0;
    for (const imu::Measurement& x : m) s += double(x.dt);
    return s;
}

// Σ a·dt (acc) or Σ w·dt
Eigen::Vector3d integral(const std::vector<imu::Measurement>& m, bool acc) {
    Eigen::Vector3d s = Eigen::Vector3d::Zero();
    for (const imu::Measurement& x : m) s += (acc ? x.a : x.w).cast<double>() * double(x.dt);
    return s;
}

// interval.hpp on jittered timestamps with signals linear in time: interpolation and the trapezoid are exact,
// so Σ a·dt over each interval is ∫ a dt. Frames are chained through `consumed` as the block does.
void syntheticIntervals() {
    std::printf("interval, synthetic (jittered stamps, linear signals)\n");
    const Eigen::Vector3d a0(0.3, -9.8, 1.2), ka(2.0, 0.5, -1.0), w0(0.1, -0.2, 0.05), kw(-0.4, 0.3, 0.6);
    std::vector<imu_preintegration::Sample> q;
    unsigned r = 12345;
    double t = 10.0;
    for (int i = 0; i < 200; ++i) {
        r = r * 1103515245u + 12345u;
        t += 0.004 + 0.002 * double((r >> 16) & 0x7fff) / 32767.0;  // 4–6 ms
        imu_preintegration::Sample s;
        s.t = t;
        s.a = (a0 + ka * (t - 10.0)).cast<float>();
        s.w = (w0 + kw * (t - 10.0)).cast<float>();
        q.push_back(s);
    }
    const auto exact = [&](const Eigen::Vector3d& c0, const Eigen::Vector3d& k, double ta, double tb) {
        return Eigen::Vector3d(c0 * (tb - ta) + k * 0.5 * ((tb - 10.0) * (tb - 10.0) - (ta - 10.0) * (ta - 10.0)));
    };
    // frames: before the first sample (extrapolated back), unaligned, exactly on samples, a short one inside a
    // single step, and past the last sample (the end of the data: extrapolated forward)
    const std::vector<double> frames = {q[0].t - 0.003, q[10].t + 0.0017, q[20].t, q[30].t, q[30].t + 0.001,
                                        q[30].t + 0.002, q[80].t - 0.0001, q[190].t + 0.001, q[199].t + 0.002};
    std::vector<imu_preintegration::Sample> queue = q;
    double worst = 0, worstDt = 0;
    std::size_t steps = 0;
    for (std::size_t k = 1; k < frames.size(); ++k) {
        const imu_preintegration::Interval iv = imu_preintegration::intervalMeasurements(queue, frames[k - 1], frames[k]);
        check(!iv.steps.empty(), "synthetic: interval " + std::to_string(k) + " has steps", double(iv.steps.size()));
        worst = std::max({worst, rel(integral(iv.steps, true), exact(a0, ka, frames[k - 1], frames[k])),
                          rel(integral(iv.steps, false), exact(w0, kw, frames[k - 1], frames[k]))});
        worstDt = std::max(worstDt, std::abs(sumDt(iv.steps) - (frames[k] - frames[k - 1])));
        steps += iv.steps.size();
        queue.erase(queue.begin(), queue.begin() + long(iv.consumed));
    }
    check(worst < 1e-5, "synthetic: Σ a·dt, Σ w·dt = the exact integrals (rel)", worst);
    check(worstDt < 1e-7, "synthetic: dts add up to each interval (s)", worstDt);
    check(steps >= 200, "synthetic: every sample used (" + std::to_string(steps) + " steps)", double(steps));
    // no sample on one side: nothing
    const std::vector<imu_preintegration::Sample> after(q.begin() + 50, q.end());
    check(imu_preintegration::intervalMeasurements(after, q[0].t, q[40].t).steps.empty(),
          "synthetic: all samples after the interval: no steps", 0);
    check(imu_preintegration::intervalMeasurements({q[0]}, q[0].t - 0.01, q[0].t + 0.01).steps.empty(),
          "synthetic: a single sample: no steps", 0);
}

// Every step at the nominal period 1/hz (the only case where the covariance matches the original's)
bool nominal(const std::vector<imu::Measurement>& m, double hz) {
    for (const imu::Measurement& x : m)
        if (std::abs(double(x.dt) * hz - 1.0) > 1e-5) return false;
    return true;
}

// Largest relative difference of C against fixture prefix p
double compareC(const imu::Preintegrated& x, const fixture::File& f, const std::string& p) {
    Worst w;
    w(rel(x.C.block<9, 9>(0, 0), f[p + "C"].block(0, 0, 9, 9)));
    w(rel(x.C.block<6, 6>(9, 9), f[p + "C"].block(9, 9, 6, 6)));
    w(rel(x.C.block<9, 6>(0, 9), f[p + "C"].block(0, 9, 9, 6)));  // zero in both
    return w.v;
}

// Largest relative difference over everything a Preintegrated holds, against fixture prefix p (C only when withC;
// otherwise its difference goes to *cDiff)
double compare(const imu::Preintegrated& x, const fixture::File& f, const std::string& p, double* rotation, bool withC,
               double* cDiff = nullptr) {
    Worst w;
    w(std::abs(x.dT - f.scalar(p + "dT")) / f.scalar(p + "dT"));
    w(rel(x.dV, f[p + "dV"]));
    w(rel(x.dP, f[p + "dP"]));
    w(rel(x.JRg, f[p + "JRg"]));
    w(rel(x.JVg, f[p + "JVg"]));
    w(rel(x.JVa, f[p + "JVa"]));
    w(rel(x.JPg, f[p + "JPg"]));
    w(rel(x.JPa, f[p + "JPa"]));
    const double dC = compareC(x, f, p);
    if (withC) w(dC);
    if (cDiff) *cDiff = std::max(*cDiff, dC);
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
    const double hz = rig.scalar("imu_hz");
    calib::ImuNoise cn;
    cn.gyro_noise = rig["imu_noise_density"](0);
    cn.accel_noise = rig["imu_noise_density"](1);
    cn.gyro_walk = rig["imu_noise_density"](2);
    cn.accel_walk = rig["imu_noise_density"](3);
    const imu::Noise noise = imu::Noise::fromContinuous(cn);
    {
        // At dt = 1/hz the per-step discretization is what Tracking puts in IMU::Calib (σ·√f, σ_w/√f)
        const Eigen::MatrixXd d = rig["imu_calib_discrete"];
        const Eigen::Vector4f df = d.cast<float>();  // the float values IMU::Calib holds, squared in float as there
        Eigen::Matrix<float, 6, 1> c, cw;
        c << df(0) * df(0), df(0) * df(0), df(0) * df(0), df(1) * df(1), df(1) * df(1), df(1) * df(1);
        cw << df(2) * df(2), df(2) * df(2), df(2) * df(2), df(3) * df(3), df(3) * df(3), df(3) * df(3);
        const float dt = float(1.0 / hz);
        const imu::Noise fixed = noise.atFixedRate(hz);
        const double e = std::max(rel(noise.measurementCov(dt).diagonal(), c), rel(noise.walkCov(dt).diagonal(), cw));
        const double ef = std::max(rel(fixed.measurementCov(0.0013f).diagonal(), c), rel(fixed.walkCov(0.0013f).diagonal(), cw));
        std::printf("noise\n");
        check(e < 1e-6, "discretized at dt = 1/imu_hz = Tracking's IMU::Calib values", e);
        check(ef == 0, "atFixedRate(imu_hz), any dt = IMU::Calib values (exact)", ef);
    }

    syntheticIntervals();

    const char* cases[] = {"frame_20hz_a",   "frame_20hz_b", "frame_20hz_aligned", "frame_20hz_zero_bias", "frame_20hz_big_dbias",
                           "keyframe_0p5s",  "keyframe_1p5s", "short_4ms",         "short_4ms_empty"};
    Worst wSel, wInt, wRot, wQ, wU, wZ, wM, wR, wApp, wAppRot, wSplit, wC, wFixed;
    for (const char* name : cases) {
        const fixture::File f(std::string("preintegration/") + name + ".txt");
        std::printf("%s\n", name);
        const std::vector<imu_preintegration::Sample> q = samples(f);
        const double tPrev = ns(f.scalar("t_prev")), tCur = ns(f.scalar("t_cur")), tNext = ns(f.scalar("t_next"));

        // Selection and interpolation
        const imu_preintegration::Interval ia = imu_preintegration::intervalMeasurements(q, tPrev, tCur);
        const std::vector<imu_preintegration::Sample> rest(q.begin() + long(ia.consumed), q.end());
        const imu_preintegration::Interval ib = imu_preintegration::intervalMeasurements(rest, tCur, tNext);
        const std::vector<imu::Measurement> mA = measurements(f, "meas_a"), mB = measurements(f, "meas_b");
        const double dSum = std::max(std::abs(sumDt(ia.steps) - (tCur - tPrev)), std::abs(sumDt(ib.steps) - (tNext - tCur)));
        check(!ia.steps.empty() && !ib.steps.empty() && dSum < 1e-7,
              "interval: " + std::to_string(ia.steps.size()) + ", " + std::to_string(ib.steps.size()) +
                  " steps, dts add up to the interval (s)", dSum);
        if (std::string(name) == "frame_20hz_aligned") {
            double dSel = (ia.steps.size() == mA.size() && ib.steps.size() == mB.size()) ? 0 : 1;
            for (std::size_t i = 0; dSel == 0 && i < mA.size(); ++i)
                dSel = std::max({dSel, rel(ia.steps[i].a, mA[i].a), rel(ia.steps[i].w, mA[i].w),
                                 double(std::abs(ia.steps[i].dt - mA[i].dt) / mA[i].dt)});
            for (std::size_t i = 0; dSel < 1 && i < mB.size(); ++i)
                dSel = std::max({dSel, rel(ib.steps[i].a, mB[i].a), rel(ib.steps[i].w, mB[i].w),
                                 double(std::abs(ib.steps[i].dt - mB[i].dt) / mB[i].dt)});
            check(dSel < 1e-6, "interval: frames on samples = the original's meas_a, meas_b", dSel);
        }
        if (!mA.empty()) {
            const double dI = std::max(rel(integral(ia.steps, true), integral(mA, true)), rel(integral(ia.steps, false), integral(mA, false)));
            std::printf("    interval vs the original's meas_a (1 ms margin there): ∫a, ∫w rel %.3g, %zu vs %zu steps\n", dI,
                        ia.steps.size(), mA.size());
            wSel(dI);
        }
        if (!f.has("a_dT")) continue;  // the original made no preintegration for it

        const imu::Bias b0 = bias(f, "b0"), b1 = bias(f, "b1"), b2 = bias(f, "b2");
        const bool nomA = nominal(mA, hz), nomAB = nomA && nominal(mB, hz);
        double cDiff = 0;  // C against the original where it is not checked

        // a_: integrated at b0
        imu::Preintegrated A = integrate(mA, b0, noise);
        double rot;
        const double dA = compare(A, f, "a_", &rot, nomA, &cDiff);
        check(dA < 2e-5 && rot < 1e-5, std::string("a_: integrate at b0 (rel, dR rad)") + (nomA ? "" : ", C not checked"),
              std::max(dA, rot));
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
        const double dZ0 = compare(Z, f, "z_", &zrot, nomA, &cDiff);
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
            const double dM = compare(B, f, "m_", &mrot, nomAB, &cDiff);
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
        const double dR = compare(A, f, "r_", &rrot, nomA, &cDiff);
        check(dR < 2e-5 && rrot < 1e-5, "r_: reintegrate", std::max(dR, rrot));
        wR(dR);

        if (!nomAB) {
            std::printf("    C vs the original (noise at a fixed rate there, at each dt here; not checked): %.3g\n", cDiff);
            wC(cDiff);
        }
        {   // atFixedRate: the original's discretization, so its C
            const double dF = compareC(integrate(mA, b0, noise.atFixedRate(hz)), f, "a_");
            check(dF < 2e-5, "a_ C with atFixedRate(imu_hz) = the original's", dF);
            wFixed(dF);
        }

        // split: every step cut in two halves of the same measurement. Discretized at dt, the noise added
        // over the interval stays the same (the original's fixed-rate noise would double it)
        {
            std::vector<imu::Measurement> half;
            for (imu::Measurement m : mA) {
                m.dt *= 0.5f;
                half.push_back(m);
                half.push_back(m);
            }
            const imu::Preintegrated whole = integrate(mA, imu::Bias(), noise), split = integrate(half, imu::Bias(), noise);
            const double dS = std::max(rel(split.C.block<9, 9>(0, 0), whole.C.block<9, 9>(0, 0)),
                                       rel(split.C.block<6, 6>(9, 9), whole.C.block<6, 6>(9, 9)));
            check(dS < 2e-3, "split: C with every step halved", dS);
            wSplit(dS);
        }

        // The design's error: corrected from 0 against re-integrated at b1 (information, no threshold)
        std::printf("    zero -> b1 first-order vs reintegrated: dR %.3g rad, dV %.3g m/s, dP %.3g m (dT %.3f s)\n",
                    angle(Z.updatedDeltaRotation(), A.dR), (Z.updatedDeltaVelocity() - A.dV).norm(),
                    (Z.updatedDeltaPosition() - A.dP).norm(), A.dT);
    }

    std::printf("worst: interval vs the original %.3g, integrate %.3g (dR %.3g rad), delta(b) %.3g, updated %.3g, zero %.3g, merge %.3g, "
                "reintegrate %.3g, append %.3g (dR %.3g rad), split %.3g; C vs the original off the rate %.3g, at a fixed rate %.3g\n",
                wSel.v, wInt.v, wRot.v, wQ.v, wU.v, wZ.v, wM.v, wR.v, wApp.v, wAppRot.v, wSplit.v, wC.v, wFixed.v);
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
