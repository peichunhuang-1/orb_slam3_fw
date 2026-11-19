// types/geometry/so3.hpp: exp / log against Eigen::AngleAxis, the Jacobians against numerical derivatives,
// and agreement with ORB-SLAM3's formulas (copied below) where those are accurate.
#include <cstdio>
#include <random>
#include <string>

#include <Eigen/Geometry>

#include "types/geometry/so3.hpp"

namespace {

int failures = 0;
void check(bool ok, const std::string& what, double value) {
    std::printf("  %-64s %10.3g  %s\n", what.c_str(), value, ok ? "ok" : "FAIL");
    if (!ok) ++failures;
}

using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;

// ORB-SLAM3 (g2o_types.cpp, imu_types.cpp), double, for angles away from its cut-offs
M3 origExp(const V3& w) {
    const double d2 = w.squaredNorm(), d = std::sqrt(d2);
    const M3 W = geometry::hat(w);
    return M3::Identity() + W * std::sin(d) / d + W * W * (1.0 - std::cos(d)) / d2;
}
M3 origJr(const V3& w) {
    const double d2 = w.squaredNorm(), d = std::sqrt(d2);
    const M3 W = geometry::hat(w);
    return M3::Identity() - W * (1.0 - std::cos(d)) / d2 + W * W * (d - std::sin(d)) / (d2 * d);
}
M3 origJrInv(const V3& w) {
    const double d2 = w.squaredNorm(), d = std::sqrt(d2);
    const M3 W = geometry::hat(w);
    return M3::Identity() + W / 2 + W * W * (1.0 / d2 - (1.0 + std::cos(d)) / (2.0 * d * std::sin(d)));
}
V3 origLog(const M3& R) {
    const double tr = R.trace();
    V3 w((R(2, 1) - R(1, 2)) / 2, (R(0, 2) - R(2, 0)) / 2, (R(1, 0) - R(0, 1)) / 2);
    const double c = (tr - 1.0) * 0.5;
    if (c > 1 || c < -1) return w;
    const double th = std::acos(c), s = std::sin(th);
    return std::fabs(s) < 1e-5 ? w : V3(th * w / s);
}

M3 truthExp(const V3& w) {
    const double th = w.norm();
    return th == 0 ? M3::Identity() : M3(Eigen::AngleAxisd(th, w / th).toRotationMatrix());
}

V3 randomDir(std::mt19937& g) {
    std::normal_distribution<double> n(0, 1);
    return V3(n(g), n(g), n(g)).normalized();
}

}  // namespace

int main() {
    std::mt19937 g(7);
    std::uniform_real_distribution<double> u(0, 1);
    const double angles[] = {0, 1e-12, 1e-8, 1e-5, 1e-4, 1e-3, 0.01, 0.0999, 0.1, 0.1001, 0.5, 1, 2, 3,
                             3.1, 3.14, M_PI - 1e-4, M_PI - 1e-7};

    std::printf("exp and log (double)\n");
    double eExp = 0, eLog = 0, eOrth = 0;
    for (double th : angles)
        for (int k = 0; k < 20; ++k) {
            const V3 w = th * randomDir(g);
            const M3 R = geometry::exp(w);
            eExp = std::max(eExp, (R - truthExp(w)).cwiseAbs().maxCoeff());
            eOrth = std::max(eOrth, (R.transpose() * R - M3::Identity()).cwiseAbs().maxCoeff());
            if (th < M_PI - 1e-6) eLog = std::max(eLog, (geometry::log(R) - w).norm());
        }
    check(eExp < 1e-15, "exp vs AngleAxis, max |dR|", eExp);
    check(eOrth < 1e-15, "exp orthonormal, max |RtR - I|", eOrth);
    check(eLog < 1e-12, "log(exp(w)) = w for |w| < pi - 1e-6, max |dw|", eLog);
    {   // at π: either sign of the axis
        const V3 n = randomDir(g);
        const V3 l = geometry::log(truthExp(M_PI * n));
        const double e = std::min((l - M_PI * n).norm(), (l + M_PI * n).norm());
        check(e < 1e-7, "log at pi: +-pi * axis", e);
    }

    std::printf("Jacobians against numerical derivatives (double)\n");
    double eJr = 0, eJrInv = 0, eInv = 0;
    for (double th : angles) {
        if (th > 3) continue;
        for (int k = 0; k < 10; ++k) {
            const V3 w = th * randomDir(g);
            const M3 R = geometry::exp(w), Jr = geometry::rightJacobian(w);
            M3 num, numInv;
            for (int j = 0; j < 3; ++j) {
                const double h = 1e-6;
                const V3 d = V3::Unit(j) * h;
                // exp(w + d) = exp(w) exp(Jr d)
                num.col(j) = (geometry::log(R.transpose() * geometry::exp(V3(w + d))) -
                              geometry::log(R.transpose() * geometry::exp(V3(w - d)))) / (2 * h);
                // log(exp(w) exp(d)) = w + Jr⁻¹ d
                numInv.col(j) = (geometry::log(M3(R * geometry::exp(d))) - geometry::log(M3(R * geometry::exp(V3(-d))))) / (2 * h);
            }
            eJr = std::max(eJr, (Jr - num).cwiseAbs().maxCoeff());
            eJrInv = std::max(eJrInv, (geometry::inverseRightJacobian(w) - numInv).cwiseAbs().maxCoeff());
            eInv = std::max(eInv, (geometry::inverseRightJacobian(w) * Jr - M3::Identity()).cwiseAbs().maxCoeff());
        }
    }
    check(eJr < 1e-8, "rightJacobian vs numerical, max", eJr);
    check(eJrInv < 1e-8, "inverseRightJacobian vs numerical, max", eJrInv);
    check(eInv < 1e-14, "inverseRightJacobian * rightJacobian = I, max", eInv);

    std::printf("ORB-SLAM3 formulas, |w| in [0.05, 3] (double)\n");
    double dExp = 0, dJr = 0, dJrInv = 0, dLog = 0;
    for (int k = 0; k < 2000; ++k) {
        const V3 w = (0.05 + 2.95 * u(g)) * randomDir(g);
        dExp = std::max(dExp, (geometry::exp(w) - origExp(w)).cwiseAbs().maxCoeff());
        dJr = std::max(dJr, (geometry::rightJacobian(w) - origJr(w)).cwiseAbs().maxCoeff());
        dJrInv = std::max(dJrInv, (geometry::inverseRightJacobian(w) - origJrInv(w)).cwiseAbs().maxCoeff());
        dLog = std::max(dLog, (geometry::log(origExp(w)) - origLog(origExp(w))).norm());
    }
    check(dExp < 1e-14, "exp vs original", dExp);
    check(dJr < 1e-12, "rightJacobian vs original", dJr);
    check(dJrInv < 1e-12, "inverseRightJacobian vs original", dJrInv);
    check(dLog < 1e-10, "log vs original", dLog);

    std::printf("float\n");
    double fExp = 0, fJr = 0, fJrInv = 0, fLog = 0;
    for (double th : angles) {
        if (th > 3) continue;
        for (int k = 0; k < 20; ++k) {
            const V3 w = th * randomDir(g);
            const Eigen::Vector3f wf = w.cast<float>();
            fExp = std::max(fExp, (geometry::exp(wf).cast<double>() - geometry::exp(w)).cwiseAbs().maxCoeff());
            fJr = std::max(fJr, (geometry::rightJacobian(wf).cast<double>() - geometry::rightJacobian(w)).cwiseAbs().maxCoeff());
            fJrInv = std::max(fJrInv, (geometry::inverseRightJacobian(wf).cast<double>() -
                                       geometry::inverseRightJacobian(w)).cwiseAbs().maxCoeff());
            fLog = std::max(fLog, (geometry::log(geometry::exp(wf)).cast<double>() - w).norm());
        }
    }
    check(fExp < 1e-6, "exp float vs double", fExp);
    check(fJr < 1e-6, "rightJacobian float vs double", fJr);
    check(fJrInv < 1e-6, "inverseRightJacobian float vs double", fJrInv);
    check(fLog < 2e-6, "log(exp(w)) float", fLog);

    {
        Eigen::Matrix3d R = truthExp(V3(0.3, -0.2, 0.5));
        R(0, 1) += 1e-3;
        const M3 N = geometry::normalizeRotation(R);
        check((N.transpose() * N - M3::Identity()).cwiseAbs().maxCoeff() < 1e-14, "normalizeRotation orthonormal",
              (N.transpose() * N - M3::Identity()).cwiseAbs().maxCoeff());
    }

    std::printf("%s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
