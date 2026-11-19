#pragma once
// SO(3) for tracking, the IMU preintegration and the optimizers: hat / vee, exp, log, right Jacobian and
// its inverse, nearest rotation. Float and double (the preintegration runs in float, the optimizers in double).
//
// Same maps as ORB-SLAM3 (IMU::IntegratedRotation, RightJacobianSO3, InverseRightJacobianSO3, ExpSO3, LogSO3,
// NormalizeRotation), with numerically stable forms where the originals lose precision:
//   (1 - cos θ) / θ²        as 2 sin²(θ/2) / θ²
//   (θ - sin θ) / θ³        series below θ = 0.1
//   1/θ² - (1 + cos θ) / (2θ sin θ)   as 1/θ² - cot(θ/2) / (2θ), series below θ = 0.1
//   log                     θ = atan2(|vee|, (tr - 1) / 2) instead of acos; near π, the axis from the
//                           symmetric part (the original returns the near-zero skew part there)
// The originals' small-angle cut-offs (the identity Jacobian, I + W for exp) are first-order versions of the
// same series; the results agree to the rounding of the scalar type.
#include <cmath>

#include <Eigen/Core>
#include <Eigen/SVD>

namespace geometry {

template <typename T> using Vec3 = Eigen::Matrix<T, 3, 1>;
template <typename T> using Mat3 = Eigen::Matrix<T, 3, 3>;

// [v]x: hat(v) * u = v.cross(u)
template <typename Derived>
Mat3<typename Derived::Scalar> hat(const Eigen::MatrixBase<Derived>& v) {
    using T = typename Derived::Scalar;
    Mat3<T> W;
    W << T(0), -v(2), v(1),
         v(2), T(0), -v(0),
         -v(1), v(0), T(0);
    return W;
}

// Inverse of hat on the skew part of M
template <typename Derived>
Vec3<typename Derived::Scalar> vee(const Eigen::MatrixBase<Derived>& M) {
    using T = typename Derived::Scalar;
    return Vec3<T>(M(2, 1) - M(1, 2), M(0, 2) - M(2, 0), M(1, 0) - M(0, 1)) * T(0.5);
}

namespace detail {

// sin θ / θ and (1 - cos θ) / θ², from θ² and θ
template <typename T> void expCoefficients(T th2, T th, T& a, T& b) {
    if (th < T(1e-4)) {
        a = T(1) - th2 / T(6);
        b = T(0.5) - th2 / T(24);
    } else {
        const T s = std::sin(T(0.5) * th);
        a = std::sin(th) / th;
        b = T(2) * s * s / th2;
    }
}

// (θ - sin θ) / θ³
template <typename T> T jacobianC(T th2, T th) {
    if (th < T(0.1)) return T(1) / T(6) - th2 * (T(1) / T(120) - th2 * (T(1) / T(5040) - th2 / T(362880)));
    return (th - std::sin(th)) / (th2 * th);
}

}  // namespace detail

// exp(hat(w)) (Rodrigues)
template <typename Derived>
Mat3<typename Derived::Scalar> exp(const Eigen::MatrixBase<Derived>& w) {
    using T = typename Derived::Scalar;
    const T th2 = w.squaredNorm(), th = std::sqrt(th2);
    T a, b;
    detail::expCoefficients(th2, th, a, b);
    const Mat3<T> W = hat(w);
    return Mat3<T>::Identity() + a * W + b * W * W;
}

// Rotation vector of R, |result| in [0, π]. R must be a rotation (orthonormal, det 1).
template <typename Derived>
Vec3<typename Derived::Scalar> log(const Eigen::MatrixBase<Derived>& R) {
    using T = typename Derived::Scalar;
    const Vec3<T> v = vee(R);  // sin θ · axis
    const T s = v.norm();
    const T c = std::max(T(-1), std::min(T(1), (R.trace() - T(1)) * T(0.5)));  // cos θ
    const T th = std::atan2(s, c);
    if (c > T(-0.9)) {
        if (s < T(1e-4)) return v * (T(1) + s * s / T(6));  // θ / sin θ
        return v * (th / s);
    }
    // Near π the skew part vanishes: the axis from (R + Rᵀ)/2 = cos θ I + (1 - cos θ) n nᵀ, its sign from v
    const Mat3<T> nn = ((R + R.transpose()) * T(0.5) - c * Mat3<T>::Identity()) / (T(1) - c);
    int k;
    nn.diagonal().maxCoeff(&k);
    Vec3<T> n = nn.col(k) / std::sqrt(nn(k, k));
    if (n.dot(v) < T(0)) n = -n;
    return n * th;
}

// Right Jacobian: exp(w + δ) ≈ exp(w) exp(Jr(w) δ)
template <typename Derived>
Mat3<typename Derived::Scalar> rightJacobian(const Eigen::MatrixBase<Derived>& w) {
    using T = typename Derived::Scalar;
    const T th2 = w.squaredNorm(), th = std::sqrt(th2);
    T a, b;
    detail::expCoefficients(th2, th, a, b);
    const Mat3<T> W = hat(w);
    return Mat3<T>::Identity() - b * W + detail::jacobianC(th2, th) * W * W;
}

// Jr(w)⁻¹: log(exp(w) exp(δ)) ≈ w + Jr(w)⁻¹ δ. Defined for |w| < 2π.
template <typename Derived>
Mat3<typename Derived::Scalar> inverseRightJacobian(const Eigen::MatrixBase<Derived>& w) {
    using T = typename Derived::Scalar;
    const T th2 = w.squaredNorm(), th = std::sqrt(th2);
    T d;  // 1/θ² - (1 + cos θ) / (2θ sin θ)
    if (th < T(0.1))
        d = T(1) / T(12) + th2 * (T(1) / T(720) + th2 * (T(1) / T(30240) + th2 / T(1209600)));
    else
        d = T(1) / th2 - std::cos(T(0.5) * th) / (T(2) * th * std::sin(T(0.5) * th));
    const Mat3<T> W = hat(w);
    return Mat3<T>::Identity() + T(0.5) * W + d * W * W;
}

// Nearest orthonormal matrix (U Vᵀ of the SVD), as ORB-SLAM3's NormalizeRotation
template <typename Derived>
Mat3<typename Derived::Scalar> normalizeRotation(const Eigen::MatrixBase<Derived>& R) {
    using T = typename Derived::Scalar;
    Eigen::JacobiSVD<Mat3<T>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

}  // namespace geometry
