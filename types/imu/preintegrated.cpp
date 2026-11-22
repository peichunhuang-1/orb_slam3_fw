#include "types/imu/preintegrated.hpp"

#include <cmath>
#include <stdexcept>

#include "types/calib/stereo_imu_rig.hpp"
#include "types/geometry/so3.hpp"

namespace imu {

namespace {

// Rotation of a gyro-bias correction. A non-finite or huge correction only comes from a diverging optimizer
// step; as ORB-SLAM3's BiasCorrectionRotation, it is ignored (the step keeps its error elsewhere and is
// rejected) instead of producing NaNs.
Eigen::Matrix3f biasCorrection(const Eigen::Vector3f& w) {
    if (!w.allFinite() || w.squaredNorm() > 1e12f) return Eigen::Matrix3f::Identity();
    return geometry::exp(w);
}

}  // namespace

Noise Noise::fromDensities(float ng, float na, float ngw, float naw) {
    Noise n;
    n.density << ng, ng, ng, na, na, na;
    n.walk_density << ngw, ngw, ngw, naw, naw, naw;
    return n;
}

Eigen::Matrix<float, 6, 6> Noise::measurementCov(float dt) const {
    const Eigen::Matrix<float, 6, 1> s = fixed_hz > 0 ? Eigen::Matrix<float, 6, 1>(density * std::sqrt(fixed_hz))
                                                      : Eigen::Matrix<float, 6, 1>(density / std::sqrt(dt));
    return s.cwiseProduct(s).asDiagonal();
}

Eigen::Matrix<float, 6, 6> Noise::walkCov(float dt) const {
    const Eigen::Matrix<float, 6, 1> s = fixed_hz > 0 ? Eigen::Matrix<float, 6, 1>(walk_density / std::sqrt(fixed_hz))
                                                      : Eigen::Matrix<float, 6, 1>(walk_density * std::sqrt(dt));
    return s.cwiseProduct(s).asDiagonal();
}

Noise Noise::fromContinuous(const calib::ImuNoise& c) {
    return fromDensities(float(c.gyro_noise), float(c.accel_noise), float(c.gyro_walk), float(c.accel_walk));
}

void Preintegrated::initialize(const Bias& b) {
    dT = 0;
    dR.setIdentity();
    dV.setZero();
    dP.setZero();
    JRg.setZero();
    JVg.setZero();
    JVa.setZero();
    JPg.setZero();
    JPa.setZero();
    C.setZero();
    avgA.setZero();
    avgW.setZero();
    b_ = b;
    bu_ = b;
    measurements_.clear();
}

void Preintegrated::integrate(const Eigen::Vector3f& acceleration, const Eigen::Vector3f& angVel, float dt) {
    measurements_.push_back({acceleration, angVel, dt});

    const Eigen::Vector3f acc = acceleration - b_.a;
    const Eigen::Vector3f w = angVel - b_.g;

    avgA = (dT * avgA + dR * acc * dt) / (dT + dt);
    avgW = (dT * avgW + w * dt) / (dT + dt);

    // Position, then velocity, with the rotation at the start of the step
    dP = dP + dV * dt + 0.5f * dR * acc * dt * dt;
    dV = dV + dR * acc * dt;

    // Error transition A and noise input B (rotation, velocity, position)
    const Eigen::Matrix3f Wacc = geometry::hat(acc);
    Eigen::Matrix<float, 9, 9> A = Eigen::Matrix<float, 9, 9>::Identity();
    Eigen::Matrix<float, 9, 6> B = Eigen::Matrix<float, 9, 6>::Zero();
    A.block<3, 3>(3, 0) = -dR * dt * Wacc;
    A.block<3, 3>(6, 0) = -0.5f * dR * dt * dt * Wacc;
    A.block<3, 3>(6, 3) = Eigen::Matrix3f::Identity() * dt;
    B.block<3, 3>(3, 3) = dR * dt;
    B.block<3, 3>(6, 3) = 0.5f * dR * dt * dt;

    // Position and velocity Jacobians, with the rotation and JRg at the start of the step
    JPa = JPa + JVa * dt - 0.5f * dR * dt * dt;
    JPg = JPg + JVg * dt - 0.5f * dR * dt * dt * Wacc * JRg;
    JVa = JVa - dR * dt;
    JVg = JVg - dR * dt * Wacc * JRg;

    // Rotation of this step and its right Jacobian
    const Eigen::Vector3f phi = w * dt;
    const Eigen::Matrix3f dRi = geometry::exp(phi);
    const Eigen::Matrix3f Jr = geometry::rightJacobian(phi);
    dR = geometry::normalizeRotation(Eigen::Matrix3f(dR * dRi));

    A.block<3, 3>(0, 0) = dRi.transpose();
    B.block<3, 3>(0, 0) = Jr * dt;

    // Noise discretized at this step's dt (a zero-length step adds none: B is zero)
    C.block<9, 9>(0, 0) = A * C.block<9, 9>(0, 0) * A.transpose();
    if (dt > 0) {
        C.block<9, 9>(0, 0) += B * noise_.measurementCov(dt) * B.transpose();
        C.block<6, 6>(9, 9) += noise_.walkCov(dt);
    }

    JRg = dRi.transpose() * JRg - Jr * dt;

    dT += dt;
}

void Preintegrated::setNewBias(const Bias& bu) { bu_ = bu; }

Eigen::Matrix<float, 6, 1> Preintegrated::deltaBias() const {
    Eigen::Matrix<float, 6, 1> db;
    db << bu_.g - b_.g, bu_.a - b_.a;
    return db;
}

Eigen::Matrix3f Preintegrated::deltaRotation(const Bias& b) const {
    return geometry::normalizeRotation(Eigen::Matrix3f(dR * biasCorrection(JRg * (b.g - b_.g))));
}

Eigen::Vector3f Preintegrated::deltaVelocity(const Bias& b) const {
    return dV + JVg * (b.g - b_.g) + JVa * (b.a - b_.a);
}

Eigen::Vector3f Preintegrated::deltaPosition(const Bias& b) const {
    return dP + JPg * (b.g - b_.g) + JPa * (b.a - b_.a);
}

void Preintegrated::reintegrate() {
    const std::vector<Measurement> m = measurements_;
    initialize(bu_);
    for (const Measurement& x : m) integrate(x);
}

void Preintegrated::mergePrevious(const Preintegrated& prev) {
    if (&prev == this) return;
    const std::vector<Measurement> m1 = prev.measurements_, m2 = measurements_;
    initialize(bu_);
    for (const Measurement& x : m1) integrate(x);
    for (const Measurement& x : m2) integrate(x);
}

void Preintegrated::append(const Preintegrated& next) {
    if (next.b_.g != b_.g || next.b_.a != b_.a)
        throw std::invalid_argument("imu::Preintegrated::append: different linearization biases");
    if (next.dT <= 0) return;
    if (dT <= 0) {
        const Bias bu = bu_;
        *this = next;
        bu_ = bu;
        return;
    }
    const Eigen::Matrix3f R1 = dR, R2 = next.dR;
    const Eigen::Matrix3f R2t = R2.transpose();
    const Eigen::Matrix3f Vx = geometry::hat(next.dV), Px = geometry::hat(next.dP);
    const float t2 = next.dT;

    // Covariance: δ = Φ δ1 + G δ2 (δ2 is in next's frame: its velocity and position rotate by R1)
    Eigen::Matrix<float, 9, 9> Phi = Eigen::Matrix<float, 9, 9>::Identity();
    Phi.block<3, 3>(0, 0) = R2t;
    Phi.block<3, 3>(3, 0) = -R1 * Vx;
    Phi.block<3, 3>(6, 0) = -R1 * Px;
    Phi.block<3, 3>(6, 3) = Eigen::Matrix3f::Identity() * t2;
    Eigen::Matrix<float, 9, 9> G = Eigen::Matrix<float, 9, 9>::Identity();
    G.block<3, 3>(3, 3) = R1;
    G.block<3, 3>(6, 6) = R1;
    C.block<9, 9>(0, 0) = Phi * C.block<9, 9>(0, 0) * Phi.transpose() + G * next.C.block<9, 9>(0, 0) * G.transpose();
    C.block<6, 6>(9, 9) += next.C.block<6, 6>(9, 9);

    // Jacobians (chain rule, with this one's JRg before it changes)
    JPa = JPa + JVa * t2 + R1 * next.JPa;
    JPg = JPg + JVg * t2 - R1 * Px * JRg + R1 * next.JPg;
    JVa = JVa + R1 * next.JVa;
    JVg = JVg - R1 * Vx * JRg + R1 * next.JVg;
    JRg = R2t * JRg + next.JRg;

    const float t = dT + t2;
    avgA = (dT * avgA + t2 * R1 * next.avgA) / t;
    avgW = (dT * avgW + t2 * next.avgW) / t;

    dP = dP + dV * t2 + R1 * next.dP;
    dV = dV + R1 * next.dV;
    dR = geometry::normalizeRotation(Eigen::Matrix3f(R1 * R2));
    dT = t;

    measurements_.insert(measurements_.end(), next.measurements_.begin(), next.measurements_.end());
}

}  // namespace imu
