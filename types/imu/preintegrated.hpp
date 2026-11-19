#pragma once
// IMU preintegration between two times (ORB-SLAM3's IMU::Preintegrated, Forster et al. on-manifold
// preintegration): ΔR, ΔV, ΔP at a linearization bias b̄, their Jacobians with respect to the bias, the
// covariance, and the measurements, so it can be re-integrated at another bias.
//
// Same math as the original, in float as there:
//   integrate        IntegrateNewMeasurement (same update order; ΔR orthonormalized every step)
//   setNewBias, updatedDelta*, delta*(b)
//                    first-order bias correction: ΔR(b) = ΔR Exp(JRg δbg), ΔV(b) = ΔV + JVg δbg + JVa δba, ...
//   reintegrate      Reintegrate: integrate the measurements again at the updated bias
//   mergePrevious    MergePrevious: prev's then this one's measurements, integrated at this one's updated bias
// and one addition, append (concatenation of two preintegrations at the same b̄, by the chain rule): the
// tracking side chains the per-frame preintegrations into the one from the last keyframe. It equals
// integrating both measurement lists in sequence, up to rounding.
//
// Conventions:
//   Bias      g (gyro, rad/s), a (accelerometer, m/s²)
//   C         15x15 covariance, order (rotation, velocity, position, gyro walk, acc walk); the rotation error
//             is a right perturbation, ΔR_true = ΔR Exp(δφ)
//   Noise     discrete: per-sample variances, as ORB-SLAM3's IMU::Calib (Tracking builds it from the
//             continuous densities: noise·√rate, walk/√rate)
#include <vector>

#include <Eigen/Core>

#include "fw/fw.hpp"

namespace calib { struct ImuNoise; }

namespace imu {

struct Bias {
    Eigen::Vector3f g = Eigen::Vector3f::Zero();  // gyro
    Eigen::Vector3f a = Eigen::Vector3f::Zero();  // accelerometer
};

// One integration step: the IntegrateNewMeasurement arguments
struct Measurement {
    Eigen::Vector3f a = Eigen::Vector3f::Zero();  // specific force (m/s²)
    Eigen::Vector3f w = Eigen::Vector3f::Zero();  // angular velocity (rad/s)
    float dt = 0;                                 // s
};

struct Noise {
    Eigen::Matrix<float, 6, 6> cov = Eigen::Matrix<float, 6, 6>::Zero();       // (gyro, acc) measurement noise
    Eigen::Matrix<float, 6, 6> cov_walk = Eigen::Matrix<float, 6, 6>::Zero();  // (gyro, acc) bias random walk

    // From discrete standard deviations (IMU::Calib's ng, na, ngw, naw)
    static Noise fromDiscrete(float ng, float na, float ngw, float naw);
    // From continuous densities at rate_hz, as ORB-SLAM3's Tracking: ng·√f, na·√f, ngw/√f, naw/√f
    static Noise fromContinuous(const calib::ImuNoise& n);
};

class Preintegrated {
public:
    Preintegrated() { initialize(Bias()); }
    Preintegrated(const Bias& b, const Noise& noise) : noise_(noise) { initialize(b); }

    void integrate(const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro, float dt);
    void integrate(const Measurement& m) { integrate(m.a, m.w, m.dt); }

    // The updated bias bu (δb = bu − b̄): read by updatedDelta*
    void setNewBias(const Bias& bu);
    const Bias& bias() const { return b_; }           // linearization point b̄ (GetOriginalBias)
    const Bias& updatedBias() const { return bu_; }   // GetUpdatedBias
    Eigen::Matrix<float, 6, 1> deltaBias() const;     // bu − b̄: (δbg, δba) (GetDeltaBias)

    // At bias b, first order in b − b̄ (GetDelta*(b))
    Eigen::Matrix3f deltaRotation(const Bias& b) const;
    Eigen::Vector3f deltaVelocity(const Bias& b) const;
    Eigen::Vector3f deltaPosition(const Bias& b) const;
    // At the updated bias (GetUpdatedDelta*)
    Eigen::Matrix3f updatedDeltaRotation() const { return deltaRotation(bu_); }
    Eigen::Vector3f updatedDeltaVelocity() const { return deltaVelocity(bu_); }
    Eigen::Vector3f updatedDeltaPosition() const { return deltaPosition(bu_); }

    // Integrate the measurements again at the updated bias; it becomes b̄
    void reintegrate();
    // prev's measurements then this one's, integrated at this one's updated bias
    void mergePrevious(const Preintegrated& prev);
    // This one followed by next (next starts where this one ends). Both must have the same b̄ (throws
    // std::invalid_argument otherwise); the updated bias is kept.
    void append(const Preintegrated& next);

    const Noise& noise() const { return noise_; }
    const std::vector<Measurement>& measurements() const { return measurements_; }

    // At b̄ (GetOriginalDelta*, and the members the optimizers read)
    float dT = 0;
    Eigen::Matrix3f dR;
    Eigen::Vector3f dV, dP;
    Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
    Eigen::Matrix<float, 15, 15> C;
    Eigen::Vector3f avgA, avgW;  // time averages: rotated specific force, angular velocity (bias removed)

private:
    void initialize(const Bias& b);

    Noise noise_;
    Bias b_, bu_;
    std::vector<Measurement> measurements_;
};

}  // namespace imu

FW_REGISTER_TYPE(imu::Preintegrated, "imu/Preintegrated")
