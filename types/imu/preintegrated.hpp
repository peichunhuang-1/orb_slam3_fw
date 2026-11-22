#pragma once
// IMU preintegration between two times (ORB-SLAM3's IMU::Preintegrated, Forster et al. on-manifold
// preintegration): ΔR, ΔV, ΔP at a linearization bias b̄, their Jacobians with respect to the bias, the
// covariance, and the measurements, so it can be re-integrated at another bias.
//
// Same math as the original, in float as there, except for the noise (below):
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
//   Noise     continuous-time densities (Kalibr / EuRoC sensor.yaml), discretized per step from the step's
//             own dt: measurement σ_d² = σ²/dt, bias walk σ_wd² = σ_w²·dt. ORB-SLAM3 discretizes once at a
//             configured IMU rate f (IMU::Calib: σ·√f, σ_w/√f) and uses that for every step, including the
//             shorter interpolated ones at the frame ends; here no rate is assumed, the sample timestamps give
//             it. Equal to the original for every step with dt = 1/f.
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
    // Continuous-time densities σ per axis (gyro x y z, acc x y z)
    Eigen::Matrix<float, 6, 1> density = Eigen::Matrix<float, 6, 1>::Zero();       // noise: rad/s/√Hz, m/s²/√Hz
    Eigen::Matrix<float, 6, 1> walk_density = Eigen::Matrix<float, 6, 1>::Zero();  // bias walk: rad/s²/√Hz, m/s³/√Hz

    static Noise fromDensities(float ng, float na, float ngw, float naw);
    static Noise fromContinuous(const calib::ImuNoise& n);

    // Discretized for one step of dt seconds: measurement σ²/dt, walk σ_w²·dt
    Eigen::Matrix<float, 6, 6> measurementCov(float dt) const;
    Eigen::Matrix<float, 6, 6> walkCov(float dt) const;

    // ORB-SLAM3's discretization instead: every step at the rate hz whatever its dt, (σ·√f)² and (σ_w/√f)² in
    // float as Tracking builds IMU::Calib. Only for comparing against the original (the golden tests); the
    // pipeline discretizes at each step's dt.
    Noise atFixedRate(double hz) const {
        Noise n = *this;
        n.fixed_hz = float(hz);
        return n;
    }
    float fixed_hz = 0;  // 0: each step's own dt
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
