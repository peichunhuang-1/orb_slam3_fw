#pragma once
// The inertial optimizations' variables and edges (ORB-SLAM3's g2o_types: VertexPose, VertexGDir, VertexScale,
// EdgeInertial, EdgeInertialGS, EdgeGyroRW / EdgeAccRW, EdgePriorAcc / EdgePriorGyro, EdgeMono) as nls manifolds
// and residuals. Same errors and analytic Jacobians as the original, including two of its details:
//   - the bias priors' Jacobian is +I although their error is prior - b (EdgePriorAcc / EdgePriorGyro)
//   - VertexPose's periodic NormalizeRotation discards its result, so the body rotation is only the product of
//     the (orthonormalized) increments
// Velocities and biases are nls::Euclidean(3).
#include <memory>

#include <Eigen/Core>

#include "libs/nls/problem.hpp"
#include "types/camera/kannala_brandt.hpp"
#include "types/geometry/se3.hpp"
#include "types/imu/preintegrated.hpp"

namespace ba_solver {

constexpr double kGravity = 9.81;  // IMU::GRAVITY_VALUE

// Body pose (ImuCamPose's Rwb, twb): Rwb column-major then twb, 12 doubles. x ⊞ (ω, υ): twb += Rwb·υ,
// Rwb = Rwb·Exp(ω) (ImuCamPose::Update, in the body frame).
class ImuPoseManifold final : public nls::Manifold {
public:
    int ambientSize() const override { return 12; }
    int tangentSize() const override { return 6; }
    void plus(const double* x, const double* delta, double* x_plus) const override;
};

// Gravity direction (GDirection): Rwg column-major, 9 doubles. x ⊞ (a, b) = Rwg·Exp(a, b, 0).
class GravityDirManifold final : public nls::Manifold {
public:
    int ambientSize() const override { return 9; }
    int tangentSize() const override { return 2; }
    void plus(const double* x, const double* delta, double* x_plus) const override;
};

// Scale (VertexScale): s ⊞ δ = s·exp(δ)
class ScaleManifold final : public nls::Manifold {
public:
    int ambientSize() const override { return 1; }
    int tangentSize() const override { return 1; }
    void plus(const double* x, const double* delta, double* x_plus) const override;
};

// A body pose block's values from a keyframe's Tcw and T_cb (as GetImuRotation / GetImuPosition, in float)
void imuPoseFrom(const geometry::SE3f& Tcw, const geometry::SE3f& T_cb, double x[12]);
// The left camera's Tcw from a body pose block (VertexPose's Rcw[0], tcw[0], as the write-back reads them)
geometry::SE3f cameraPoseOf(const double x[12], const geometry::SE3f& T_cb);

// EdgeInertial's / EdgeInertialGS's information: C(0:9, 0:9)⁻¹, symmetrized, eigenvalues below 1e-12 set to 0
Eigen::Matrix<double, 9, 9> inertialInformation(const imu::Preintegrated& p);

// EdgeInertial: blocks (P1, V1, G1, A1, P2, V2); the preintegration from 1 to 2, kept alive by the caller
class InertialResidual final : public nls::Residual {
public:
    explicit InertialResidual(std::shared_ptr<const imu::Preintegrated> p);
    int size() const override { return 9; }
    void evaluate(const double* const* blocks, double* e, double* const* jacobians) const override;

private:
    std::shared_ptr<const imu::Preintegrated> p_;
    Eigen::Matrix3d JRg_, JVg_, JPg_, JVa_, JPa_;
    double dt_;
};

// EdgeInertialGS: blocks (P1, V1, G, A, P2, V2, Rwg, scale)
class InertialGSResidual final : public nls::Residual {
public:
    explicit InertialGSResidual(std::shared_ptr<const imu::Preintegrated> p);
    int size() const override { return 9; }
    void evaluate(const double* const* blocks, double* e, double* const* jacobians) const override;

private:
    std::shared_ptr<const imu::Preintegrated> p_;
    Eigen::Matrix3d JRg_, JVg_, JPg_, JVa_, JPa_;
    double dt_;
};

// EdgeGyroRW / EdgeAccRW: blocks (b1, b2), e = b2 - b1
class BiasWalkResidual final : public nls::Residual {
public:
    int size() const override { return 3; }
    void evaluate(const double* const* blocks, double* e, double* const* jacobians) const override;
};

// EdgePriorAcc / EdgePriorGyro: block (b), e = prior - b, Jacobian +I (the original's)
class BiasPriorResidual final : public nls::Residual {
public:
    explicit BiasPriorResidual(const Eigen::Vector3d& prior) : prior_(prior) {}
    int size() const override { return 3; }
    void evaluate(const double* const* blocks, double* e, double* const* jacobians) const override;

private:
    Eigen::Vector3d prior_;
};

// The camera in the body (ImuCamPose's Rcb[i], tcb[i]): left T_cb, right Trl·T_cb (composed in double)
void cameraInBody(const geometry::SE3f& T_cb, const geometry::SE3f* Trl, Eigen::Matrix3d& Rcb, Eigen::Vector3d& tcb);

// EdgeMono on a body pose: blocks (body pose, point), e = uv - π(Rcb·Rwbᵀ·(Xw - twb) + tcb)
class BodyReprojectionResidual final : public nls::Residual {
public:
    BodyReprojectionResidual(const camera::KannalaBrandt8& camera, const Eigen::Matrix3d& Rcb, const Eigen::Vector3d& tcb,
                             const Eigen::Vector2d& uv);
    int size() const override { return 2; }
    void evaluate(const double* const* blocks, double* e, double* const* jacobians) const override;
    // The point's depth in this camera (isDepthPositive)
    double depth(const double* pose, const double* point) const;

private:
    const camera::KannalaBrandt8* camera_;
    Eigen::Matrix3d Rcb_, Rbc_;
    Eigen::Vector3d tcb_, tbc_;
    Eigen::Vector2d uv_;
};

}  // namespace ba_solver
