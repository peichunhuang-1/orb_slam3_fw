#pragma once
// KB8 reprojection of a map point in a keyframe's left or right camera, as an nls residual: ORB-SLAM3's
// EdgeSE3ProjectXYZ (left; Trl = identity) and EdgeSE3ProjectXYZToBody (right).
//   e = uv - π(Trl · Tcw · Xw)
// Blocks: the keyframe's Tcw (nls::SE3Left, δ = (ω, υ) applied as exp(δ)·Tcw, g2o's VertexSE3Expmap) and the
// point's world position (3). The Jacobians are g2o's linearizeOplus of those edges.
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "libs/nls/problem.hpp"
#include "types/camera/kannala_brandt.hpp"

namespace ba_solver {

class Kb8Reprojection final : public nls::Residual {
public:
    Kb8Reprojection(const camera::KannalaBrandt8& camera, const Eigen::Quaterniond& q_rl, const Eigen::Vector3d& t_rl,
                    const Eigen::Vector2d& uv)
        : camera_(camera), R_rl_(q_rl.toRotationMatrix()), t_rl_(t_rl), uv_(uv) {}

    int size() const override { return 2; }

    void evaluate(const double* const* x, double* e, double* const* J) const override {
        const Eigen::Quaterniond q(x[0][3], x[0][0], x[0][1], x[0][2]);
        const Eigen::Map<const Eigen::Vector3d> t(x[0] + 4), Xw(x[1]);
        const Eigen::Vector3d Xl = q * Xw + t;
        const Eigen::Vector3d Xr = R_rl_ * Xl + t_rl_;
        Eigen::Map<Eigen::Vector2d> E(e);
        E = uv_ - camera_.project_orb(Xr);
        if (!J[0] && !J[1]) return;
        const Eigen::Matrix<double, 2, 3> A = -camera_.project_jacobian_orb(Xr) * R_rl_;
        if (J[0]) {
            // d(exp(δ)·Xl)/dδ = [ -[Xl]x  I ]
            Eigen::Matrix<double, 3, 6> d;
            d << 0, Xl.z(), -Xl.y(), 1, 0, 0,
                 -Xl.z(), 0, Xl.x(), 0, 1, 0,
                 Xl.y(), -Xl.x(), 0, 0, 0, 1;
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> Jp(J[0]);
            Jp = A * d;
        }
        if (J[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> Jx(J[1]);
            Jx = A * q.toRotationMatrix();
        }
    }

    // The point's depth in this camera (isDepthPositive)
    double depth(const double* pose, const double* point) const {
        const Eigen::Quaterniond q(pose[3], pose[0], pose[1], pose[2]);
        const Eigen::Map<const Eigen::Vector3d> t(pose + 4), Xw(point);
        return (R_rl_ * (q * Xw + t) + t_rl_).z();
    }

private:
    const camera::KannalaBrandt8& camera_;
    Eigen::Matrix3d R_rl_;
    Eigen::Vector3d t_rl_;
    Eigen::Vector2d uv_;
};

}  // namespace ba_solver
