#include "algorithms/ba_solver/inertial_residuals.hpp"

#include <Eigen/Eigenvalues>

#include "types/geometry/so3.hpp"

namespace ba_solver {

namespace {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
template <int C>
using RowJ = Eigen::Map<Eigen::Matrix<double, 9, C, Eigen::RowMajor>>;

// Writes expr into the matrix at p
template <class M, class E>
void assign(double* p, const E& expr) {
    Eigen::Map<M> m(p);
    m = expr;
}

// ExpSO3: Rodrigues, then orthonormalized (NormalizeRotation)
Mat3 expSO3(const Vec3& w) { return geometry::normalizeRotation(geometry::exp(w)); }

imu::Bias biasOf(const double* g, const double* a) {
    imu::Bias b;
    b.g = Eigen::Vector3f(float(g[0]), float(g[1]), float(g[2]));
    b.a = Eigen::Vector3f(float(a[0]), float(a[1]), float(a[2]));
    return b;
}

struct Pose {
    Eigen::Map<const Mat3> R;
    Eigen::Map<const Vec3> t;
    explicit Pose(const double* x) : R(x), t(x + 9) {}
};

}  // namespace

void ImuPoseManifold::plus(const double* x, const double* delta, double* x_plus) const {
    const Mat3 R = Eigen::Map<const Mat3>(x);
    const Vec3 t = Eigen::Map<const Vec3>(x + 9);
    const Vec3 ur(delta[0], delta[1], delta[2]), ut(delta[3], delta[4], delta[5]);
    assign<Vec3>(x_plus + 9, t + R * ut);
    assign<Mat3>(x_plus, R * expSO3(ur));
}

void GravityDirManifold::plus(const double* x, const double* delta, double* x_plus) const {
    const Mat3 R = Eigen::Map<const Mat3>(x);
    assign<Mat3>(x_plus, R * expSO3(Vec3(delta[0], delta[1], 0.0)));
}

void ScaleManifold::plus(const double* x, const double* delta, double* x_plus) const {
    x_plus[0] = x[0] * std::exp(delta[0]);
}

void imuPoseFrom(const geometry::SE3f& Tcw, const geometry::SE3f& T_cb, double x[12]) {
    const geometry::SE3f Twc = Tcw.inverse();
    const Eigen::Matrix3f Rwb = (Twc * T_cb).rotationMatrix();
    const Eigen::Vector3f twb = Twc.rotationMatrix() * T_cb.translation() + Twc.translation();
    assign<Mat3>(x, Rwb.cast<double>());
    assign<Vec3>(x + 9, twb.cast<double>());
}

geometry::SE3f cameraPoseOf(const double x[12], const geometry::SE3f& T_cb) {
    const Pose P(x);
    const Mat3 Rcb = T_cb.rotationMatrix().cast<double>();
    const Vec3 tcb = T_cb.translation().cast<double>();
    const Mat3 Rbw = P.R.transpose();
    const Vec3 tbw = -Rbw * P.t;
    const Mat3 Rcw = Rcb * Rbw;
    const Vec3 tcw = Rcb * tbw + tcb;
    return geometry::SE3f(Eigen::Matrix3f(Rcw.cast<float>()), Eigen::Vector3f(tcw.cast<float>()));
}

Eigen::Matrix<double, 9, 9> inertialInformation(const imu::Preintegrated& p) {
    using M9 = Eigen::Matrix<double, 9, 9>;
    M9 info = p.C.block<9, 9>(0, 0).cast<double>().inverse();
    info = (info + info.transpose()) / 2;
    Eigen::SelfAdjointEigenSolver<M9> es(info);
    Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
    for (int i = 0; i < 9; ++i)
        if (eigs[i] < 1e-12) eigs[i] = 0;
    return es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
}

// ---- EdgeInertial

InertialResidual::InertialResidual(std::shared_ptr<const imu::Preintegrated> p)
    : p_(std::move(p)),
      JRg_(p_->JRg.cast<double>()), JVg_(p_->JVg.cast<double>()), JPg_(p_->JPg.cast<double>()),
      JVa_(p_->JVa.cast<double>()), JPa_(p_->JPa.cast<double>()), dt_(p_->dT) {}

void InertialResidual::evaluate(const double* const* x, double* e, double* const* J) const {
    const Pose P1(x[0]), P2(x[4]);
    const Eigen::Map<const Vec3> v1(x[1]), v2(x[5]);
    const imu::Bias b1 = biasOf(x[2], x[3]);
    const Vec3 g(0, 0, -kGravity);
    const Mat3 dR = p_->deltaRotation(b1).cast<double>();
    const Vec3 dV = p_->deltaVelocity(b1).cast<double>();
    const Vec3 dP = p_->deltaPosition(b1).cast<double>();
    const Mat3 Rbw1 = P1.R.transpose();
    const Mat3 eR = dR.transpose() * Rbw1 * P2.R;
    const Vec3 er = geometry::log(eR);
    Eigen::Map<Eigen::Matrix<double, 9, 1>> E(e);
    E.segment<3>(0) = er;
    E.segment<3>(3) = Rbw1 * (v2 - v1 - g * dt_) - dV;
    E.segment<3>(6) = Rbw1 * (P2.t - P1.t - v1 * dt_ - g * dt_ * dt_ / 2) - dP;

    const Mat3 invJr = geometry::inverseRightJacobian(er);
    if (J[0]) {
        RowJ<6> j(J[0]);
        j.setZero();
        j.block<3, 3>(0, 0) = -invJr * P2.R.transpose() * P1.R;
        j.block<3, 3>(3, 0) = geometry::hat(Vec3(Rbw1 * (v2 - v1 - g * dt_)));
        j.block<3, 3>(6, 0) = geometry::hat(Vec3(Rbw1 * (P2.t - P1.t - v1 * dt_ - 0.5 * g * dt_ * dt_)));
        j.block<3, 3>(6, 3) = -Mat3::Identity();
    }
    if (J[1]) {
        RowJ<3> j(J[1]);
        j.setZero();
        j.block<3, 3>(3, 0) = -Rbw1;
        j.block<3, 3>(6, 0) = -Rbw1 * dt_;
    }
    if (J[2]) {
        const Vec3 dbg = (b1.g - p_->bias().g).cast<double>();  // GetDeltaBias(b1)
        RowJ<3> j(J[2]);
        j.setZero();
        j.block<3, 3>(0, 0) = -invJr * eR.transpose() * geometry::rightJacobian(Vec3(JRg_ * dbg)) * JRg_;
        j.block<3, 3>(3, 0) = -JVg_;
        j.block<3, 3>(6, 0) = -JPg_;
    }
    if (J[3]) {
        RowJ<3> j(J[3]);
        j.setZero();
        j.block<3, 3>(3, 0) = -JVa_;
        j.block<3, 3>(6, 0) = -JPa_;
    }
    if (J[4]) {
        RowJ<6> j(J[4]);
        j.setZero();
        j.block<3, 3>(0, 0) = invJr;
        j.block<3, 3>(6, 3) = Rbw1 * P2.R;
    }
    if (J[5]) {
        RowJ<3> j(J[5]);
        j.setZero();
        j.block<3, 3>(3, 0) = Rbw1;
    }
}

// ---- EdgeInertialGS

InertialGSResidual::InertialGSResidual(std::shared_ptr<const imu::Preintegrated> p)
    : p_(std::move(p)),
      JRg_(p_->JRg.cast<double>()), JVg_(p_->JVg.cast<double>()), JPg_(p_->JPg.cast<double>()),
      JVa_(p_->JVa.cast<double>()), JPa_(p_->JPa.cast<double>()), dt_(p_->dT) {}

void InertialGSResidual::evaluate(const double* const* x, double* e, double* const* J) const {
    const Pose P1(x[0]), P2(x[4]);
    const Eigen::Map<const Vec3> v1(x[1]), v2(x[5]);
    const imu::Bias b = biasOf(x[2], x[3]);
    const Eigen::Map<const Mat3> Rwg(x[6]);
    const double s = x[7][0];
    const Vec3 gI(0, 0, -kGravity);
    const Vec3 g = Rwg * gI;
    const Mat3 dR = p_->deltaRotation(b).cast<double>();
    const Vec3 dV = p_->deltaVelocity(b).cast<double>();
    const Vec3 dP = p_->deltaPosition(b).cast<double>();
    const Mat3 Rbw1 = P1.R.transpose();
    const Mat3 eR = dR.transpose() * Rbw1 * P2.R;
    const Vec3 er = geometry::log(eR);
    Eigen::Map<Eigen::Matrix<double, 9, 1>> E(e);
    E.segment<3>(0) = er;
    E.segment<3>(3) = Rbw1 * (s * (v2 - v1) - g * dt_) - dV;
    E.segment<3>(6) = Rbw1 * (s * (P2.t - P1.t - v1 * dt_) - g * dt_ * dt_ / 2) - dP;

    const Mat3 invJr = geometry::inverseRightJacobian(er);
    if (J[0]) {
        RowJ<6> j(J[0]);
        j.setZero();
        j.block<3, 3>(0, 0) = -invJr * P2.R.transpose() * P1.R;
        j.block<3, 3>(3, 0) = geometry::hat(Vec3(Rbw1 * (s * (v2 - v1) - g * dt_)));
        j.block<3, 3>(6, 0) = geometry::hat(Vec3(Rbw1 * (s * (P2.t - P1.t - v1 * dt_) - 0.5 * g * dt_ * dt_)));
        j.block<3, 3>(6, 3) = -s * Mat3::Identity();
    }
    if (J[1]) {
        RowJ<3> j(J[1]);
        j.setZero();
        j.block<3, 3>(3, 0) = -s * Rbw1;
        j.block<3, 3>(6, 0) = -s * Rbw1 * dt_;
    }
    if (J[2]) {
        const Vec3 dbg = (b.g - p_->bias().g).cast<double>();
        RowJ<3> j(J[2]);
        j.setZero();
        j.block<3, 3>(0, 0) = -invJr * eR.transpose() * geometry::rightJacobian(Vec3(JRg_ * dbg)) * JRg_;
        j.block<3, 3>(3, 0) = -JVg_;
        j.block<3, 3>(6, 0) = -JPg_;
    }
    if (J[3]) {
        RowJ<3> j(J[3]);
        j.setZero();
        j.block<3, 3>(3, 0) = -JVa_;
        j.block<3, 3>(6, 0) = -JPa_;
    }
    if (J[4]) {
        RowJ<6> j(J[4]);
        j.setZero();
        j.block<3, 3>(0, 0) = invJr;
        j.block<3, 3>(6, 3) = s * Rbw1 * P2.R;
    }
    if (J[5]) {
        RowJ<3> j(J[5]);
        j.setZero();
        j.block<3, 3>(3, 0) = s * Rbw1;
    }
    if (J[6]) {
        Eigen::Matrix<double, 3, 2> Gm = Eigen::Matrix<double, 3, 2>::Zero();
        Gm(0, 1) = -kGravity;
        Gm(1, 0) = kGravity;
        const Eigen::Matrix<double, 3, 2> dGdTheta = Rwg * Gm;
        RowJ<2> j(J[6]);
        j.setZero();
        j.block<3, 2>(3, 0) = -Rbw1 * dGdTheta * dt_;
        j.block<3, 2>(6, 0) = -0.5 * Rbw1 * dGdTheta * dt_ * dt_;
    }
    if (J[7]) {
        Eigen::Map<Eigen::Matrix<double, 9, 1>> j(J[7]);  // one column: row-major is column-major
        j.setZero();
        j.block<3, 1>(3, 0) = Rbw1 * (v2 - v1);
        j.block<3, 1>(6, 0) = Rbw1 * (P2.t - P1.t - v1 * dt_);
    }
}

// ---- random walk, priors

void BiasWalkResidual::evaluate(const double* const* x, double* e, double* const* J) const {
    for (int i = 0; i < 3; ++i) e[i] = x[1][i] - x[0][i];
    if (J[0]) assign<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(J[0], -Mat3::Identity());
    if (J[1]) assign<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(J[1], Mat3::Identity());
}

void BiasPriorResidual::evaluate(const double* const* x, double* e, double* const* J) const {
    for (int i = 0; i < 3; ++i) e[i] = prior_[i] - x[0][i];
    if (J[0]) assign<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(J[0], Mat3::Identity());
}

// ---- EdgeMono

void cameraInBody(const geometry::SE3f& T_cb, const geometry::SE3f* Trl, Mat3& Rcb, Vec3& tcb) {
    Rcb = T_cb.rotationMatrix().cast<double>();
    tcb = T_cb.translation().cast<double>();
    if (!Trl) return;
    const Eigen::Matrix4d T = Trl->matrix().cast<double>();
    const Mat3 R = T.block<3, 3>(0, 0);
    tcb = R * tcb + T.block<3, 1>(0, 3);
    Rcb = R * Rcb;
}

BodyReprojectionResidual::BodyReprojectionResidual(const camera::KannalaBrandt8& camera, const Mat3& Rcb,
                                                   const Vec3& tcb, const Eigen::Vector2d& uv)
    : camera_(&camera), Rcb_(Rcb), Rbc_(Rcb.transpose()), tcb_(tcb), tbc_(-Rcb.transpose() * tcb), uv_(uv) {}

void BodyReprojectionResidual::evaluate(const double* const* x, double* e, double* const* J) const {
    const Pose P(x[0]);
    const Eigen::Map<const Vec3> Xw(x[1]);
    const Mat3 Rbw = P.R.transpose();
    const Mat3 Rcw = Rcb_ * Rbw;
    const Vec3 tcw = Rcb_ * (-Rbw * P.t) + tcb_;
    const Vec3 Xc = Rcw * Xw + tcw;
    assign<Eigen::Vector2d>(e, uv_ - camera_->project_orb(Xc));
    if (!J[0] && !J[1]) return;
    const Eigen::Matrix<double, 2, 3> pj = camera_->project_jacobian_orb(Xc);
    if (J[0]) {
        const Vec3 Xb = Rbc_ * Xc + tbc_;
        Eigen::Matrix<double, 3, 6> d;
        d << 0.0, Xb.z(), -Xb.y(), 1.0, 0.0, 0.0,
             -Xb.z(), 0.0, Xb.x(), 0.0, 1.0, 0.0,
             Xb.y(), -Xb.x(), 0.0, 0.0, 0.0, 1.0;
        assign<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>(J[0], pj * Rcb_ * d);
    }
    if (J[1]) assign<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>>(J[1], -pj * Rcw);
}

}  // namespace ba_solver
