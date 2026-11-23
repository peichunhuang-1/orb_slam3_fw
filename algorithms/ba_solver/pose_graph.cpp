#include "algorithms/ba_solver/pose_graph.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/LU>
#include <Eigen/SVD>

#include "libs/nls/problem.hpp"
#include "types/geometry/so3.hpp"

namespace ba_solver {

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using geometry::SE3d;

constexpr double kDelta = 1e-9;  // g2o's BaseBinaryEdge::linearizeOplus
constexpr double kScalar = 1.0 / (2 * kDelta);

// ---- pose_graph: g2o's Sim3 with s = 1 (no normalization of the quaternion, as there)

struct Rigid {
    Eigen::Quaterniond r = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

Rigid operator*(const Rigid& a, const Rigid& b) { return {a.r * b.r, a.r * b.t + a.t}; }
Rigid inverse(const Rigid& a) { return {a.r.conjugate(), a.r.conjugate() * (-a.t)}; }

Rigid load(const double* x) { return {Eigen::Quaterniond(x[3], x[0], x[1], x[2]), Eigen::Vector3d(x[4], x[5], x[6])}; }
void store(const Rigid& T, double* x) {
    x[0] = T.r.x();
    x[1] = T.r.y();
    x[2] = T.r.z();
    x[3] = T.r.w();
    x[4] = T.t.x();
    x[5] = T.t.y();
    x[6] = T.t.z();
}

// g2o::Sim3(update) with σ = 0
Rigid expUpdate(const double* u) {
    const Eigen::Vector3d omega(u[0], u[1], u[2]), upsilon(u[3], u[4], u[5]);
    const double theta = omega.norm();
    const Eigen::Matrix3d Omega = geometry::hat(omega), Omega2 = Omega * Omega, I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R;
    double A, B;
    if (theta < 0.00001) {
        A = 1. / 2.;
        B = 1. / 6.;
        R = I + Omega + Omega2;
    } else {
        const double theta2 = theta * theta;
        A = (1 - std::cos(theta)) / theta2;
        B = (theta - std::sin(theta)) / (theta2 * theta);
        R = I + std::sin(theta) / theta * Omega + (1 - std::cos(theta)) / theta2 * Omega2;
    }
    const Eigen::Matrix3d W = A * Omega + B * Omega2 + I;
    return {Eigen::Quaterniond(R), W * upsilon};
}

Eigen::Vector3d deltaR(const Eigen::Matrix3d& R) {
    return Eigen::Vector3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
}

// g2o::Sim3::log with σ = 0 (the scale component, 0, left out)
Vector6d logRigid(const Rigid& T) {
    const Eigen::Matrix3d R = T.r.toRotationMatrix();
    const double d = 0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1);
    const double eps = 0.00001;
    Eigen::Vector3d omega;
    double A, B;
    if (d > 1 - eps) {
        omega = 0.5 * deltaR(R);
        A = 1. / 2.;
        B = 1. / 6.;
    } else {
        const double theta = std::acos(d), theta2 = theta * theta;
        omega = theta / (2 * std::sqrt(1 - d * d)) * deltaR(R);
        A = (1 - std::cos(theta)) / theta2;
        B = (theta - std::sin(theta)) / (theta2 * theta);
    }
    const Eigen::Matrix3d Omega = geometry::hat(omega);
    const Eigen::Matrix3d W = A * Omega + B * Omega * Omega + Eigen::Matrix3d::Identity();
    Vector6d res;
    res.head<3>() = omega;
    res.tail<3>() = W.lu().solve(T.t);
    return res;
}

// VertexSim3Expmap with _fix_scale: x ⊞ δ = Sim3(δ, σ = 0) · x
class RigidLeft final : public nls::Manifold {
public:
    int ambientSize() const override { return 7; }
    int tangentSize() const override { return 6; }
    void plus(const double* x, const double* delta, double* x_plus) const override {
        store(expUpdate(delta) * load(x), x_plus);
    }
};

// EdgeSim3: vertex 0 = i, vertex 1 = j, error log(Sji · Siw · Sjw⁻¹)
class PoseEdge final : public nls::Residual {
public:
    explicit PoseEdge(const SE3d& Sji) : Sji_{Sji.unitQuaternion(), Sji.translation()} {}
    int size() const override { return 6; }

    void evaluate(const double* const* x, double* e, double* const* J) const override {
        const Rigid Si = load(x[0]), Sj = load(x[1]);
        Eigen::Map<Vector6d>{e} = error(Si, Sj);
        for (int v = 0; v < 2; ++v) {
            if (!J[v]) continue;
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> Jv(J[v]);
            for (int d = 0; d < 6; ++d) {
                double u[6] = {0, 0, 0, 0, 0, 0};
                u[d] = kDelta;
                const Rigid plus = expUpdate(u) * (v == 0 ? Si : Sj);
                u[d] = -kDelta;
                const Rigid minus = expUpdate(u) * (v == 0 ? Si : Sj);
                const Vector6d ep = v == 0 ? error(plus, Sj) : error(Si, plus);
                const Vector6d em = v == 0 ? error(minus, Sj) : error(Si, minus);
                Jv.col(d) = kScalar * (ep - em);
            }
        }
    }

private:
    Vector6d error(const Rigid& Si, const Rigid& Sj) const { return logRigid(Sji_ * Si * inverse(Sj)); }
    Rigid Sji_;
};

// ---- pose_graph_4dof: ImuCamPose as the 4DoF vertex keeps it

using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;

// ORB-SLAM3's ExpSO3, LogSO3
Mat3 expSO3(double x, double y, double z) {
    const double d2 = x * x + y * y + z * z, d = std::sqrt(d2);
    Mat3 W;
    W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
    if (d < 1e-5) return geometry::normalizeRotation(Mat3(Mat3::Identity() + W + 0.5 * W * W));
    return geometry::normalizeRotation(Mat3(Mat3::Identity() + W * std::sin(d) / d + W * W * (1.0 - std::cos(d)) / d2));
}

Vec3 logSO3(const Mat3& R) {
    const double tr = R(0, 0) + R(1, 1) + R(2, 2);
    const Vec3 w((R(2, 1) - R(1, 2)) / 2, (R(0, 2) - R(2, 0)) / 2, (R(1, 0) - R(0, 1)) / 2);
    const double costheta = (tr - 1.0) * 0.5f;
    if (costheta > 1 || costheta < -1) return w;
    const double theta = std::acos(costheta), s = std::sin(theta);
    if (std::fabs(s) < 1e-5) return w;
    return theta * w / s;
}

// The vertex's state, stored in the block: DR (9, column-major), twb (3), Rcw (9), tcw (3), its (1). Rcw and tcw
// are kept, not derived: until the first update they are the keyframe's own (ImuCamPose(pKF) takes the camera
// pose and the IMU pose separately, both from floats).
constexpr int kDR = 0, kTwb = 9, kRcw = 12, kTcw = 21, kIts = 24, kSize4 = 25;

struct YawState {
    Mat3 DR, Rcw;
    Vec3 twb, tcw;
    double its;
};

YawState loadYaw(const double* x) {
    YawState s;
    s.DR = Eigen::Map<const Mat3>(x + kDR);
    s.twb = Eigen::Map<const Vec3>(x + kTwb);
    s.Rcw = Eigen::Map<const Mat3>(x + kRcw);
    s.tcw = Eigen::Map<const Vec3>(x + kTcw);
    s.its = x[kIts];
    return s;
}

void storeYaw(const YawState& s, double* x) {
    Eigen::Map<Mat3>(x + kDR) = s.DR;
    Eigen::Map<Vec3>(x + kTwb) = s.twb;
    Eigen::Map<Mat3>(x + kRcw) = s.Rcw;
    Eigen::Map<Vec3>(x + kTcw) = s.tcw;
    x[kIts] = s.its;
}

// VertexPose4DoF::oplusImpl → ImuCamPose::UpdateW with (0, 0, δ0, δ1, δ2, δ3). Per vertex: Rwb0 (the body
// rotation it was made with) and the camera-body extrinsics.
class YawPose final : public nls::Manifold {
public:
    YawPose(const Mat3& Rwb0, const Mat3& Rcb, const Vec3& tcb) : Rwb0_(Rwb0), Rcb_(Rcb), tcb_(tcb) {}
    int ambientSize() const override { return kSize4; }
    int tangentSize() const override { return 4; }
    void plus(const double* x, const double* delta, double* x_plus) const override {
        YawState s = loadYaw(x);
        const Mat3 dR = expSO3(0, 0, delta[0]);
        s.DR = dR * s.DR;
        const Mat3 Rwb = s.DR * Rwb0_;
        s.twb += Vec3(delta[1], delta[2], delta[3]);
        s.its += 1;
        if (s.its >= 5) {
            s.DR(0, 2) = 0.0;
            s.DR(1, 2) = 0.0;
            s.DR(2, 0) = 0.0;
            s.DR(2, 1) = 0.0;
            s.DR = geometry::normalizeRotation(s.DR);
            s.its = 0;
        }
        const Mat3 Rbw = Rwb.transpose();
        const Vec3 tbw = -Rbw * s.twb;
        s.Rcw = Rcb_ * Rbw;
        s.tcw = Rcb_ * tbw + tcb_;
        storeYaw(s, x_plus);
    }

private:
    Mat3 Rwb0_, Rcb_;
    Vec3 tcb_;
};

// Edge4DoF: vertex 0 = i, vertex 1 = j
class YawEdge final : public nls::Residual {
public:
    YawEdge(const SE3d& Tij, std::shared_ptr<const YawPose> mi, std::shared_ptr<const YawPose> mj)
        : dR_(Tij.unitQuaternion().toRotationMatrix()), dt_(Tij.translation()), m_{std::move(mi), std::move(mj)} {}
    int size() const override { return 6; }

    void evaluate(const double* const* x, double* e, double* const* J) const override {
        Eigen::Map<Vector6d>{e} = error(x[0], x[1]);
        for (int v = 0; v < 2; ++v) {
            if (!J[v]) continue;
            Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> Jv(J[v]);
            std::array<double, kSize4> plus, minus;
            for (int d = 0; d < 4; ++d) {
                double u[4] = {0, 0, 0, 0};
                u[d] = kDelta;
                m_[v]->plus(x[v], u, plus.data());
                u[d] = -kDelta;
                m_[v]->plus(x[v], u, minus.data());
                const Vector6d ep = v == 0 ? error(plus.data(), x[1]) : error(x[0], plus.data());
                const Vector6d em = v == 0 ? error(minus.data(), x[1]) : error(x[0], minus.data());
                Jv.col(d) = kScalar * (ep - em);
            }
        }
    }

private:
    Vector6d error(const double* xi, const double* xj) const {
        const Mat3 Ri = Eigen::Map<const Mat3>(xi + kRcw), Rj = Eigen::Map<const Mat3>(xj + kRcw);
        const Vec3 ti = Eigen::Map<const Vec3>(xi + kTcw), tj = Eigen::Map<const Vec3>(xj + kTcw);
        Vector6d err;
        err << logSO3(Ri * Rj.transpose() * dR_.transpose()), Ri * (-Rj.transpose() * tj) + ti - dt_;
        return err;
    }

    Mat3 dR_;
    Vec3 dt_;
    std::shared_ptr<const YawPose> m_[2];
};

ba::Result makeResult(const ba::Problem& p, const nls::Summary& s) {
    ba::Result r;
    r.seq = p.seq;
    r.kind = p.kind;
    r.iterations = s.iterations;
    r.ok = true;
    return r;
}

}  // namespace

ba::Result solvePoseGraph(const ba::Problem& p) {
    const std::size_t n = p.keyframes.size();
    nls::Problem prob;
    const auto manifold = std::make_shared<RigidLeft>();
    std::vector<std::array<double, 7>> x(n);
    std::vector<int> block(n);
    for (std::size_t i = 0; i < n; ++i) {
        const SE3d& T = p.pose_graph[i].Tcw;
        store({T.unitQuaternion(), T.translation()}, x[i].data());
        block[i] = prob.addBlock(x[i].data(), manifold);
        prob.setFixed(block[i], p.keyframes[i].fixed);
    }
    for (const ba::PoseGraphEdge& e : p.edges)
        prob.addResidual(std::make_shared<PoseEdge>(e.T), {block[std::size_t(e.i)], block[std::size_t(e.j)]});

    nls::Options opt;
    opt.max_iterations = p.iterations;
    opt.initial_lambda = p.initial_lambda;
    ba::Result r = makeResult(p, prob.solve(opt));
    r.poses.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Rigid T = load(x[i].data());
        r.poses.emplace_back(T.r, T.t);
    }
    return r;
}

ba::Result solvePoseGraph4DoF(const ba::Problem& p) {
    const std::size_t n = p.keyframes.size();
    nls::Problem prob;
    std::vector<std::array<double, kSize4>> x(n);
    std::vector<std::shared_ptr<const YawPose>> manifolds(n);
    std::vector<int> block(n);
    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& k = p.keyframes[i];
        const Mat3 Rcb = k.T_cb.rotationMatrix().cast<double>();
        const Vec3 tcb = k.T_cb.translation().cast<double>();
        YawState s;
        Mat3 Rwb;
        if (p.pose_graph[i].corrected) {
            // ImuCamPose(Rwc, twc, pKF) from the corrected pose
            const SE3d& Tcw = p.pose_graph[i].Tcw;
            const Eigen::Quaterniond qwc = Tcw.unitQuaternion().conjugate();
            const Mat3 Rwc = qwc.toRotationMatrix();
            const Vec3 twc = qwc * (-Tcw.translation());
            s.twb = Rwc * tcb + twc;
            Rwb = Rwc * Rcb;
            s.Rcw = Rwc.transpose();
            s.tcw = -s.Rcw * twc;
        } else {
            // ImuCamPose(pKF): the keyframe's IMU pose (GetImuPosition, GetImuRotation) and its camera pose
            const geometry::SE3f Twc = k.Tcw.inverse();
            const Eigen::Vector3f Owb = Twc.rotationMatrix() * k.T_cb.translation() + Twc.translation();
            s.twb = Owb.cast<double>();
            Rwb = (Twc * k.T_cb).rotationMatrix().cast<double>();
            s.Rcw = k.Tcw.rotationMatrix().cast<double>();
            s.tcw = k.Tcw.translation().cast<double>();
        }
        s.DR.setIdentity();
        s.its = 0;
        storeYaw(s, x[i].data());
        manifolds[i] = std::make_shared<YawPose>(Rwb, Rcb, tcb);
        block[i] = prob.addBlock(x[i].data(), manifolds[i]);
        prob.setFixed(block[i], k.fixed);
    }
    Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity();
    info(0, 0) = 1e3;
    info(1, 1) = 1e3;
    for (const ba::PoseGraphEdge& e : p.edges) {
        const std::size_t i = std::size_t(e.i), j = std::size_t(e.j);
        prob.addResidual(std::make_shared<YawEdge>(e.T, manifolds[i], manifolds[j]), {block[i], block[j]}, info);
    }

    nls::Options opt;
    opt.max_iterations = p.iterations;
    opt.initial_lambda = p.initial_lambda;
    ba::Result r = makeResult(p, prob.solve(opt));
    r.poses.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const YawState s = loadYaw(x[i].data());
        r.poses.emplace_back(s.Rcw, s.tcw);
    }
    return r;
}

}  // namespace ba_solver
