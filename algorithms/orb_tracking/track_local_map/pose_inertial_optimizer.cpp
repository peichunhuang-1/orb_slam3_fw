#include "algorithms/orb_tracking/track_local_map/pose_inertial_optimizer.hpp"

#include <cmath>
#include <optional>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include "types/geometry/so3.hpp"

namespace orb_tracking {
namespace {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Matrix9d = Eigen::Matrix<double, 9, 9>;

constexpr double kGravity = 9.81;  // IMU::GRAVITY_VALUE

// ExpSO3: Rodrigues, then orthonormalized (NormalizeRotation)
Mat3 expSO3(const Vec3& w) { return geometry::normalizeRotation(geometry::exp(w)); }

// g2o's RobustKernelHuber::setDelta: the square kept in float
struct Huber {
    double delta;
    float dsqr;
    explicit Huber(double d) : delta(d), dsqr(float(delta * delta)) {}
    double weight(double e) const { return e <= dsqr ? 1. : delta / std::sqrt(e); }  // rho'(e)
};

// Information as EdgeInertial / ConstraintPoseImu make it: eigenvalues below 1e-12 set to 0
template <int N>
Eigen::Matrix<double, N, N> clampEigen(const Eigen::Matrix<double, N, N>& M) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> es(M);
    Eigen::Matrix<double, N, 1> eigs = es.eigenvalues();
    for (int i = 0; i < N; ++i)
        if (eigs[i] < 1e-12) eigs[i] = 0;
    return es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
}

// VertexPose's estimate (ImuCamPose): the body pose and both cameras' poses derived from it
struct BodyPose {
    Mat3 Rwb;
    Vec3 twb;
    Mat3 Rcw[2], Rcb[2], Rbc[2];
    Vec3 tcw[2], tcb[2], tbc[2];

    explicit BodyPose(const tracking::Frame& f) {
        twb = f.imuPosition().cast<double>();
        Rwb = f.imuRotation().cast<double>();
        tcw[0] = f.pose().translation().cast<double>();
        Rcw[0] = f.pose().rotationMatrix().cast<double>();
        tcb[0] = f.T_cb.translation().cast<double>();
        Rcb[0] = f.T_cb.rotationMatrix().cast<double>();
        Rbc[0] = Rcb[0].transpose();
        tbc[0] = f.T_cb.inverse().translation().cast<double>();
        const Eigen::Matrix4d Trl = f.Trl.matrix().cast<double>();
        const Mat3 R = Trl.block<3, 3>(0, 0);
        const Vec3 t = Trl.block<3, 1>(0, 3);
        Rcw[1] = R * Rcw[0];
        tcw[1] = R * tcw[0] + t;
        tcb[1] = R * tcb[0] + t;
        Rcb[1] = R * Rcb[0];
        Rbc[1] = Rcb[1].transpose();
        tbc[1] = -Rbc[1] * tcb[1];
    }
    // A fixed keyframe: only the body pose is read (no observation edges on it)
    explicit BodyPose(const ImuState& s) : Rwb(s.Rwb.cast<double>()), twb(s.twb.cast<double>()) {}

    // ImuCamPose::Update: in the body frame (its periodic NormalizeRotation discards its result)
    void update(const double* pu) {
        const Vec3 ur(pu[0], pu[1], pu[2]), ut(pu[3], pu[4], pu[5]);
        twb += Rwb * ut;
        Rwb = Rwb * expSO3(ur);
        const Mat3 Rbw = Rwb.transpose();
        const Vec3 tbw = -Rbw * twb;
        for (int i = 0; i < 2; ++i) {
            Rcw[i] = Rcb[i] * Rbw;
            tcw[i] = Rcb[i] * tbw + tcb[i];
        }
    }
};

struct Vertex {
    BodyPose pose;
    Vec3 v, bg, ba;
    int offset = -1;  // first column in the system; -1: fixed
};

struct MonoEdge {
    int index = 0, cam = 0;
    Vec3 Xw;
    Eigen::Vector2d obs, error = Eigen::Vector2d::Zero();
    Eigen::Matrix2d information;
    bool active = true, robust = true;
    double chi2() const { return error.dot(information * error); }
};

// The system and its edges
class Problem {
public:
    Problem(const camera::StereoRig& rig, Vertex& cur, Vertex& prev, const imu::Preintegrated& inertial,
            const imu::Preintegrated& walks)
        : rig_(rig), cur_(cur), prev_(prev), pint_(inertial) {
        dim_ = prev_.offset >= 0 ? 30 : 15;
        JRg_ = pint_.JRg.cast<double>();
        JVg_ = pint_.JVg.cast<double>();
        JPg_ = pint_.JPg.cast<double>();
        JVa_ = pint_.JVa.cast<double>();
        JPa_ = pint_.JPa.cast<double>();
        dt_ = pint_.dT;
        g_ << 0, 0, -kGravity;
        Matrix9d info = pint_.C.block<9, 9>(0, 0).cast<double>().inverse();
        info = (info + info.transpose()) / 2;
        info_inertial_ = clampEigen<9>(info);
        info_g_ = walks.C.block<3, 3>(9, 9).cast<double>().inverse();
        info_a_ = walks.C.block<3, 3>(12, 12).cast<double>().inverse();
    }

    std::vector<MonoEdge> mono;
    std::optional<tracking::PoseImuPrior> prior;  // EdgePriorPoseImu on prev (LastFrame)

    void computeError(MonoEdge& e) const {
        const Vec3 Xc = cur_.pose.Rcw[e.cam] * e.Xw + cur_.pose.tcw[e.cam];
        e.error = e.obs - rig_.camera(e.cam).project_orb(Xc);
    }
    bool depthPositive(const MonoEdge& e) const {
        return (cur_.pose.Rcw[e.cam].row(2) * e.Xw + cur_.pose.tcw[e.cam](2)) > 0.0;
    }

    // OptimizationAlgorithmGaussNewton, `iterations` times: errors, system, LDLT, update
    void optimize(int iterations) {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(dim_);
        for (int it = 0; it < iterations; ++it) {
            computeActiveErrors();
            Eigen::MatrixXd H;
            Eigen::VectorXd b;
            buildSystem(H, b);
            Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
            const bool ok = ldlt.isPositive();
            if (ok) x = ldlt.solve(b);
            update(x);
            if (!ok) break;
        }
    }

    // Jacobian of a mono edge wrt the current pose (EdgeMonoOnlyPose::linearizeOplus)
    Eigen::Matrix<double, 2, 6> monoJacobian(const MonoEdge& e) const {
        const BodyPose& P = cur_.pose;
        const Vec3 Xc = P.Rcw[e.cam] * e.Xw + P.tcw[e.cam];
        const Vec3 Xb = P.Rbc[e.cam] * Xc + P.tbc[e.cam];
        const Eigen::Matrix<double, 2, 3> proj_jac = rig_.camera(e.cam).project_jacobian_orb(Xc);
        Eigen::Matrix<double, 3, 6> d;
        const double x = Xb(0), y = Xb(1), z = Xb(2);
        d << 0.0, z, -y, 1.0, 0.0, 0.0,
             -z, 0.0, x, 0.0, 1.0, 0.0,
             y, -x, 0.0, 0.0, 0.0, 1.0;
        return proj_jac * P.Rcb[e.cam] * d;
    }

    // EdgeInertial: error and the Jacobians wrt (P1, V1, G1, A1, P2, V2), 1 = prev, 2 = cur
    Eigen::Matrix<double, 9, 1> inertialError() const {
        const imu::Bias b1 = biasOf(prev_);
        const Mat3 dR = pint_.deltaRotation(b1).cast<double>();
        const Vec3 dV = pint_.deltaVelocity(b1).cast<double>();
        const Vec3 dP = pint_.deltaPosition(b1).cast<double>();
        const BodyPose &P1 = prev_.pose, &P2 = cur_.pose;
        const Vec3 er = geometry::log(dR.transpose() * P1.Rwb.transpose() * P2.Rwb);
        const Vec3 ev = P1.Rwb.transpose() * (cur_.v - prev_.v - g_ * dt_) - dV;
        const Vec3 ep = P1.Rwb.transpose() * (P2.twb - P1.twb - prev_.v * dt_ - g_ * dt_ * dt_ / 2) - dP;
        Eigen::Matrix<double, 9, 1> e;
        e << er, ev, ep;
        return e;
    }
    struct InertialJacobians {
        Eigen::Matrix<double, 9, 6> P1, P2;
        Eigen::Matrix<double, 9, 3> V1, G1, A1, V2;
    };
    InertialJacobians inertialJacobians() const {
        const imu::Bias b1 = biasOf(prev_);
        const Vec3 dbg = (b1.g - pint_.bias().g).cast<double>();  // GetDeltaBias(b1)
        const Mat3 Rwb1 = prev_.pose.Rwb, Rbw1 = Rwb1.transpose(), Rwb2 = cur_.pose.Rwb;
        const Mat3 dR = pint_.deltaRotation(b1).cast<double>();
        const Mat3 eR = dR.transpose() * Rbw1 * Rwb2;
        const Vec3 er = geometry::log(eR);
        const Mat3 invJr = geometry::inverseRightJacobian(er);
        InertialJacobians J;
        J.P1.setZero();
        J.P1.block<3, 3>(0, 0) = -invJr * Rwb2.transpose() * Rwb1;
        J.P1.block<3, 3>(3, 0) = geometry::hat(Rbw1 * (cur_.v - prev_.v - g_ * dt_));
        J.P1.block<3, 3>(6, 0) = geometry::hat(Rbw1 * (cur_.pose.twb - prev_.pose.twb - prev_.v * dt_ - 0.5 * g_ * dt_ * dt_));
        J.P1.block<3, 3>(6, 3) = -Mat3::Identity();
        J.V1.setZero();
        J.V1.block<3, 3>(3, 0) = -Rbw1;
        J.V1.block<3, 3>(6, 0) = -Rbw1 * dt_;
        J.G1.setZero();
        J.G1.block<3, 3>(0, 0) = -invJr * eR.transpose() * geometry::rightJacobian(Vec3(JRg_ * dbg)) * JRg_;
        J.G1.block<3, 3>(3, 0) = -JVg_;
        J.G1.block<3, 3>(6, 0) = -JPg_;
        J.A1.setZero();
        J.A1.block<3, 3>(3, 0) = -JVa_;
        J.A1.block<3, 3>(6, 0) = -JPa_;
        J.P2.setZero();
        J.P2.block<3, 3>(0, 0) = invJr;
        J.P2.block<3, 3>(6, 3) = Rbw1 * Rwb2;
        J.V2.setZero();
        J.V2.block<3, 3>(3, 0) = Rbw1;
        return J;
    }

    // EdgePriorPoseImu on prev: error (rotation, translation, velocity, gyro, acc) and Jacobians
    Eigen::Matrix<double, 15, 1> priorError() const {
        const tracking::PoseImuPrior& c = *prior;
        Eigen::Matrix<double, 15, 1> e;
        e << geometry::log(Mat3(c.Rwb.transpose() * prev_.pose.Rwb)), c.Rwb.transpose() * (prev_.pose.twb - c.twb),
            prev_.v - c.vwb, prev_.bg - c.bg, prev_.ba - c.ba;
        return e;
    }
    Eigen::Matrix<double, 15, 15> priorJacobian() const {  // columns P (6), V, G, A
        const tracking::PoseImuPrior& c = *prior;
        const Vec3 er = geometry::log(Mat3(c.Rwb.transpose() * prev_.pose.Rwb));
        Eigen::Matrix<double, 15, 15> J = Eigen::Matrix<double, 15, 15>::Zero();
        J.block<3, 3>(0, 0) = geometry::inverseRightJacobian(er);
        J.block<3, 3>(3, 3) = c.Rwb.transpose() * prev_.pose.Rwb;
        J.block<3, 3>(6, 6) = Mat3::Identity();
        J.block<3, 3>(9, 9) = Mat3::Identity();
        J.block<3, 3>(12, 12) = Mat3::Identity();
        return J;
    }

    const Matrix9d& inertialInformation() const { return info_inertial_; }
    const Mat3& gyroInformation() const { return info_g_; }
    const Mat3& accInformation() const { return info_a_; }

private:
    static imu::Bias biasOf(const Vertex& v) {
        imu::Bias b;
        b.g = v.bg.cast<float>();
        b.a = v.ba.cast<float>();
        return b;
    }

    void computeActiveErrors() {
        for (MonoEdge& e : mono)
            if (e.active) computeError(e);
        e_inertial_ = inertialError();
        e_g_ = cur_.bg - prev_.bg;
        e_a_ = cur_.ba - prev_.ba;
        if (prior) e_prior_ = priorError();
    }

    // Adds one edge: blocks (offset, Jacobian) of its free vertices, its information W (robust-weighted) and r =
    // -W·e (BaseMultiEdge::computeQuadraticForm)
    struct Block {
        int offset;
        Eigen::MatrixXd J;
    };
    static void add(Eigen::MatrixXd& H, Eigen::VectorXd& b, const std::vector<Block>& blocks, const Eigen::MatrixXd& W,
                    const Eigen::VectorXd& r) {
        for (std::size_t i = 0; i < blocks.size(); ++i) {
            const Block& A = blocks[i];
            if (A.offset < 0) continue;
            const Eigen::MatrixXd AtO = A.J.transpose() * W;
            b.segment(A.offset, A.J.cols()).noalias() += A.J.transpose() * r;
            H.block(A.offset, A.offset, A.J.cols(), A.J.cols()).noalias() += AtO * A.J;
            for (std::size_t j = i + 1; j < blocks.size(); ++j) {
                const Block& B = blocks[j];
                if (B.offset < 0) continue;
                const Eigen::MatrixXd hij = AtO * B.J;
                H.block(A.offset, B.offset, A.J.cols(), B.J.cols()) += hij;
                H.block(B.offset, A.offset, B.J.cols(), A.J.cols()) += hij.transpose();
            }
        }
    }

    void buildSystem(Eigen::MatrixXd& H, Eigen::VectorXd& b) const {
        H = Eigen::MatrixXd::Zero(dim_, dim_);
        b = Eigen::VectorXd::Zero(dim_);
        const Huber mono_huber(float(std::sqrt(5.991)));
        for (const MonoEdge& e : mono) {
            if (!e.active) continue;
            const Eigen::Matrix<double, 2, 6> A = monoJacobian(e);
            const double rho1 = e.robust ? mono_huber.weight(e.chi2()) : 1.;
            const Eigen::Matrix2d weighted = rho1 * e.information;
            b.segment<6>(cur_.offset).noalias() -= rho1 * A.transpose() * e.information * e.error;
            H.block<6, 6>(cur_.offset, cur_.offset).noalias() += A.transpose() * weighted * A;
        }
        const InertialJacobians J = inertialJacobians();
        add(H, b,
            {{prevCol(0), J.P1}, {prevCol(6), J.V1}, {prevCol(9), J.G1}, {prevCol(12), J.A1},
             {cur_.offset, J.P2}, {cur_.offset + 6, J.V2}},
            info_inertial_, -info_inertial_ * e_inertial_);
        add(H, b, {{prevCol(9), -Mat3::Identity()}, {cur_.offset + 9, Mat3::Identity()}}, info_g_, -info_g_ * e_g_);
        add(H, b, {{prevCol(12), -Mat3::Identity()}, {cur_.offset + 12, Mat3::Identity()}}, info_a_, -info_a_ * e_a_);
        if (prior) {
            const Huber prior_huber(5);
            const Matrix15d& info = prior->H;
            const double rho1 = prior_huber.weight(e_prior_.dot(info * e_prior_));
            Eigen::Matrix<double, 15, 1> r = -info * e_prior_;
            r *= rho1;
            const Eigen::Matrix<double, 15, 15> Jp = priorJacobian();
            add(H, b,
                {{prevCol(0), Jp.block<15, 6>(0, 0)}, {prevCol(6), Jp.block<15, 3>(0, 6)},
                 {prevCol(9), Jp.block<15, 3>(0, 9)}, {prevCol(12), Jp.block<15, 3>(0, 12)}},
                rho1 * info, r);
        }
    }
    // Column of the previous state's k-th variable, -1 when it is fixed
    int prevCol(int k) const { return prev_.offset < 0 ? -1 : prev_.offset + k; }

    // SparseOptimizer::update: vertices in id order (cur P, V, G, A, then prev)
    void update(const Eigen::VectorXd& x) {
        for (Vertex* v : {&cur_, &prev_}) {
            if (v->offset < 0) continue;
            const double* u = x.data() + v->offset;
            v->pose.update(u);
            v->v += Vec3(u[6], u[7], u[8]);
            v->bg += Vec3(u[9], u[10], u[11]);
            v->ba += Vec3(u[12], u[13], u[14]);
        }
    }

    const camera::StereoRig& rig_;
    Vertex& cur_;
    Vertex& prev_;
    const imu::Preintegrated& pint_;
    int dim_ = 15;
    Mat3 JRg_, JVg_, JPg_, JVa_, JPa_;
    double dt_ = 0;
    Vec3 g_;
    Matrix9d info_inertial_;
    Mat3 info_g_, info_a_;
    Eigen::Matrix<double, 9, 1> e_inertial_;
    Vec3 e_g_, e_a_;
    Eigen::Matrix<double, 15, 1> e_prior_;
};

Vertex frameVertex(const tracking::Frame& f, int offset) {
    Vertex v{BodyPose(f), f.velocity().cast<double>(), f.bias.g.cast<double>(), f.bias.a.cast<double>(), offset};
    return v;
}

// The mono edges of the frame's map points (left and right observations)
int addObservations(Problem& p, tracking::Frame& frame, const std::vector<const Eigen::Vector3f*>& world) {
    const orb::FrameFeatures& features = *frame.features;
    int n = 0;
    for (int i = 0; i < frame.size(); ++i) {
        if (!world[std::size_t(i)]) continue;
        const cv::KeyPoint& kp = features.keypoint(i);
        ++n;
        frame.outlier[std::size_t(i)] = false;
        MonoEdge e;
        e.index = i;
        e.cam = features.isRight(i) ? 1 : 0;
        e.Xw = world[std::size_t(i)]->cast<double>();
        e.obs = Eigen::Vector2d(kp.pt.x, kp.pt.y);
        const float unc2 = 1.f;  // KannalaBrandt8::uncertainty2
        const float inv_sigma2 = features.inv_level_sigma2[std::size_t(kp.octave)] / unc2;
        e.information = Eigen::Matrix2d::Identity() * inv_sigma2;
        p.mono.push_back(e);
    }
    return n;
}

// The 4 rounds, the recovery of not too bad points; returns nBad
int rounds(Problem& p, tracking::Frame& frame, const std::vector<float>& track_depth, const float chi2_mono[4],
           std::size_t n_other_edges, bool rec_init) {
    int n_bad = 0, n_inliers = 0;
    for (int it = 0; it < 4; ++it) {
        p.optimize(10);
        n_bad = 0;
        n_inliers = 0;
        const float chi2_close = 1.5 * chi2_mono[it];
        for (MonoEdge& e : p.mono) {
            const std::size_t idx = std::size_t(e.index);
            if (frame.outlier[idx]) p.computeError(e);
            const float chi2 = float(e.chi2());
            const bool close = track_depth[idx] < 10.f;
            if ((chi2 > chi2_mono[it] && !close) || (close && chi2 > chi2_close) || !p.depthPositive(e)) {
                frame.outlier[idx] = true;
                e.active = false;
                ++n_bad;
            } else {
                frame.outlier[idx] = false;
                e.active = true;
                ++n_inliers;
            }
            if (it == 2) e.robust = false;
        }
        if (p.mono.size() + n_other_edges < 10) break;
    }
    if (n_inliers < 30 && !rec_init) {
        n_bad = 0;
        const float chi2_mono_out = 18.f;
        for (MonoEdge& e : p.mono) {
            p.computeError(e);
            if (e.chi2() < chi2_mono_out)
                frame.outlier[std::size_t(e.index)] = false;
            else
                ++n_bad;
        }
    }
    return n_bad;
}

// The pose-only Hessian of the inlier observations (EdgeMonoOnlyPose::GetHessian)
Eigen::Matrix<double, 6, 6> observationHessian(const Problem& p, const tracking::Frame& frame) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    for (const MonoEdge& e : p.mono) {
        if (frame.outlier[std::size_t(e.index)]) continue;
        const Eigen::Matrix<double, 2, 6> J = p.monoJacobian(e);
        H += J.transpose() * e.information * J;
    }
    return H;
}

void setResult(tracking::Frame& frame, const Vertex& v, const Matrix15d& H) {
    frame.setImuPoseVelocity(v.pose.Rwb.cast<float>(), v.pose.twb.cast<float>(), v.v.cast<float>());
    frame.bias.g = v.bg.cast<float>();
    frame.bias.a = v.ba.cast<float>();
    tracking::PoseImuPrior c;  // ConstraintPoseImu
    c.Rwb = v.pose.Rwb;
    c.twb = v.pose.twb;
    c.vwb = v.v;
    c.bg = v.bg;
    c.ba = v.ba;
    c.H = clampEigen<15>(Matrix15d((H + H) / 2));
    frame.prior = c;
}

// Optimizer::Marginalize(H, 0, 14) of a 30x30 H: the Schur complement of the first 15 states (pseudo-inverse,
// singular values above 1e-6), as the last 15x15 block
Matrix15d marginalizeFirst(const Eigen::Matrix<double, 30, 30>& H) {
    const Eigen::MatrixXd Hbb = H.block<15, 15>(0, 0);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hbb, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType inv = svd.singularValues();
    for (int i = 0; i < 15; ++i) inv(i) = inv(i) > 1e-6 ? 1.0 / inv(i) : 0;
    const Eigen::MatrixXd invHb = svd.matrixV() * inv.asDiagonal() * svd.matrixU().transpose();
    const Eigen::MatrixXd Hcc = H.block<15, 15>(15, 15), Hcb = H.block<15, 15>(15, 0), Hbc = H.block<15, 15>(0, 15);
    return Hcc - Hcb * invHb * Hbc;
}

}  // namespace

int optimizePoseInertialLastKeyFrame(tracking::Frame& frame, const ImuState& keyframe, const camera::StereoRig& rig,
                                     const std::vector<const Eigen::Vector3f*>& world,
                                     const std::vector<float>& track_depth, bool rec_init) {
    Vertex cur = frameVertex(frame, 0);
    Vertex kf{BodyPose(keyframe), keyframe.vwb.cast<double>(), keyframe.bias.g.cast<double>(),
              keyframe.bias.a.cast<double>(), -1};
    const imu::Preintegrated& pint = *frame.preintegrated;
    Problem p(rig, cur, kf, pint, pint);
    const int n_initial = addObservations(p, frame, world);

    const float chi2_mono[4] = {12, 7.5, 5.991, 5.991};
    const int n_bad = rounds(p, frame, track_depth, chi2_mono, 3, rec_init);

    Matrix15d H = Matrix15d::Zero();
    const Problem::InertialJacobians J = p.inertialJacobians();  // GetHessian2: wrt P2, V2
    Matrix9d J2;
    J2.block<9, 6>(0, 0) = J.P2;
    J2.block<9, 3>(0, 6) = J.V2;
    H.block<9, 9>(0, 0) += J2.transpose() * p.inertialInformation() * J2;
    H.block<3, 3>(9, 9) += p.gyroInformation();
    H.block<3, 3>(12, 12) += p.accInformation();
    H.block<6, 6>(0, 0) += observationHessian(p, frame);
    setResult(frame, cur, H);
    return n_initial - n_bad;
}

int optimizePoseInertialLastFrame(tracking::Frame& frame, tracking::Frame& prev, const camera::StereoRig& rig,
                                  const std::vector<const Eigen::Vector3f*>& world,
                                  const std::vector<float>& track_depth, bool rec_init) {
    Vertex cur = frameVertex(frame, 0);
    Vertex pv = frameVertex(prev, 15);
    Problem p(rig, cur, pv, *frame.preintegrated_frame, *frame.preintegrated);
    p.prior = prev.prior;  // the original requires it (it dereferences mpcpi)
    const int n_initial = addObservations(p, frame, world);

    const float chi2_mono[4] = {5.991f, 5.991f, 5.991f, 5.991f};
    const int n_bad = rounds(p, frame, track_depth, chi2_mono, p.prior ? 4 : 3, rec_init);

    // H over (prev P, V, G, A, cur P, V, G, A)
    Eigen::Matrix<double, 30, 30> H = Eigen::Matrix<double, 30, 30>::Zero();
    const Problem::InertialJacobians J = p.inertialJacobians();
    Eigen::Matrix<double, 9, 24> Ji;
    Ji << J.P1, J.V1, J.G1, J.A1, J.P2, J.V2;
    H.block<24, 24>(0, 0) += Ji.transpose() * p.inertialInformation() * Ji;
    Eigen::Matrix<double, 3, 6> Jrw;
    Jrw << -Mat3::Identity(), Mat3::Identity();
    const Eigen::Matrix<double, 6, 6> Hgr = Jrw.transpose() * p.gyroInformation() * Jrw;
    H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
    H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
    H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
    H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);
    const Eigen::Matrix<double, 6, 6> Har = Jrw.transpose() * p.accInformation() * Jrw;
    H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
    H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
    H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
    H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);
    if (p.prior) {
        const Eigen::Matrix<double, 15, 15> Jp = p.priorJacobian();
        H.block<15, 15>(0, 0) += Jp.transpose() * p.prior->H * Jp;
    }
    H.block<6, 6>(15, 15) += observationHessian(p, frame);

    setResult(frame, cur, marginalizeFirst(H));
    prev.prior.reset();
    return n_initial - n_bad;
}

}  // namespace orb_tracking
