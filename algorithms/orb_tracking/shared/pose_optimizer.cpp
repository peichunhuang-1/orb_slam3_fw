#include "algorithms/orb_tracking/shared/pose_optimizer.hpp"

#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "types/geometry/se3.hpp"

namespace orb_tracking {
namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using geometry::SE3d;

// g2o's skew and SE3Quat::exp (rotation first, then translation)
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m.fill(0.);
    m(0, 1) = -v(2);
    m(0, 2) = v(1);
    m(1, 2) = -v(0);
    m(1, 0) = v(2);
    m(2, 0) = -v(1);
    m(2, 1) = v(0);
    return m;
}

SE3d expSE3(const Vector6d& update) {
    const Eigen::Vector3d omega = update.head<3>(), upsilon = update.tail<3>();
    const double theta = omega.norm();
    const Eigen::Matrix3d Omega = skew(omega);
    Eigen::Matrix3d R, V;
    if (theta < 0.00001) {
        R = Eigen::Matrix3d::Identity() + Omega + Omega * Omega;  // as g2o (not the series' 1/2)
        V = R;
    } else {
        const Eigen::Matrix3d Omega2 = Omega * Omega;
        R = Eigen::Matrix3d::Identity() + std::sin(theta) / theta * Omega + (1 - std::cos(theta)) / (theta * theta) * Omega2;
        V = Eigen::Matrix3d::Identity() + (1 - std::cos(theta)) / (theta * theta) * Omega +
            (theta - std::sin(theta)) / std::pow(theta, 3) * Omega2;
    }
    return SE3d(Eigen::Quaterniond(R), V * upsilon);  // SE3Quat(q, t) normalizes
}

// g2o's RobustKernelHuber with setDelta(sqrt(5.991)): delta in double from the float, its square in float
struct Huber {
    double delta;
    float dsqr;
    explicit Huber(float d) : delta(d), dsqr(float(delta * delta)) {}
    // rho[0], rho[1]
    void robustify(double e, double& rho0, double& rho1) const {
        if (e <= dsqr) {
            rho0 = e;
            rho1 = 1.;
        } else {
            const double sqrte = std::sqrt(e);
            rho0 = 2 * sqrte * delta - dsqr;
            rho1 = delta / sqrte;
        }
    }
};

struct Edge {
    int index = 0;
    bool right = false;
    Eigen::Vector2d obs;
    Eigen::Matrix2d information;
    Eigen::Vector3d Xw;
    Eigen::Vector2d error = Eigen::Vector2d::Zero();
    bool active = true;  // level 0
    bool robust = true;
    double chi2() const { return error.dot(information * error); }
};

class PoseProblem {
public:
    PoseProblem(const camera::StereoRig& rig, const SE3d& Trl, const Huber& huber, const ParallelFor& par)
        : rig_(rig), Trl_(Trl), huber_(huber), par_(par) {
        Rrl_ = Trl_.unitQuaternion().toRotationMatrix();
    }

    std::vector<Edge> edges;
    SE3d estimate;

    // The KB8 model in double (project_jacobian_d): the error and (J != nullptr) d(u, v) / dXl, Xl the point in the
    // left camera, from one θ. Returns Xl.
    Eigen::Vector3d computeError(Edge& e, Eigen::Matrix<double, 2, 3>* J = nullptr) const {
        const Eigen::Vector3d Xl = estimate * e.Xw;
        if (!e.right) {
            e.error = e.obs - rig_.camera(0).project_jacobian_d(Xl, J);
        } else {
            e.error = e.obs - rig_.camera(1).project_jacobian_d(Trl_ * Xl, J);
            if (J) *J = *J * Rrl_;
        }
        return Xl;
    }

    // g2o's SparseOptimizer::optimize(iterations) with OptimizationAlgorithmLevenberg, on the active edges
    void optimize(int iterations) {
        double lambda = 0, ni = 2;
        int n_bad = 0;
        Vector6d x = Vector6d::Zero();
        for (int it = 0; it < iterations; ++it) {
            Matrix6d H;
            Vector6d b;
            double current_chi = buildSystem(H, b);
            const double ini_chi = current_chi;
            if (it == 0) {
                double max_diagonal = 0.;
                for (int j = 0; j < 6; ++j) max_diagonal = std::max(std::fabs(H(j, j)), max_diagonal);
                lambda = 1e-5 * max_diagonal;
                ni = 2;
                n_bad = 0;
            }
            double rho = 0;
            int qmax = 0;
            do {
                const SE3d backup = estimate;
                Eigen::MatrixXd Hl = H;
                Hl.diagonal().array() += lambda;
                Eigen::LDLT<Eigen::MatrixXd> ldlt(Hl);
                const bool ok = ldlt.isPositive();
                if (ok) x = ldlt.solve(Eigen::VectorXd(b));
                estimate = expSE3(x) * estimate;
                double temp_chi = activeErrors();
                if (!ok) temp_chi = std::numeric_limits<double>::max();
                rho = current_chi - temp_chi;
                double scale = 0.;
                for (int j = 0; j < 6; ++j) scale += x[j] * (lambda * x[j] + b[j]);
                scale += 1e-3;
                rho /= scale;
                if (rho > 0 && std::isfinite(temp_chi)) {
                    double alpha = 1. - std::pow((2 * rho - 1), 3);
                    alpha = std::min(alpha, 2. / 3.);
                    const double scale_factor = std::max(1. / 3., alpha);
                    lambda *= scale_factor;
                    ni = 2;
                    current_chi = temp_chi;
                } else {
                    lambda *= ni;
                    ni *= 2;
                    estimate = backup;  // the errors stay those of the rejected step, as in g2o
                }
                ++qmax;
            } while (rho < 0 && qmax < 10);
            if (qmax == 10 || rho == 0) break;
            if ((ini_chi - current_chi) * 1e3 < ini_chi)
                ++n_bad;
            else
                n_bad = 0;
            if (n_bad >= 3) break;
        }
    }

private:
    // An active edge's part of the robust chi2 (rho0)
    double robustChi2(const Edge& e) const {
        if (!e.robust) return e.chi2();
        double rho0, rho1;
        huber_.robustify(e.chi2(), rho0, rho1);
        return rho0;
    }

    // The active edges in kParallelChunks fixed chunks on par_, each chunk's sums added in chunk order (the result
    // does not depend on the thread count)
    template <class F>
    void forChunks(const F& fn) {
        const int n = int(edges.size());
        runParallel(par_, 0, kParallelChunks, [&](int c) {
            for (int k = chunkBegin(n, c); k < chunkBegin(n, c + 1); ++k) {
                Edge& e = edges[std::size_t(k)];
                if (e.active) fn(c, e);
            }
        });
    }

    // The active edges' errors (EdgeSE3ProjectXYZOnlyPose::computeError); returns their robust chi2
    double activeErrors() {
        std::array<double, kParallelChunks> chi{};
        forChunks([&](int c, Edge& e) {
            computeError(e);
            chi[std::size_t(c)] += robustChi2(e);
        });
        double sum = 0.0;
        for (double v : chi) sum += v;
        return sum;
    }

    // The active edges' errors and BlockSolver::buildSystem with BaseUnaryEdge::constructQuadraticForm, from one
    // projection per edge; returns their robust chi2
    double buildSystem(Matrix6d& H, Vector6d& b) {
        std::array<Matrix6d, kParallelChunks> Hc;
        std::array<Vector6d, kParallelChunks> bc;
        std::array<double, kParallelChunks> chi{};
        for (int c = 0; c < kParallelChunks; ++c) {
            Hc[std::size_t(c)].setZero();
            bc[std::size_t(c)].setZero();
        }
        forChunks([&](int c, Edge& e) {
            Eigen::Matrix<double, 2, 3> J;
            const Eigen::Vector3d Xl = computeError(e, &J);
            const double x = Xl[0], y = Xl[1], z = Xl[2];
            Eigen::Matrix<double, 3, 6> d;
            d << 0., z, -y, 1., 0., 0.,
                 -z, 0., x, 0., 1., 0.,
                 y, -x, 0., 0., 0., 1.;
            const Eigen::Matrix<double, 2, 6> A = -J * d;
            const Eigen::Matrix2d& omega = e.information;
            double rho1 = 1.;
            if (e.robust) {
                double rho0;
                huber_.robustify(e.chi2(), rho0, rho1);
                chi[std::size_t(c)] += rho0;
            } else {
                chi[std::size_t(c)] += e.chi2();
            }
            bc[std::size_t(c)].noalias() -= rho1 * A.transpose() * omega * e.error;
            Hc[std::size_t(c)].noalias() += A.transpose() * (rho1 * omega) * A;
        });
        H.setZero();
        b.setZero();
        double sum = 0.0;
        for (int c = 0; c < kParallelChunks; ++c) {
            H += Hc[std::size_t(c)];
            b += bc[std::size_t(c)];
            sum += chi[std::size_t(c)];
        }
        return sum;
    }

    const camera::StereoRig& rig_;
    SE3d Trl_;
    Eigen::Matrix3d Rrl_;
    Huber huber_;
    const ParallelFor& par_;
};

}  // namespace

int optimizePose(tracking::Frame& frame, const camera::StereoRig& rig, const std::vector<const Eigen::Vector3f*>& world,
                 const ParallelFor& par) {
    const orb::FrameFeatures& features = *frame.features;
    const int N = frame.size();
    const float delta_mono = std::sqrt(5.991);
    const SE3d Trl(frame.Trl.unitQuaternion().cast<double>(), frame.Trl.translation().cast<double>());
    PoseProblem problem(rig, Trl, Huber(delta_mono), par);
    problem.edges.reserve(std::size_t(N));

    int n_initial = 0;
    for (int i = 0; i < N; ++i) {
        if (!world[std::size_t(i)]) continue;
        ++n_initial;
        const cv::KeyPoint& kp = features.keypoint(i);
        frame.outlier[std::size_t(i)] = false;
        Edge e;
        e.index = i;
        e.right = features.isRight(i);
        e.obs = Eigen::Vector2d(kp.pt.x, kp.pt.y);
        const float inv_sigma2 = features.inv_level_sigma2[std::size_t(kp.octave)];
        e.information = Eigen::Matrix2d::Identity() * inv_sigma2;
        e.Xw = world[std::size_t(i)]->cast<double>();
        problem.edges.push_back(e);
    }
    if (n_initial < 3) return 0;

    const float chi2_mono = 5.991f;
    const geometry::SE3f Tcw0 = frame.pose();
    const SE3d start(Tcw0.unitQuaternion().cast<double>(), Tcw0.translation().cast<double>());
    int n_bad = 0;
    for (int it = 0; it < 4; ++it) {
        problem.estimate = start;
        problem.optimize(10);
        n_bad = 0;
        for (Edge& e : problem.edges) {
            const std::size_t idx = std::size_t(e.index);
            if (frame.outlier[idx]) problem.computeError(e);
            const float chi2 = float(e.chi2());
            if (chi2 > chi2_mono) {
                frame.outlier[idx] = true;
                e.active = false;
                ++n_bad;
            } else {
                frame.outlier[idx] = false;
                e.active = true;
            }
            if (it == 2) e.robust = false;
        }
        if (problem.edges.size() < 10) break;
    }

    const SE3d& T = problem.estimate;
    frame.setPose(geometry::SE3f(T.unitQuaternion().cast<float>(), T.translation().cast<float>()));
    return n_initial - n_bad;
}

int optimizePose(tracking::Frame& frame, const map::Atlas& atlas, const ParallelFor& par) {
    std::vector<const Eigen::Vector3f*> world(std::size_t(frame.size()), nullptr);
    for (std::size_t i = 0; i < world.size(); ++i)
        if (const map::MapPoint* p = atlas.point(frame.points[i])) world[i] = &p->pos;
    return optimizePose(frame, atlas.rig(frame.rig), world, par);
}

}  // namespace orb_tracking
