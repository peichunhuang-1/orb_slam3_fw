#include "algorithms/orb_loop_closing/optimize_relative_pose/optimize_relative_pose.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Cholesky>

#include "types/geometry/so3.hpp"

namespace orb_loop_closing {

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using geometry::SE3d;

// g2o::Sim3(update) with σ = 0: the rotation as g2o (I + Ω + Ω² below 1e-5), t = W·υ
SE3d expUpdate(const Vector6d& u) {
    const Eigen::Vector3d omega = u.head<3>(), upsilon = u.tail<3>();
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
    return SE3d(Eigen::Quaterniond(R), W * upsilon);
}

// g2o's RobustKernelHuber (as orb_tracking/shared/pose_optimizer)
struct Huber {
    double delta;
    float dsqr;
    explicit Huber(float d) : delta(d), dsqr(float(delta * delta)) {}
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
    bool inverse = false;  // e21: KB8(T12⁻¹ · X)
    const camera::KannalaBrandt8* cam = nullptr;
    Eigen::Vector3d X;     // fixed point (kf2's camera for e12, kf1's for e21)
    Eigen::Vector2d obs;
    double info = 1;       // information = info · I
    Eigen::Vector2d error = Eigen::Vector2d::Zero();
    bool active = true, robust = true;
    double chi2() const { return info * error.squaredNorm(); }
};

class Problem {
public:
    explicit Problem(const Huber& h) : huber_(h) {}

    std::vector<Edge> edges;
    SE3d estimate;

    // The KB8 model in double (project_jacobian_d), the error and (A != nullptr) d(error)/dδ for T12 ← exp(δ)·T12,
    // δ = (ω, υ), from one θ.
    //   e12: q = T12·X,     dq/dω = −[q]×,    dq/dυ = I
    //   e21: p = T12⁻¹·X,   dp/dω = R⁻¹[X]×,  dp/dυ = −R⁻¹
    void computeError(Edge& e, Eigen::Matrix<double, 2, 6>* A = nullptr) const {
        Eigen::Matrix<double, 2, 3> Jp;
        if (!e.inverse) {
            const Eigen::Vector3d q = estimate * e.X;
            e.error = e.obs - e.cam->project_jacobian_d(q, A ? &Jp : nullptr);
            if (A) {
                A->leftCols<3>() = Jp * geometry::hat(q);
                A->rightCols<3>() = -Jp;
            }
        } else {
            const SE3d Tinv = estimate.inverse();
            const Eigen::Vector3d p = Tinv * e.X;
            e.error = e.obs - e.cam->project_jacobian_d(p, A ? &Jp : nullptr);
            if (A) {
                const Eigen::Matrix3d Rinv = Tinv.rotationMatrix();
                A->leftCols<3>() = -Jp * Rinv * geometry::hat(e.X);
                A->rightCols<3>() = Jp * Rinv;
            }
        }
    }

    // g2o's optimize(iterations) with OptimizationAlgorithmLevenberg (pose_optimizer's PoseProblem::optimize)
    void optimize(int iterations) {
        double lambda = 0, ni = 2;
        int n_bad = 0;
        Vector6d x = Vector6d::Zero();
        for (int it = 0; it < iterations; ++it) {
            computeActiveErrors();
            double current_chi = activeRobustChi2();
            const double ini_chi = current_chi;
            Matrix6d H;
            Vector6d b;
            buildSystem(H, b);
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
                Matrix6d Hl = H;
                Hl.diagonal().array() += lambda;
                Eigen::LDLT<Matrix6d> ldlt(Hl);
                const bool ok = ldlt.isPositive();
                if (ok) x = ldlt.solve(b);
                estimate = expUpdate(x) * estimate;
                computeActiveErrors();
                double temp_chi = activeRobustChi2();
                if (!ok) temp_chi = std::numeric_limits<double>::max();
                rho = current_chi - temp_chi;
                double scale = 0.;
                for (int j = 0; j < 6; ++j) scale += x[j] * (lambda * x[j] + b[j]);
                scale += 1e-3;
                rho /= scale;
                if (rho > 0 && std::isfinite(temp_chi)) {
                    double alpha = 1. - std::pow((2 * rho - 1), 3);
                    alpha = std::min(alpha, 2. / 3.);
                    lambda *= std::max(1. / 3., alpha);
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
    void computeActiveErrors() {
        for (Edge& e : edges)
            if (e.active) computeError(e);
    }

    double activeRobustChi2() const {
        double chi = 0.0;
        for (const Edge& e : edges) {
            if (!e.active) continue;
            if (e.robust) {
                double rho0, rho1;
                huber_.robustify(e.chi2(), rho0, rho1);
                chi += rho0;
            } else {
                chi += e.chi2();
            }
        }
        return chi;
    }

    void buildSystem(Matrix6d& H, Vector6d& b) {
        H.setZero();
        b.setZero();
        for (Edge& e : edges) {
            if (!e.active) continue;
            Eigen::Matrix<double, 2, 6> A;
            computeError(e, &A);  // the errors are those computeActiveErrors just set; this adds the Jacobian
            double w = e.info;
            if (e.robust) {
                double rho0, rho1;
                huber_.robustify(e.chi2(), rho0, rho1);
                w *= rho1;
            }
            b.noalias() -= w * A.transpose() * e.error;
            H.noalias() += w * A.transpose() * A;
        }
    }

    Huber huber_;
};

}  // namespace

int optimizeRelativePose(const map::Atlas& a, const map::KeyFrame& kf1, const map::KeyFrame& kf2,
                         std::vector<map::MapPointId>& matches, geometry::SE3d& T12, float th2, bool all_points,
                         int unseen_level) {
    const camera::KannalaBrandt8& cam1 = a.rig(kf1.data->rig).camera(0);
    const camera::KannalaBrandt8& cam2 = a.rig(kf2.data->rig).camera(0);
    const orb::FrameFeatures& f1 = *kf1.data->features;
    const orb::FrameFeatures& f2 = *kf2.data->features;
    const geometry::SE3f T1w = kf1.pose(), T2w = kf2.pose();
    const Eigen::Matrix3f R1w = T1w.rotationMatrix(), R2w = T2w.rotationMatrix();
    const Eigen::Vector3f t1w = T1w.translation(), t2w = T2w.translation();
    const Eigen::Vector3f O2 = kf2.cameraCenter();

    Problem problem(Huber(std::sqrt(th2)));
    problem.estimate = T12;
    std::vector<std::size_t> index;  // per pair (edges 2k, 2k + 1): the kf1 feature
    int n_correspondences = 0;

    const int N = int(matches.size());
    for (int i = 0; i < N; ++i) {
        if (matches[std::size_t(i)] == map::kNone) continue;
        const map::MapPoint* p1 = a.point(kf1.points[std::size_t(i)]);
        const map::MapPoint* p2 = a.point(matches[std::size_t(i)]);
        if (!p2) continue;
        if (!p1) continue;  // the match stays, no edge (nMatchWithoutMP)
        if (p1->bad || p2->bad) continue;
        if (i >= f1.n_left) continue;
        const int i2 = p2->indexIn(kf2.id).left;
        if (i2 < 0 && !all_points) continue;
        const Eigen::Vector3f P3D1c = R1w * p1->pos + t1w;
        const Eigen::Vector3f P3D2c = R2w * p2->pos + t2w;
        if (P3D2c(2) < 0) continue;
        ++n_correspondences;

        const cv::KeyPoint& kp1 = f1.keypoint(i);
        Edge e12;
        e12.cam = &cam1;
        e12.X = P3D2c.cast<double>();
        e12.obs = Eigen::Vector2d(kp1.pt.x, kp1.pt.y);
        e12.info = f1.inv_level_sigma2[std::size_t(kp1.octave)];

        Edge e21;
        e21.inverse = true;
        e21.cam = &cam2;
        e21.X = P3D1c.cast<double>();
        int octave2;
        if (i2 >= 0) {
            const cv::KeyPoint& kp2 = f2.keypoint(i2);
            e21.obs = Eigen::Vector2d(kp2.pt.x, kp2.pt.y);
            octave2 = kp2.octave;
        } else {
            const Eigen::Vector2f uv = cam2.project_orb(P3D2c);
            e21.obs = Eigen::Vector2d(uv(0), uv(1));
            octave2 = unseen_level >= 0 ? unseen_level
                                        : p2->predictScale((p2->pos - O2).norm(), f2.log_scale_factor, f2.levels);
        }
        e21.info = f2.inv_level_sigma2[std::size_t(octave2)];

        problem.edges.push_back(e12);
        problem.edges.push_back(e21);
        index.push_back(std::size_t(i));
    }

    problem.optimize(5);

    // Inliers: the errors as the last trial step left them (g2o)
    int n_bad = 0;
    for (std::size_t k = 0; k < index.size(); ++k) {
        Edge& e12 = problem.edges[2 * k];
        Edge& e21 = problem.edges[2 * k + 1];
        if (e12.chi2() > th2 || e21.chi2() > th2) {
            matches[index[k]] = map::kNone;
            e12.active = e21.active = false;
            ++n_bad;
            continue;
        }
        e12.robust = e21.robust = false;
    }
    const int more = n_bad > 0 ? 10 : 5;
    if (n_correspondences - n_bad < 10) return 0;

    problem.optimize(more);

    int n_in = 0;
    for (std::size_t k = 0; k < index.size(); ++k) {
        Edge& e12 = problem.edges[2 * k];
        Edge& e21 = problem.edges[2 * k + 1];
        if (!e12.active) continue;
        problem.computeError(e12);
        problem.computeError(e21);
        if (e12.chi2() > th2 || e21.chi2() > th2)
            matches[index[k]] = map::kNone;
        else
            ++n_in;
    }
    T12 = problem.estimate;
    return n_in;
}

}  // namespace orb_loop_closing
