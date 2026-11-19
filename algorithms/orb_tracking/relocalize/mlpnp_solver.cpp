#include "algorithms/orb_tracking/relocalize/mlpnp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/SVD>

#include "types/geometry/so3.hpp"

namespace orb_tracking {
namespace {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// MLPnPsolver::rodrigues2rot and rot2rodrigues, as written
Mat3 rodrigues2rot(const Vec3& omega) {
    Mat3 R = Mat3::Identity();
    Mat3 skewW;
    skewW << 0.0, -omega(2), omega(1), omega(2), 0.0, -omega(0), -omega(1), omega(0), 0.0;
    const double n = omega.norm();
    if (n > std::numeric_limits<double>::epsilon())
        R = R + std::sin(n) / n * skewW + (1 - std::cos(n)) / (n * n) * (skewW * skewW);
    return R;
}

Vec3 rot2rodrigues(const Mat3& R) {
    Vec3 omega(0.0, 0.0, 0.0);
    const double trace = R.trace() - 1.0;
    const double wnorm = std::acos(trace / 2.0);  // NaN beyond ±1: no rotation, as the original
    if (wnorm > std::numeric_limits<double>::epsilon()) {
        omega << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
        omega *= wnorm / (2.0 * std::sin(wnorm));
    }
    return omega;
}

// Residuals n_kᵀ·p/|p| (p = R(w)X + t, both null-space directions per point) and their Jacobian wrt (w, t)
void residualsAndJacobian(const Eigen::Matrix<double, 6, 1>& x, const std::vector<Vec3>& pts,
                          const std::vector<Eigen::Matrix<double, 3, 2>>& nullspaces, Eigen::VectorXd& r,
                          Eigen::MatrixXd& J) {
    const Vec3 w = x.head<3>(), t = x.tail<3>();
    const Mat3 R = rodrigues2rot(w);
    const Mat3 Jr = geometry::rightJacobian(w);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const Vec3 p = R * pts[i] + t;
        const double norm = p.norm();
        const Vec3 u = p / norm;
        const Eigen::Index k = Eigen::Index(2 * i);
        r[k] = nullspaces[i].col(0).dot(u);
        r[k + 1] = nullspaces[i].col(1).dot(u);
        const Mat3 du = (Mat3::Identity() - u * u.transpose()) / norm;  // d(p/|p|)/dp
        Eigen::Matrix<double, 3, 6> dp;
        dp.leftCols<3>() = -R * geometry::hat(pts[i]) * Jr;
        dp.rightCols<3>() = Mat3::Identity();
        J.block<2, 6>(k, 0) = nullspaces[i].transpose() * du * dp;
    }
}

// mlpnp_gn: at most 5 steps, stopped early by a spurious step or by converged observations
void gaussNewton(Eigen::Matrix<double, 6, 1>& x, const std::vector<Vec3>& pts,
                 const std::vector<Eigen::Matrix<double, 3, 2>>& nullspaces) {
    const Eigen::Index n = Eigen::Index(2 * pts.size());
    Eigen::VectorXd r(n);
    Eigen::MatrixXd J(n, 6);
    const double eps_p = 1e-5;
    for (int it = 0; it < 5; ++it) {
        residualsAndJacobian(x, pts, nullspaces, r, J);
        const Eigen::MatrixXd A = J.transpose() * J;
        const Eigen::VectorXd g = J.transpose() * r;
        const Eigen::VectorXd dx = Eigen::LDLT<Eigen::MatrixXd>(A).solve(g);
        if (dx.array().abs().maxCoeff() > 5.0 || dx.array().abs().minCoeff() > 1.0) break;
        const Eigen::VectorXd dl = J * dx;
        x -= dx;
        if (dl.array().abs().maxCoeff() < eps_p) break;
    }
}

}  // namespace

Eigen::Matrix<double, 3, 4> MLPnPSolver::computePose(const std::vector<Vec3>& f, const std::vector<Vec3>& p) {
    const std::size_t n = f.size();
    std::vector<Eigen::Matrix<double, 3, 2>> nullspaces(n);
    Eigen::MatrixXd points3(3, Eigen::Index(n));
    for (std::size_t i = 0; i < n; ++i) {
        points3.col(Eigen::Index(i)) = p[i];
        Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner> svd_f(f[i].transpose(), Eigen::ComputeFullV);
        nullspaces[i] = svd_f.matrixV().block(0, 1, 3, 2);
    }

    // 1. planar scene: work in the points' eigen frame
    const Mat3 planar_test = points3 * points3.transpose();
    const Eigen::FullPivHouseholderQR<Mat3> rank_test(planar_test);
    Mat3 eigen_rot = Mat3::Identity();
    const bool planar = rank_test.rank() == 2;
    if (planar) {
        const Eigen::SelfAdjointEigenSolver<Mat3> es(planar_test);
        eigen_rot = es.eigenvectors().real().transpose();
        for (std::size_t i = 0; i < n; ++i) points3.col(Eigen::Index(i)) = eigen_rot * points3.col(Eigen::Index(i));
    }

    // 2-4. design matrix (no covariance: P = I) and its null vector
    const int cols = planar ? 9 : 12;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(Eigen::Index(2 * n), cols);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 pt = points3.col(Eigen::Index(i));
        const Eigen::Matrix<double, 3, 2>& N = nullspaces[i];
        const Eigen::Index r = Eigen::Index(2 * i);
        for (int k = 0; k < 2; ++k) {
            if (planar) {
                for (int row = 0; row < 3; ++row) {  // r_row2, r_row3
                    A(r + k, 2 * row) = N(row, k) * pt[1];
                    A(r + k, 2 * row + 1) = N(row, k) * pt[2];
                }
                for (int row = 0; row < 3; ++row) A(r + k, 6 + row) = N(row, k);
            } else {
                for (int row = 0; row < 3; ++row)
                    for (int c = 0; c < 3; ++c) A(r + k, 3 * row + c) = N(row, k) * pt[c];
                for (int row = 0; row < 3; ++row) A(r + k, 9 + row) = N(row, k);
            }
        }
    }
    const Eigen::MatrixXd AtA = A.transpose() * A;
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(AtA, Eigen::ComputeFullV);
    const Eigen::VectorXd x = svd_A.matrixV().col(cols - 1);

    Mat3 Rout;
    Vec3 tout;
    if (planar) {
        Mat3 tmp;
        tmp << 0.0, x(0), x(1), 0.0, x(2), x(3), 0.0, x(4), x(5);
        tmp.col(0) = tmp.col(1).cross(tmp.col(2));
        tmp.transposeInPlace();
        const double scale = 1.0 / std::sqrt(std::abs(tmp.col(1).norm() * tmp.col(2).norm()));
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd_R(tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Mat3 R1 = svd_R.matrixU() * svd_R.matrixV().transpose();
        if (R1.determinant() < 0) R1 *= -1.0;
        R1 = eigen_rot.transpose() * R1;
        const Vec3 t = scale * Vec3(x(6), x(7), x(8));
        R1.transposeInPlace();
        R1 *= -1;
        if (R1.determinant() < 0.0) R1.col(2) *= -1;
        Mat3 R2 = R1;
        R2.col(0) = -R1.col(0);
        R2.col(1) = -R1.col(1);
        const Mat3 Rs[4] = {R1, R1, R2, R2};
        const Vec3 ts[4] = {t, -t, t, -t};
        // the combination closest to the first 6 bearings
        int best = 0;
        double best_norm = 0;
        for (int i = 0; i < 4; ++i) {
            double norms = 0.0;
            for (int k = 0; k < 6; ++k) {
                Vec3 q = Rs[i] * p[std::size_t(k)] + ts[i];
                q = q / q.norm();
                norms += (1.0 - q.dot(f[std::size_t(k)]));
            }
            if (i == 0 || norms < best_norm) {
                best_norm = norms;
                best = i;
            }
        }
        Rout = Rs[best];
        tout = ts[best];
    } else {
        Mat3 tmp;
        tmp << x(0), x(3), x(6), x(1), x(4), x(7), x(2), x(5), x(8);
        const double scale = 1.0 / std::pow(std::abs(tmp.col(0).norm() * tmp.col(1).norm() * tmp.col(2).norm()), 1.0 / 3.0);
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd_R(tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Rout = svd_R.matrixU() * svd_R.matrixV().transpose();
        if (Rout.determinant() < 0) Rout *= -1.0;
        tout = Rout * (scale * Vec3(x(9), x(10), x(11)));
        // the sign of t from the first 6 bearings, on the inverted transform (as the original)
        double error[2];
        Eigen::Matrix4d Ts[2];
        for (int s = 0; s < 2; ++s) {
            error[s] = 0.0;
            Ts[s] = Eigen::Matrix4d::Identity();
            Ts[s].block<3, 3>(0, 0) = Rout;
            Ts[s].block<3, 1>(0, 3) = s == 0 ? tout : Vec3(-tout);
            Ts[s] = Ts[s].inverse().eval();
            for (int k = 0; k < 6; ++k) {
                Vec3 v = Ts[s].block<3, 3>(0, 0) * p[std::size_t(k)] + Ts[s].block<3, 1>(0, 3);
                v = v / v.norm();
                error[s] += (1.0 - v.dot(f[std::size_t(k)]));
            }
        }
        tout = error[0] < error[1] ? Vec3(Ts[0].block<3, 1>(0, 3)) : Vec3(Ts[1].block<3, 1>(0, 3));
        Rout = Ts[0].block<3, 3>(0, 0);
    }

    // 5. Gauss-Newton
    Eigen::Matrix<double, 6, 1> minx;
    minx << rot2rodrigues(Rout), tout;
    gaussNewton(minx, p, nullspaces);
    Eigen::Matrix<double, 3, 4> result;
    result.leftCols<3>() = rodrigues2rot(minx.head<3>());
    result.col(3) = minx.tail<3>();
    return result;
}

MLPnPSolver::MLPnPSolver(const tracking::Frame& frame, const camera::KannalaBrandt8& camera,
                         const std::vector<const Eigen::Vector3f*>& matches, unsigned seed)
    : camera_(camera), frame_size_(matches.size()), rng_(seed) {
    const orb::FrameFeatures& features = *frame.features;
    int idx = 0;
    for (std::size_t i = 0; i < matches.size(); ++i) {
        if (!matches[i]) continue;
        if (int(i) >= features.n_left) continue;  // mvKeysUn: the left keypoints
        const cv::KeyPoint& kp = features.keypoint(int(i));
        p2d_.emplace_back(kp.pt.x, kp.pt.y);
        sigma2_.push_back(features.level_sigma2[std::size_t(kp.octave)]);
        const Eigen::Vector3f b = camera_.unproject_orb(Eigen::Vector2f(kp.pt.x, kp.pt.y));
        bearings_.push_back((b / b.z()).cast<double>());
        p3d_.push_back(matches[i]->cast<double>());
        keypoint_index_.push_back(int(i));
        all_indices_.push_back(idx++);
    }
    setRansacParameters();
}

void MLPnPSolver::setRansacParameters(double probability, int min_inliers, int max_iterations, int min_set,
                                      float epsilon, float th2) {
    probability_ = probability;
    min_inliers_ = min_inliers;
    max_iterations_ = max_iterations;
    epsilon_ = epsilon;
    min_set_ = min_set;
    N_ = int(p2d_.size());
    inliers_i_.resize(std::size_t(N_));

    int n_min_inliers = int(N_ * epsilon_);
    if (n_min_inliers < min_inliers_) n_min_inliers = min_inliers_;
    if (n_min_inliers < min_set) n_min_inliers = min_set;
    min_inliers_ = n_min_inliers;
    if (epsilon_ < float(min_inliers_) / N_) epsilon_ = float(min_inliers_) / N_;

    int n_iterations;
    if (min_inliers_ == N_)
        n_iterations = 1;
    else
        n_iterations = int(std::ceil(std::log(1 - probability_) / std::log(1 - std::pow(epsilon_, 3))));
    max_iterations_ = std::max(1, std::min(n_iterations, max_iterations_));

    max_error_.resize(sigma2_.size());
    for (std::size_t i = 0; i < sigma2_.size(); ++i) max_error_[i] = sigma2_[i] * th2;
}

void MLPnPSolver::checkInliers() {
    n_inliers_i_ = 0;
    for (int i = 0; i < N_; ++i) {
        const Eigen::Vector3f P = p3d_[std::size_t(i)].cast<float>();
        const float xc = float(Ti_(0, 0) * P.x() + Ti_(0, 1) * P.y() + Ti_(0, 2) * P.z() + Ti_(0, 3));
        const float yc = float(Ti_(1, 0) * P.x() + Ti_(1, 1) * P.y() + Ti_(1, 2) * P.z() + Ti_(1, 3));
        const float zc = float(Ti_(2, 0) * P.x() + Ti_(2, 1) * P.y() + Ti_(2, 2) * P.z() + Ti_(2, 3));
        const Eigen::Vector2f uv = camera_.project_orb(Eigen::Vector3f(xc, yc, zc));
        const float dx = p2d_[std::size_t(i)].x() - uv.x(), dy = p2d_[std::size_t(i)].y() - uv.y();
        const float error2 = dx * dx + dy * dy;
        inliers_i_[std::size_t(i)] = error2 < max_error_[std::size_t(i)];
        if (inliers_i_[std::size_t(i)]) ++n_inliers_i_;
    }
}

bool MLPnPSolver::iterate(int n, bool& no_more, std::vector<bool>& inliers, int& n_inliers, Eigen::Matrix4f& Tcw) {
    Tcw.setIdentity();
    no_more = false;
    inliers.clear();
    n_inliers = 0;
    if (N_ < min_inliers_) {
        no_more = true;
        return false;
    }
    const auto toTcw = [](const Eigen::Matrix<double, 3, 4>& T) {
        Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
        M.topRows<3>() = T.cast<float>();
        return M;
    };
    const auto perFrame = [&](const std::vector<bool>& flags) {
        std::vector<bool> out(frame_size_, false);
        for (int i = 0; i < N_; ++i)
            if (flags[std::size_t(i)]) out[std::size_t(keypoint_index_[std::size_t(i)])] = true;
        return out;
    };

    int current = 0;
    while (iterations_ < max_iterations_ || current < n) {
        ++current;
        ++iterations_;
        std::vector<int> available = all_indices_;
        std::vector<Vec3> bearings(static_cast<std::size_t>(min_set_)), points(static_cast<std::size_t>(min_set_));
        for (int i = 0; i < min_set_; ++i) {
            std::uniform_int_distribution<int> pick(0, int(available.size()) - 1);
            const int randi = pick(rng_);
            const int idx = available[std::size_t(randi)];
            bearings[std::size_t(i)] = bearings_[std::size_t(idx)];
            points[std::size_t(i)] = p3d_[std::size_t(idx)];
            available[std::size_t(randi)] = available.back();
            available.pop_back();
        }
        Ti_ = computePose(bearings, points);
        checkInliers();
        if (n_inliers_i_ >= min_inliers_) {
            if (n_inliers_i_ > best_n_inliers_) {
                best_inliers_ = inliers_i_;
                best_n_inliers_ = n_inliers_i_;
                best_Tcw_ = toTcw(Ti_);
            }
            // Refine: the original's re-solved pose is discarded; the current hypothesis is checked again
            checkInliers();
            refined_n_inliers_ = n_inliers_i_;
            refined_inliers_ = inliers_i_;
            if (n_inliers_i_ > min_inliers_) {
                refined_Tcw_ = toTcw(Ti_);
                n_inliers = refined_n_inliers_;
                inliers = perFrame(refined_inliers_);
                Tcw = refined_Tcw_;
                if (iterations_ >= max_iterations_) no_more = true;
                return true;
            }
        }
    }
    if (iterations_ >= max_iterations_) {
        no_more = true;
        if (best_n_inliers_ >= min_inliers_) {
            n_inliers = best_n_inliers_;
            inliers = perFrame(best_inliers_);
            Tcw = best_Tcw_;
            return true;
        }
    }
    return false;
}

}  // namespace orb_tracking
