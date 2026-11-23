#include "algorithms/orb_loop_closing/relative_pose_ransac/relative_pose_ransac.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace orb_loop_closing {

namespace {

// The octave of the point's observation in kf: the left keypoint's, else the right one's (-1: not seen there)
int octaveIn(const map::MapPoint& p, const map::KeyFrame& kf) {
    const map::Observation o = p.indexIn(kf.id);
    const orb::FrameFeatures& f = *kf.data->features;
    if (o.left >= 0) return f.keypoint(o.left).octave;
    if (o.right >= f.n_left) return f.keypoint(o.right).octave;
    return -1;
}

}  // namespace

RelativePoseRansac::RelativePoseRansac(const map::Atlas& a, const map::KeyFrame& kf1, const map::KeyFrame& kf2,
                                       const std::vector<map::MapPointId>& matched12,
                                       const std::vector<map::KeyFrameId>& matched_kfs, unsigned seed)
    : cam1_(a.rig(kf1.data->rig).camera(0)), cam2_(a.rig(kf2.data->rig).camera(0)), rng_(seed) {
    n1_ = int(matched12.size());
    const geometry::SE3f T1w = kf1.pose(), T2w = kf2.pose();
    const Eigen::Matrix3f R1w = T1w.rotationMatrix(), R2w = T2w.rotationMatrix();
    const Eigen::Vector3f t1w = T1w.translation(), t2w = T2w.translation();
    const orb::FrameFeatures& f1 = *kf1.data->features;

    std::size_t idx = 0;
    for (int i1 = 0; i1 < n1_; ++i1) {
        if (matched12[std::size_t(i1)] == map::kNone) continue;
        const map::MapPoint* p1 = a.point(kf1.points[std::size_t(i1)]);
        const map::MapPoint* p2 = a.point(matched12[std::size_t(i1)]);
        if (!p1) continue;
        if (p1->bad || !p2 || p2->bad) continue;

        const map::KeyFrame* kfm = &kf2;
        if (!matched_kfs.empty() && matched_kfs[std::size_t(i1)] != map::kNone)
            if (const map::KeyFrame* k = a.keyframe(matched_kfs[std::size_t(i1)])) kfm = k;
        const int octave1 = octaveIn(*p1, kf1);
        const int octave2 = octaveIn(*p2, *kfm);
        if (octave1 < 0 || octave2 < 0) continue;

        const float sigma2_1 = f1.level_sigma2[std::size_t(octave1)];
        const float sigma2_2 = kfm->data->features->level_sigma2[std::size_t(octave2)];
        max_error1_.push_back(9.210f * sigma2_1);
        max_error2_.push_back(9.210f * sigma2_2);
        indices1_.push_back(i1);
        X3Dc1_.push_back(R1w * p1->pos + t1w);
        X3Dc2_.push_back(R2w * p2->pos + t2w);
        all_indices_.push_back(idx++);
    }
    for (const Eigen::Vector3f& X : X3Dc1_) P1im1_.push_back(cam1_.project_orb(X));
    for (const Eigen::Vector3f& X : X3Dc2_) P2im2_.push_back(cam2_.project_orb(X));
    setRansacParameters();
}

void RelativePoseRansac::setRansacParameters(double probability, int min_inliers, int max_iterations) {
    probability_ = probability;
    min_inliers_ = min_inliers;
    max_iterations_ = max_iterations;
    N_ = int(X3Dc1_.size());
    inliers_i_.assign(std::size_t(N_), false);

    const float epsilon = float(min_inliers_) / float(N_);
    int n_iterations;
    if (min_inliers_ == N_)
        n_iterations = 1;
    else
        n_iterations = int(std::ceil(std::log(1 - probability_) / std::log(1 - std::pow(epsilon, 3))));
    max_iterations_ = std::max(1, std::min(n_iterations, max_iterations_));
    iterations_ = 0;
}

geometry::SE3f RelativePoseRansac::iterate(int n, bool& no_more, std::vector<bool>& inliers, int& n_inliers,
                                           bool& converged) {
    no_more = false;
    converged = false;
    inliers.assign(std::size_t(n1_), false);
    n_inliers = 0;
    if (N_ < min_inliers_) {
        no_more = true;
        return geometry::SE3f();
    }

    Eigen::Matrix3f P1, P2;
    int current = 0;
    while (iterations_ < max_iterations_ && current < n) {
        ++current;
        ++iterations_;
        std::vector<std::size_t> available = all_indices_;
        for (int i = 0; i < 3; ++i) {
            std::uniform_int_distribution<int> pick(0, int(available.size()) - 1);
            const int r = pick(rng_);
            const std::size_t k = available[std::size_t(r)];
            P1.col(i) = X3Dc1_[k];
            P2.col(i) = X3Dc2_[k];
            available[std::size_t(r)] = available.back();
            available.pop_back();
        }
        computeTransform(P1, P2);
        checkInliers();
        if (n_inliers_i_ >= best_n_inliers_) {
            best_inliers_ = inliers_i_;
            best_n_inliers_ = n_inliers_i_;
            best_R_ = R12_;
            best_t_ = t12_;
            if (n_inliers_i_ > min_inliers_) {
                n_inliers = n_inliers_i_;
                for (int i = 0; i < N_; ++i)
                    if (inliers_i_[std::size_t(i)]) inliers[std::size_t(indices1_[std::size_t(i)])] = true;
                converged = true;
                return geometry::SE3f(best_R_, best_t_);
            }
        }
    }
    if (iterations_ >= max_iterations_) no_more = true;
    return geometry::SE3f(best_R_, best_t_);
}

// Horn 1987, closed-form absolute orientation with unit quaternions; scale fixed to 1
void RelativePoseRansac::computeTransform(const Eigen::Matrix3f& P1, const Eigen::Matrix3f& P2) {
    const Eigen::Vector3f O1 = P1.rowwise().sum() / 3.0f, O2 = P2.rowwise().sum() / 3.0f;
    Eigen::Matrix3f Pr1, Pr2;
    for (int i = 0; i < 3; ++i) {
        Pr1.col(i) = P1.col(i) - O1;
        Pr2.col(i) = P2.col(i) - O2;
    }
    const Eigen::Matrix3f M = Pr2 * Pr1.transpose();

    const double N11 = M(0, 0) + M(1, 1) + M(2, 2), N12 = M(1, 2) - M(2, 1), N13 = M(2, 0) - M(0, 2),
                 N14 = M(0, 1) - M(1, 0), N22 = M(0, 0) - M(1, 1) - M(2, 2), N23 = M(0, 1) + M(1, 0),
                 N24 = M(2, 0) + M(0, 2), N33 = -M(0, 0) + M(1, 1) - M(2, 2), N34 = M(1, 2) + M(2, 1),
                 N44 = -M(0, 0) - M(1, 1) + M(2, 2);
    Eigen::Matrix4f N;
    N << float(N11), float(N12), float(N13), float(N14),
         float(N12), float(N22), float(N23), float(N24),
         float(N13), float(N23), float(N33), float(N34),
         float(N14), float(N24), float(N34), float(N44);

    Eigen::EigenSolver<Eigen::Matrix4f> eig;
    eig.compute(N);
    const Eigen::Vector4f eval = eig.eigenvalues().real();
    const Eigen::Matrix4f evec = eig.eigenvectors().real();
    int max_index;
    eval.maxCoeff(&max_index);

    // The rotation vector from the quaternion (the reference's fix: no rotation when the imaginary part is 0)
    Eigen::Vector3f vec = evec.block<3, 1>(1, max_index);
    const double ang = std::atan2(double(vec.norm()), double(evec(0, max_index)));
    const float vnorm = vec.norm();
    if (vnorm > 1e-9f && std::isfinite(vnorm) && std::isfinite(evec(0, max_index)))
        vec = float(2 * ang) * vec / vnorm;
    else
        vec.setZero();
    const float theta = vec.norm();
    R12_ = theta > 0 ? Eigen::AngleAxisf(theta, vec / theta).toRotationMatrix() : Eigen::Matrix3f::Identity();

    t12_ = O1 - R12_ * O2;
}

void RelativePoseRansac::checkInliers() {
    const Eigen::Matrix3f R21 = R12_.transpose();
    const Eigen::Vector3f t21 = -R21 * t12_;
    n_inliers_i_ = 0;
    for (int i = 0; i < N_; ++i) {
        const std::size_t k = std::size_t(i);
        const Eigen::Vector2f p2im1 = cam1_.project_orb(Eigen::Vector3f(R12_ * X3Dc2_[k] + t12_));
        const Eigen::Vector2f p1im2 = cam2_.project_orb(Eigen::Vector3f(R21 * X3Dc1_[k] + t21));
        const Eigen::Vector2f d1 = P1im1_[k] - p2im1, d2 = p1im2 - P2im2_[k];
        const bool in = d1.dot(d1) < max_error1_[k] && d2.dot(d2) < max_error2_[k];
        inliers_i_[k] = in;
        if (in) ++n_inliers_i_;
    }
}

}  // namespace orb_loop_closing
