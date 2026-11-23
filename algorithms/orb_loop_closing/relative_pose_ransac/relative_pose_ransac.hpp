#pragma once
// RelativePoseRansac: ORB-SLAM3's Sim3Solver with bFixScale = true (stereo and stereo-inertial), so the
// similarity is a rigid transform T12 (kf2's camera into kf1's). Algorithms/orb_loop_closing/docs/DESIGN.md,
// "以 SE3 取代 Sim3".
//
// As the original:
//   - a correspondence is feature i1 of kf1 with its own good point and a good matched point; each point's
//     octave is that of its observation in its keyframe (kf1; for the matched one, its keyframe from
//     `matched_kfs`, kf2 when not given): the left keypoint's, or the right one's when seen in the right view
//     only; no observation there: dropped. Both points are taken into kf1's and kf2's camera with those
//     keyframes' poses (all matched points with kf2's, whichever keyframe they came from);
//   - error bounds 9.210 · σ² of each octave, both reprojections (KB8) under their bound for an inlier;
//   - RANSAC on 3 points with Horn's closed form (quaternion from the 4x4 N matrix, in float), iterations
//     ceil(log(1 - p) / log(1 - ε³)) capped at max_iterations, ε = min_inliers / N;
//   - iterate(n): n more iterations at most; converges when a hypothesis has more than min_inliers inliers
//     (and at least as many as the best so far).
// The random sampling uses std::mt19937 (the original: DUtils::Random, rand()), seeded per solver.
#include <random>
#include <vector>

#include <Eigen/Core>

#include "types/geometry/se3.hpp"
#include "types/map/atlas.hpp"

namespace orb_loop_closing {

class RelativePoseRansac {
public:
    // matched12: per kf1 feature (size kf1.size()), the matched point (kNone: none). matched_kfs: empty, or per
    // kf1 feature the keyframe the matched point came from.
    RelativePoseRansac(const map::Atlas& atlas, const map::KeyFrame& kf1, const map::KeyFrame& kf2,
                       const std::vector<map::MapPointId>& matched12,
                       const std::vector<map::KeyFrameId>& matched_kfs = {}, unsigned seed = 0);

    void setRansacParameters(double probability = 0.99, int min_inliers = 6, int max_iterations = 300);

    // Up to n more iterations. converged: a hypothesis with more than min_inliers inliers, then inliers (per kf1
    // feature) and n_inliers are set and the result is that hypothesis. no_more: max_iterations reached or too
    // few correspondences.
    geometry::SE3f iterate(int n, bool& no_more, std::vector<bool>& inliers, int& n_inliers, bool& converged);

    int correspondences() const { return N_; }
    // The best hypothesis so far (GetEstimatedRotation / GetEstimatedTranslation)
    const Eigen::Matrix3f& bestRotation() const { return best_R_; }
    const Eigen::Vector3f& bestTranslation() const { return best_t_; }

private:
    void computeTransform(const Eigen::Matrix3f& P1, const Eigen::Matrix3f& P2);
    void checkInliers();

    const camera::KannalaBrandt8& cam1_;
    const camera::KannalaBrandt8& cam2_;
    int n1_ = 0;  // kf1.size()
    int N_ = 0;   // correspondences
    std::vector<int> indices1_;
    std::vector<Eigen::Vector3f> X3Dc1_, X3Dc2_;
    std::vector<Eigen::Vector2f> P1im1_, P2im2_;
    std::vector<float> max_error1_, max_error2_;
    std::vector<std::size_t> all_indices_;

    // Current hypothesis
    Eigen::Matrix3f R12_;
    Eigen::Vector3f t12_;
    std::vector<bool> inliers_i_;
    int n_inliers_i_ = 0;

    // Best
    std::vector<bool> best_inliers_;
    int best_n_inliers_ = 0;
    Eigen::Matrix3f best_R_ = Eigen::Matrix3f::Identity();
    Eigen::Vector3f best_t_ = Eigen::Vector3f::Zero();

    double probability_ = 0.99;
    int min_inliers_ = 6, max_iterations_ = 300, iterations_ = 0;
    std::mt19937 rng_;
};

}  // namespace orb_loop_closing
