#pragma once
// ORB-SLAM3's MLPnPsolver (Urban et al., MLPnP; only Tracking::Relocalization uses it): RANSAC over minimal sets
// of 6 left-camera 2D-3D correspondences, each solved by MLPnP without covariance (the linear solution in the
// bearings' null spaces, then at most 5 Gauss-Newton steps), hypotheses scored by the KB8 reprojection error.
//
// As the original:
//   - only left keypoints take part; bearings are ORB-SLAM3's unproject (z = 1, not normalized);
//   - iterate(n) runs while the total is below the RANSAC maximum OR this call's count is below n, so one call
//     can run all the remaining iterations;
//   - Refine re-solves on the best inliers but then checks and returns the hypothesis of the current
//     iteration (the re-solved pose is never used): here it checks the current hypothesis directly;
//   - once the iterations are used up, a refined pose also reports no_more (the reference's fix of v1.0's
//     endless relocalization).
// The Gauss-Newton Jacobian is the analytic one, d(R(w)X)/dw = -R [X]x Jr(w), where the original has a
// generated expansion of the same derivative. The minimal sets come from this solver's own generator (seeded),
// so the hypotheses differ from the original's draws; the result agrees within the RANSAC tolerance.
#include <random>
#include <vector>

#include <Eigen/Core>

#include "types/camera/kannala_brandt.hpp"
#include "types/tracking/frame.hpp"

namespace orb_tracking {

class MLPnPSolver {
public:
    // matches: per frame index, the matched map point's world position (nullptr: none or bad). camera: the
    // left camera.
    MLPnPSolver(const tracking::Frame& frame, const camera::KannalaBrandt8& camera,
                const std::vector<const Eigen::Vector3f*>& matches, unsigned seed = 0);

    // SetRansacParameters (Relocalization: 0.99, 10, 300, 6, 0.5, 5.991)
    void setRansacParameters(double probability = 0.99, int min_inliers = 8, int max_iterations = 300, int min_set = 6,
                             float epsilon = 0.4f, float th2 = 5.991f);

    // Up to n more RANSAC iterations. True with a pose (Tcw) and its inliers per frame index; no_more when the
    // iterations are used up.
    bool iterate(int n, bool& no_more, std::vector<bool>& inliers, int& n_inliers, Eigen::Matrix4f& Tcw);

    int correspondences() const { return N_; }
    int minInliers() const { return min_inliers_; }
    int maxIterations() const { return max_iterations_; }
    const std::vector<float>& maxError() const { return max_error_; }

    // MLPnP on the given bearings (z = 1 rays) and world points, at least 6: [Rcw | tcw]
    static Eigen::Matrix<double, 3, 4> computePose(const std::vector<Eigen::Vector3d>& bearings,
                                                  const std::vector<Eigen::Vector3d>& points);

private:
    void checkInliers();

    const camera::KannalaBrandt8& camera_;
    std::vector<Eigen::Vector2f> p2d_;
    std::vector<float> sigma2_;
    std::vector<Eigen::Vector3d> bearings_, p3d_;
    std::vector<int> keypoint_index_, all_indices_;
    std::size_t frame_size_ = 0;

    // RANSAC
    double probability_ = 0.99;
    int min_inliers_ = 8, max_iterations_ = 300, min_set_ = 6;
    float epsilon_ = 0.4f;
    std::vector<float> max_error_;
    int N_ = 0;
    std::mt19937 rng_;

    // Current hypothesis, best so far, refined
    Eigen::Matrix<double, 3, 4> Ti_;
    std::vector<bool> inliers_i_, best_inliers_, refined_inliers_;
    int n_inliers_i_ = 0, best_n_inliers_ = 0, refined_n_inliers_ = 0, iterations_ = 0;
    Eigen::Matrix4f best_Tcw_ = Eigen::Matrix4f::Identity(), refined_Tcw_ = Eigen::Matrix4f::Identity();
};

}  // namespace orb_tracking
