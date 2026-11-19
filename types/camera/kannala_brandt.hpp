#pragma once
// Kannala-Brandt fisheye camera model (KB8), the same model as ORB-SLAM3's KannalaBrandt8,
// Kalibr's pinhole-equi and OpenCV's cv::fisheye:
//
//   θ = angle between the ray and the optical axis
//   r(θ) = θ + k1 θ³ + k2 θ⁵ + k3 θ⁷ + k4 θ⁹
//   u = fx · r(θ) · x / √(x² + y²) + cx,   v = fy · r(θ) · y / √(x² + y²) + cy
//
// Pixel centres are at integer coordinates (OpenCV / Kalibr convention). unproject() returns unit
// bearings, not z = 1 points as ORB-SLAM3 does, so it stays valid at and beyond 90°.
// The scalar functions compute in double; they are not the hot path (the matcher unprojects each
// keypoint once per frame).
#include <Eigen/Core>

namespace camera {

class KannalaBrandt8 {
public:
    KannalaBrandt8() = default;
    // Throws std::invalid_argument for fx, fy, width or height <= 0.
    KannalaBrandt8(float fx, float fy, float cx, float cy, float k1, float k2, float k3, float k4,
                   int width, int height);

    // Pixel of a point in camera coordinates. False for the zero vector and for rays beyond
    // max_theta() (where r(θ) is no longer increasing and the model has no inverse).
    bool project(const Eigen::Vector3f& p, Eigen::Vector2f& uv) const;
    // Unit bearing of a pixel. False when the pixel is beyond r(max_theta()).
    bool unproject(const Eigen::Vector2f& uv, Eigen::Vector3f& bearing) const;
    // d(u, v) / d(x, y, z) at p (p != 0, within max_theta()).
    Eigen::Matrix<float, 2, 3> project_jacobian(const Eigen::Vector3f& p) const;

    // Smallest image displacement (pixels) per radian of bearing rotation at angle θ from the
    // optical axis: min over the radial (r'(θ)) and tangential (r(θ) / sin θ) directions, times
    // min(fx, fy). Converts pixel noise to angular noise: σ_ang = σ_px / pixels_per_radian(θ).
    float pixels_per_radian(float theta) const;

    // The same lens for the image resampled to width x height (binning, ISP or GPU scaling with
    // pixel-centre alignment): fx' = fx·s, cx' = (cx + 0.5)·s − 0.5; k1..k4 act on θ and do not
    // change. Throws std::invalid_argument if the aspect ratio changes (that would be a crop).
    KannalaBrandt8 resized(int width, int height) const;

    float fx() const { return float(fx_); }
    float fy() const { return float(fy_); }
    float cx() const { return float(cx_); }
    float cy() const { return float(cy_); }
    float k(int i) const { return float(k_[i]); }   // i = 0..3 -> k1..k4
    int width() const { return width_; }
    int height() const { return height_; }
    float max_theta() const { return float(theta_max_); }   // radians

private:
    double r(double theta) const;
    double dr(double theta) const;

    double fx_ = 1, fy_ = 1, cx_ = 0, cy_ = 0;
    double k_[4] = {0, 0, 0, 0};
    int width_ = 0, height_ = 0;
    double theta_max_ = 0, r_max_ = 0;   // r(θ) increasing on [0, theta_max_]
};

}  // namespace camera
