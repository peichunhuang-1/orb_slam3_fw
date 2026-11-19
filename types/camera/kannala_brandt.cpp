#include "types/camera/kannala_brandt.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace camera {

KannalaBrandt8::KannalaBrandt8(float fx, float fy, float cx, float cy, float k1, float k2, float k3,
                               float k4, int width, int height)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), k_{k1, k2, k3, k4}, width_(width), height_(height) {
    if (!(fx > 0) || !(fy > 0)) throw std::invalid_argument("KannalaBrandt8: fx and fy must be > 0");
    if (width <= 0 || height <= 0) throw std::invalid_argument("KannalaBrandt8: width and height must be > 0");
    // Valid range: r(θ) increasing. Scan up to π in 1 mrad steps (calibrated lenses stay
    // monotonic well past their FOV; strong negative k1 can turn back earlier).
    const double pi = 3.14159265358979323846, step = 1e-3;
    double t = 0;
    while (t + step <= pi && dr(t + step) > 0) t += step;
    theta_max_ = t;
    r_max_ = r(t);
}

double KannalaBrandt8::r(double t) const {
    const double t2 = t * t;
    return t * (1 + t2 * (k_[0] + t2 * (k_[1] + t2 * (k_[2] + t2 * k_[3]))));
}

double KannalaBrandt8::dr(double t) const {
    const double t2 = t * t;
    return 1 + t2 * (3 * k_[0] + t2 * (5 * k_[1] + t2 * (7 * k_[2] + t2 * 9 * k_[3])));
}

bool KannalaBrandt8::project(const Eigen::Vector3f& p, Eigen::Vector2f& uv) const {
    const double x = p.x(), y = p.y(), z = p.z();
    const double rho = std::sqrt(x * x + y * y);
    if (rho == 0 && z == 0) return false;
    const double theta = std::atan2(rho, z);
    if (theta > theta_max_) return false;
    if (rho < 1e-12 * std::abs(z)) {   // on the optical axis (r(θ) / ρ -> 1 / z)
        uv = Eigen::Vector2f(float(cx_), float(cy_));
        return true;
    }
    const double s = r(theta) / rho;
    uv = Eigen::Vector2f(float(fx_ * s * x + cx_), float(fy_ * s * y + cy_));
    return true;
}

bool KannalaBrandt8::unproject(const Eigen::Vector2f& uv, Eigen::Vector3f& bearing) const {
    const double mx = (uv.x() - cx_) / fx_, my = (uv.y() - cy_) / fy_;
    const double rd = std::sqrt(mx * mx + my * my);
    if (rd > r_max_ * (1 + 1e-9)) return false;
    if (rd < 1e-12) {
        bearing = Eigen::Vector3f(0, 0, 1);
        return true;
    }
    // Newton on r(θ) = rd; r is increasing on [0, θmax] and r(θ) ≈ θ, so θ = rd is a good start.
    double theta = std::min(rd, theta_max_);
    for (int i = 0; i < 20; ++i) {
        const double fix = (r(theta) - rd) / dr(theta);
        theta = std::clamp(theta - fix, 0.0, theta_max_);
        if (std::abs(fix) < 1e-12) break;
    }
    const double s = std::sin(theta) / rd;
    bearing = Eigen::Vector3f(float(mx * s), float(my * s), float(std::cos(theta)));
    return true;
}

Eigen::Matrix<float, 2, 3> KannalaBrandt8::project_jacobian(const Eigen::Vector3f& p) const {
    const double x = p.x(), y = p.y(), z = p.z();
    const double rho2 = x * x + y * y, rho = std::sqrt(rho2);
    Eigen::Matrix<float, 2, 3> J;
    if (rho < 1e-9 * std::abs(z)) {   // pinhole limit on the optical axis
        J << float(fx_ / z), 0, float(-fx_ * x / (z * z)),
             0, float(fy_ / z), float(-fy_ * y / (z * z));
        return J;
    }
    const double n2 = rho2 + z * z;
    const double theta = std::atan2(rho, z);
    const double rt = r(theta), drt = dr(theta);
    // d(r·x/ρ): r'·∂θ·x/ρ + r·∂(x/ρ), with ∂θ/∂(x, y, z) = (z·x / (ρ·n²), z·y / (ρ·n²), −ρ / n²)
    const double a = drt * z / (rho2 * n2);   // r'·z / (ρ²·n²)
    const double b = rt / (rho2 * rho);       // r / ρ³
    J << float(fx_ * (a * x * x + b * y * y)), float(fx_ * (a - b) * x * y), float(-fx_ * drt * x / n2),
         float(fy_ * (a - b) * x * y), float(fy_ * (a * y * y + b * x * x)), float(-fy_ * drt * y / n2);
    return J;
}

// The parameters are floats (ORB-SLAM3's mvParameters); fx_ ... hold them exactly.
Eigen::Vector2d KannalaBrandt8::project_orb(const Eigen::Vector3d& v) const {
    const float p0 = float(fx_), p1 = float(fy_), p2 = float(cx_), p3 = float(cy_);
    const float p4 = float(k_[0]), p5 = float(k_[1]), p6 = float(k_[2]), p7 = float(k_[3]);
    const double x2_plus_y2 = v[0] * v[0] + v[1] * v[1];
    const double theta = atan2f(sqrtf(float(x2_plus_y2)), float(v[2]));
    const double psi = atan2f(float(v[1]), float(v[0]));
    const double theta2 = theta * theta;
    const double theta3 = theta * theta2;
    const double theta5 = theta3 * theta2;
    const double theta7 = theta5 * theta2;
    const double theta9 = theta7 * theta2;
    const double r = theta + p4 * theta3 + p5 * theta5 + p6 * theta7 + p7 * theta9;
    return Eigen::Vector2d(p0 * r * std::cos(psi) + p2, p1 * r * std::sin(psi) + p3);
}

Eigen::Vector2f KannalaBrandt8::project_orb(const Eigen::Vector3f& v) const {
    const float p0 = float(fx_), p1 = float(fy_), p2 = float(cx_), p3 = float(cy_);
    const float p4 = float(k_[0]), p5 = float(k_[1]), p6 = float(k_[2]), p7 = float(k_[3]);
    const float x2_plus_y2 = v[0] * v[0] + v[1] * v[1];
    const float theta = atan2f(sqrtf(x2_plus_y2), v[2]);
    const float psi = atan2f(v[1], v[0]);
    const float theta2 = theta * theta;
    const float theta3 = theta * theta2;
    const float theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2;
    const float theta9 = theta7 * theta2;
    const float r = theta + p4 * theta3 + p5 * theta5 + p6 * theta7 + p7 * theta9;
    return Eigen::Vector2f(p0 * r * std::cos(psi) + p2, p1 * r * std::sin(psi) + p3);
}

Eigen::Matrix<double, 2, 3> KannalaBrandt8::project_jacobian_orb(const Eigen::Vector3d& v) const {
    const float p0 = float(fx_), p1 = float(fy_);
    const float p4 = float(k_[0]), p5 = float(k_[1]), p6 = float(k_[2]), p7 = float(k_[3]);
    const double x2 = v[0] * v[0], y2 = v[1] * v[1], z2 = v[2] * v[2];
    const double r2 = x2 + y2;
    const double r = std::sqrt(r2);
    const double r3 = r2 * r;
    const double theta = std::atan2(r, v[2]);
    const double theta2 = theta * theta, theta3 = theta2 * theta;
    const double theta4 = theta2 * theta2, theta5 = theta4 * theta;
    const double theta6 = theta2 * theta4, theta7 = theta6 * theta;
    const double theta8 = theta4 * theta4, theta9 = theta8 * theta;
    const double f = theta + theta3 * p4 + theta5 * p5 + theta7 * p6 + theta9 * p7;
    const double fd = 1 + 3 * p4 * theta2 + 5 * p5 * theta4 + 7 * p6 * theta6 + 9 * p7 * theta8;
    Eigen::Matrix<double, 2, 3> J;
    J(0, 0) = p0 * (fd * v[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
    J(1, 0) = p1 * (fd * v[2] * v[1] * v[0] / (r2 * (r2 + z2)) - f * v[1] * v[0] / r3);
    J(0, 1) = p0 * (fd * v[2] * v[1] * v[0] / (r2 * (r2 + z2)) - f * v[1] * v[0] / r3);
    J(1, 1) = p1 * (fd * v[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);
    J(0, 2) = -p0 * fd * v[0] / (r2 + z2);
    J(1, 2) = -p1 * fd * v[1] / (r2 + z2);
    return J;
}

Eigen::Vector3f KannalaBrandt8::unproject_orb(const Eigen::Vector2f& p2D) const {
    const float p0 = float(fx_), p1 = float(fy_), p2 = float(cx_), p3 = float(cy_);
    const float p4 = float(k_[0]), p5 = float(k_[1]), p6 = float(k_[2]), p7 = float(k_[3]);
    const float precision = 1e-6f;
    const float pw_x = (p2D.x() - p2) / p0, pw_y = (p2D.y() - p3) / p1;
    float scale = 1.f;
    const double pi = 3.14159265358979323846;
    float theta_d = sqrtf(pw_x * pw_x + pw_y * pw_y);
    theta_d = fminf(fmaxf(float(-pi / 2.f), theta_d), float(pi / 2.f));
    if (theta_d > 1e-8) {
        float theta = theta_d;
        for (int j = 0; j < 10; j++) {
            const float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta4 * theta4;
            const float k0_theta2 = p4 * theta2, k1_theta4 = p5 * theta4;
            const float k2_theta6 = p6 * theta6, k3_theta8 = p7 * theta8;
            const float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                                    (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
            theta = theta - theta_fix;
            if (fabsf(theta_fix) < precision) break;
        }
        scale = std::tan(theta) / theta_d;
    }
    return Eigen::Vector3f(pw_x * scale, pw_y * scale, 1.f);
}

float KannalaBrandt8::pixels_per_radian(float theta) const {
    const double f = std::min(fx_, fy_);
    if (theta < 1e-6f) return float(f);
    const double tangential = r(theta) / std::sin(theta);
    return float(f * std::min(dr(theta), tangential));
}

KannalaBrandt8 KannalaBrandt8::resized(int width, int height) const {
    if (width <= 0 || height <= 0) throw std::invalid_argument("KannalaBrandt8::resized: size must be > 0");
    const double sx = double(width) / width_, sy = double(height) / height_;
    if (std::abs(sx - sy) > 1e-3 * std::max(sx, sy))
        throw std::invalid_argument("KannalaBrandt8::resized: aspect ratio changes (" + std::to_string(width_) +
                                    "x" + std::to_string(height_) + " -> " + std::to_string(width) + "x" +
                                    std::to_string(height) + "); a crop needs its own calibration");
    return KannalaBrandt8(float(fx_ * sx), float(fy_ * sy), float((cx_ + 0.5) * sx - 0.5),
                          float((cy_ + 0.5) * sy - 0.5), float(k_[0]), float(k_[1]), float(k_[2]),
                          float(k_[3]), width, height);
}

}  // namespace camera
