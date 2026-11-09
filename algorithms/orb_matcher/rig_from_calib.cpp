#include "algorithms/orb_matcher/rig_from_calib.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

namespace orb_matcher {

void Radtan::distort(double x, double y, double& xd, double& yd) const {
    const double r2 = x * x + y * y, rad = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
}

bool Radtan::undistort(double xd, double yd, double& x, double& y) const {
    x = xd;
    y = yd;
    for (int i = 0; i < 200; ++i) {
        const double r2 = x * x + y * y, rad = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
        x = (xd - (2 * p1 * x * y + p2 * (r2 + 2 * x * x))) / rad;
        y = (yd - (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)) / rad;
    }
    double ex, ey;
    distort(x, y, ex, ey);
    return std::isfinite(x) && std::isfinite(y) && std::hypot(ex - xd, ey - yd) < 1e-10;
}

KannalaBrandt8 fitKannalaBrandt8(const Radtan& rt, int W, int H, ModelFit* fit) {
    if (W <= 0 || H <= 0) throw std::invalid_argument("fitKannalaBrandt8: image size must be > 0");
    std::vector<Eigen::Vector2d> und, pix;
    for (int v = 0; v < H; v += 4)
        for (int u = 0; u < W; u += 4) {
            double x, y;
            if (rt.undistort((u - rt.cx) / rt.fx, (v - rt.cy) / rt.fy, x, y)) {
                und.emplace_back(x, y);
                pix.emplace_back(u, v);
            }
        }
    if (und.size() < 16) throw std::invalid_argument("fitKannalaBrandt8: the distortion model does not invert over the image");
    // pixel = c + f · r(θ) · (x, y) / ρ, r(θ) = θ + k1 θ³ + ... + k4 θ⁹: linear in k1..k4
    Eigen::MatrixXd A(2 * und.size(), 4);
    Eigen::VectorXd b(2 * und.size());
    for (size_t i = 0; i < und.size(); ++i) {
        const double rho = und[i].norm(), th = std::atan(rho);
        const double ux = rho > 0 ? und[i].x() / rho : 0, uy = rho > 0 ? und[i].y() / rho : 0;
        double tp = th * th * th;
        for (int k = 0; k < 4; ++k, tp *= th * th) {
            A(2 * i, k) = rt.fx * tp * ux;
            A(2 * i + 1, k) = rt.fy * tp * uy;
        }
        b(2 * i) = pix[i].x() - rt.cx - rt.fx * th * ux;
        b(2 * i + 1) = pix[i].y() - rt.cy - rt.fy * th * uy;
    }
    const Eigen::Vector4d k = A.colPivHouseholderQr().solve(b);
    const KannalaBrandt8 cam(float(rt.fx), float(rt.fy), float(rt.cx), float(rt.cy), float(k[0]), float(k[1]),
                             float(k[2]), float(k[3]), W, H);
    if (fit) {
        ModelFit f;
        f.fitted = true;
        double sum = 0;
        std::vector<double> errs;
        errs.reserve(und.size());
        for (size_t i = 0; i < und.size(); ++i) {   // through the class (float), not the fit
            Eigen::Vector2f uv;
            const double e = cam.project(Eigen::Vector3f(float(und[i].x()), float(und[i].y()), 1.f), uv)
                                 ? (uv.cast<double>() - pix[i]).norm()
                                 : 1e9;
            f.max_px = std::max(f.max_px, e);
            if (pix[i].x() >= 19 && pix[i].x() < W - 19 && pix[i].y() >= 19 && pix[i].y() < H - 19)
                f.inner_max_px = std::max(f.inner_max_px, e);
            sum += e * e;
            errs.push_back(e);
        }
        f.rms_px = std::sqrt(sum / double(errs.size()));
        std::sort(errs.begin(), errs.end());
        f.p99_px = errs[size_t(0.99 * double(errs.size() - 1))];
        *fit = f;
    }
    return cam;
}

KannalaBrandt8 cameraFromCalib(const calib::PinholeCamera& c, ModelFit* fit) {
    const std::string& m = c.distortion_model;
    const std::string who = "orb_matcher: camera '" + c.frame_id + "'";
    if (c.width <= 0 || c.height <= 0) throw std::invalid_argument(who + ": image size must be > 0");
    if (m == "equidistant" || m == "kannala_brandt" || m == "fisheye" || m == "KannalaBrandt8") {
        if (c.d.size() < 4) throw std::invalid_argument(who + ": distortion '" + m + "' needs k1 k2 k3 k4");
        if (fit) *fit = ModelFit();
        return KannalaBrandt8(float(c.fx), float(c.fy), float(c.cx), float(c.cy), float(c.d[0]), float(c.d[1]),
                              float(c.d[2]), float(c.d[3]), c.width, c.height);
    }
    Radtan rt;
    rt.fx = c.fx;
    rt.fy = c.fy;
    rt.cx = c.cx;
    rt.cy = c.cy;
    if (m == "plumb_bob" || m == "radtan") {
        if (c.d.size() != 4 && c.d.size() != 5)
            throw std::invalid_argument(who + ": distortion '" + m + "' needs k1 k2 p1 p2 [k3], got " +
                                        std::to_string(c.d.size()) + " coefficients");
        rt.k1 = c.d[0];
        rt.k2 = c.d[1];
        rt.p1 = c.d[2];
        rt.p2 = c.d[3];
        if (c.d.size() == 5) rt.k3 = c.d[4];
    } else if (m.empty() || m == "none") {
        for (double x : c.d)
            if (x != 0) throw std::invalid_argument(who + ": no distortion model but non-zero coefficients");
    } else {
        throw std::invalid_argument(who + ": distortion model '" + m +
                                    "' not supported (equidistant, kannala_brandt, plumb_bob, radtan, none)");
    }
    if (!(rt.fx > 0) || !(rt.fy > 0)) throw std::invalid_argument(who + ": fx and fy must be > 0");
    return fitKannalaBrandt8(rt, c.width, c.height, fit);
}

StereoRig stereoRigFromCalib(const calib::StereoImuRig& rig, ModelFit* left_fit, ModelFit* right_fit) {
    const KannalaBrandt8 left = cameraFromCalib(rig.left, left_fit);
    const KannalaBrandt8 right = cameraFromCalib(rig.right, right_fit);
    const Eigen::Isometry3d T_12 = rig.T_body_left.inverse() * rig.T_body_right;   // p_c1 = T_12 · p_c2
    return StereoRig(left, right, T_12.linear().cast<float>(), T_12.translation().cast<float>());
}

}  // namespace orb_matcher
