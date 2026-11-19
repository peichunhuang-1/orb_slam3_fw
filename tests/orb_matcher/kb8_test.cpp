// kb8_test: KB8 camera model (camera::KannalaBrandt8) and stereo rig (camera::StereoRig).
//
//   kb8_test
//
// 1. pixel -> bearing -> pixel and bearing -> pixel -> bearing round trips
// 2. same results as ORB-SLAM3's KannalaBrandt8 (project / unproject transcribed below)
// 3. project_jacobian against finite differences
// 4. pixels_per_radian against measured pixel motion per radian
// 5. valid range (max_theta) for a lens whose r(θ) turns back
// 6. resized(): 2x downscale maps pixels as (u + 0.5) / 2 − 0.5; aspect change rejected
// 7. camera::stereoRigFromCalib (calib::StereoImuRig: radtan -> KB8 fit, extrinsics) and its errors
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <random>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include "types/camera/kannala_brandt.hpp"
#include "types/camera/stereo_rig.hpp"
#include "types/camera/rig_from_calib.hpp"

using camera::KannalaBrandt8;
using camera::StereoRig;
using camera::stereoRigFromCalib;

static int failures = 0;
static void check(bool ok, const char* what) {
    printf("  %-62s %s\n", what, ok ? "ok" : "FAIL");
    failures += !ok;
}
template <class E>
static bool throws(const std::function<void()>& f) {
    try { f(); } catch (const E&) { return true; } catch (...) { return false; }
    return false;
}

static double angle(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    const Eigen::Vector3d x = a.cast<double>().normalized(), y = b.cast<double>().normalized();
    return std::atan2(x.cross(y).norm(), x.dot(y));
}
static Eigen::Vector3f bearingAt(double theta, double psi) {
    return Eigen::Vector3f(float(std::sin(theta) * std::cos(psi)), float(std::sin(theta) * std::sin(psi)),
                           float(std::cos(theta)));
}

// ORB-SLAM3 v1.0 KannalaBrandt8::project / unproject (src/CameraModels/KannalaBrandt8.cc),
// float as in the original. Parameters: fx fy cx cy k1 k2 k3 k4.
static Eigen::Vector2f orbProject(const float* P, const Eigen::Vector3f& v3D) {
    const float x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
    const float theta = atan2f(sqrtf(x2_plus_y2), v3D[2]);
    const float psi = atan2f(v3D[1], v3D[0]);
    const float theta2 = theta * theta, theta3 = theta * theta2, theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2, theta9 = theta7 * theta2;
    const float r = theta + P[4] * theta3 + P[5] * theta5 + P[6] * theta7 + P[7] * theta9;
    return Eigen::Vector2f(float(P[0] * r * cos(psi) + P[2]), float(P[1] * r * sin(psi) + P[3]));
}
static Eigen::Vector3f orbUnproject(const float* P, const Eigen::Vector2f& p2D) {
    const float precision = 1e-6f;
    const float pwx = (p2D.x() - P[2]) / P[0], pwy = (p2D.y() - P[3]) / P[1];
    float scale = 1.f;
    float theta_d = sqrtf(pwx * pwx + pwy * pwy);
    theta_d = fminf(fmaxf(-float(M_PI) / 2.f, theta_d), float(M_PI) / 2.f);
    if (theta_d > 1e-8) {
        float theta = theta_d;
        for (int j = 0; j < 10; j++) {
            const float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2,
                        theta8 = theta4 * theta4;
            const float k0_theta2 = P[4] * theta2, k1_theta4 = P[5] * theta4;
            const float k2_theta6 = P[6] * theta6, k3_theta8 = P[7] * theta8;
            const float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                                    (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
            theta = theta - theta_fix;
            if (fabsf(theta_fix) < precision) break;
        }
        scale = std::tan(theta) / theta_d;
    }
    return Eigen::Vector3f(pwx * scale, pwy * scale, 1.f);
}

struct TestCamera {
    const char* name;
    float P[8];         // fx fy cx cy k1 k2 k3 k4
    int width, height;
    double thetaTest;   // largest ray angle used in the tests
    camera::KannalaBrandt8 cam;
    TestCamera(const char* n, const float (&p)[8], int w, int h, double thetaDeg)
        : name(n), width(w), height(h), thetaTest(thetaDeg * M_PI / 180),
          cam(p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], w, h) {
        std::copy(p, p + 8, P);
    }
};

static void testCamera(const TestCamera& tc) {
    const camera::KannalaBrandt8& cam = tc.cam;
    printf("%s (%dx%d, max_theta %.1f deg)\n", tc.name, cam.width(), cam.height(), cam.max_theta() * 180 / M_PI);
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> uni(0, 1);
    char msg[128];

    // 1a. every 4th pixel of the image: pixel -> bearing -> pixel
    double maxPx = 0;
    int bad = 0;
    for (int v = 0; v < cam.height(); v += 4)
        for (int u = 0; u < cam.width(); u += 4) {
            const Eigen::Vector2f uv(u + 0.25f, v + 0.75f);
            Eigen::Vector3f f;
            Eigen::Vector2f back;
            if (!cam.unproject(uv, f) || std::abs(f.norm() - 1) > 1e-6f || !cam.project(f, back)) { ++bad; continue; }
            maxPx = std::max(maxPx, double((back - uv).norm()));
        }
    snprintf(msg, sizeof msg, "pixel -> bearing -> pixel, max %.2g px", maxPx);
    check(bad == 0 && maxPx < 1e-3, msg);

    // 1b. random bearings up to thetaTest: bearing -> pixel -> bearing
    double maxRad = 0;
    bad = 0;
    for (int i = 0; i < 20000; ++i) {
        const Eigen::Vector3f f = bearingAt(tc.thetaTest * std::sqrt(uni(rng)), 2 * M_PI * uni(rng));
        Eigen::Vector2f uv;
        Eigen::Vector3f back;
        if (!cam.project(f, uv) || !cam.unproject(uv, back)) { ++bad; continue; }
        maxRad = std::max(maxRad, angle(f, back));
    }
    snprintf(msg, sizeof msg, "bearing -> pixel -> bearing, max %.2g rad", maxRad);
    check(bad == 0 && maxRad < 1e-6, msg);
    Eigen::Vector2f c;
    check(cam.project(Eigen::Vector3f(0, 0, 3), c) && (c - Eigen::Vector2f(cam.cx(), cam.cy())).norm() < 1e-4f,
          "optical axis -> (cx, cy)");
    check(!cam.project(Eigen::Vector3f::Zero(), c), "zero vector rejected");

    // 2. ORB-SLAM3 (rays below 85°: its unproject returns z = 1 points)
    double maxOrbPx = 0, maxOrbRad = 0;
    for (int i = 0; i < 20000; ++i) {
        const double theta = std::min(tc.thetaTest, 85 * M_PI / 180) * std::sqrt(uni(rng));
        const Eigen::Vector3f f = bearingAt(theta, 2 * M_PI * uni(rng)) * float(0.5 + 5 * uni(rng));
        Eigen::Vector2f uv;
        cam.project(f, uv);
        maxOrbPx = std::max(maxOrbPx, double((orbProject(tc.P, f) - uv).norm()));
        Eigen::Vector3f b;
        cam.unproject(uv, b);
        maxOrbRad = std::max(maxOrbRad, angle(orbUnproject(tc.P, uv), b));
    }
    snprintf(msg, sizeof msg, "project == ORB-SLAM3, max %.2g px", maxOrbPx);
    check(maxOrbPx < 2e-3, msg);
    snprintf(msg, sizeof msg, "unproject == ORB-SLAM3, max %.2g rad", maxOrbRad);
    check(maxOrbRad < 2e-5, msg);

    // 3. Jacobian, central differences (points 0.5-3 m away)
    double maxJ = 0;
    for (int i = 0; i < 2000; ++i) {
        const Eigen::Vector3f p = bearingAt(tc.thetaTest * std::sqrt(uni(rng)), 2 * M_PI * uni(rng)) *
                                  float(0.5 + 2.5 * uni(rng));
        const Eigen::Matrix<float, 2, 3> J = cam.project_jacobian(p);
        Eigen::Matrix<float, 2, 3> Jn;
        const float h = 1e-3f * p.norm();
        for (int k = 0; k < 3; ++k) {
            Eigen::Vector3f a = p, b = p;
            a[k] += h;
            b[k] -= h;
            Eigen::Vector2f ua, ub;
            cam.project(a, ua);
            cam.project(b, ub);
            Jn.col(k) = (ua - ub) / (2 * h);
        }
        maxJ = std::max(maxJ, double((J - Jn).norm() / J.norm()));
    }
    snprintf(msg, sizeof msg, "project_jacobian vs finite differences, max rel %.2g", maxJ);
    check(maxJ < 1e-3, msg);
    const Eigen::Matrix<float, 2, 3> J0 = cam.project_jacobian(Eigen::Vector3f(0, 0, 2));
    check(std::abs(J0(0, 0) - cam.fx() / 2) < 1e-3f && std::abs(J0(1, 1) - cam.fy() / 2) < 1e-3f,
          "project_jacobian on the optical axis (pinhole limit)");

    // 4. pixels_per_radian: radial and tangential pixel motion for a 1 mrad bearing rotation
    bool pprOk = std::abs(cam.pixels_per_radian(0) - std::min(cam.fx(), cam.fy())) < 1e-3f;
    const double fRatio = std::min(cam.fx(), cam.fy()) / std::max(cam.fx(), cam.fy());
    for (double deg : {5.0, 30.0, 60.0, tc.thetaTest * 180 / M_PI})
        for (double psi : {0.0, 0.7, 1.6, 2.5, 4.0}) {
            const double t = deg * M_PI / 180, d = 1e-3;
            Eigen::Vector2f r0, r1, t0, t1;
            cam.project(bearingAt(t - d, psi), r0);
            cam.project(bearingAt(t + d, psi), r1);
            cam.project(bearingAt(t, psi - d / std::sin(t)), t0);
            cam.project(bearingAt(t, psi + d / std::sin(t)), t1);
            const double measured = std::min((r1 - r0).norm(), (t1 - t0).norm()) / (2 * d);
            const double ppr = cam.pixels_per_radian(float(t));
            // ppr uses min(fx, fy): at most the measured motion, at least measured · fmin / fmax
            pprOk &= ppr <= measured * 1.001 && ppr >= measured * fRatio * 0.999;
        }
    check(pprOk, "pixels_per_radian vs measured pixel motion");
}

int main() {
    // TUM-VI cam0 (512x512 fisheye, ORB-SLAM3 TUM-VI.yaml)
    const float tum[8] = {190.97847715128717f, 190.9733070521226f, 254.93170605935475f, 256.8974428996504f,
                          0.0034823894022493434f, 0.0007150348452162257f, -0.0020532361418706202f,
                          0.00020293673591811182f};
    // A wide lens on the 1440x1080 sensor (made-up parameters, ~82° at the corners)
    const float wide[8] = {640.f, 641.5f, 719.5f, 539.5f, -0.012f, 0.004f, -0.0008f, 0.00005f};
    testCamera(TestCamera("TUM-VI cam0", tum, 512, 512, 85));
    testCamera(TestCamera("wide 1440x1080", wide, 1440, 1080, 80));

    printf("valid range and arguments\n");
    {
        // r(θ) = θ − 0.3 θ³ turns back at r'(θ) = 1 − 0.9 θ² = 0, θ = 1.0541
        const camera::KannalaBrandt8 bent(300, 300, 320, 240, -0.3f, 0, 0, 0, 640, 480);
        check(std::abs(bent.max_theta() - 1.0541f) < 2e-3f, "max_theta where r(θ) turns back");
        Eigen::Vector2f uv;
        Eigen::Vector3f f;
        check(bent.project(bearingAt(1.0, 0.3), uv), "ray inside the valid range projects");
        check(!bent.project(bearingAt(1.2, 0.3), uv), "ray beyond max_theta rejected");
        check(!bent.unproject(Eigen::Vector2f(320 + 300 * 0.71f, 240), f), "pixel beyond r(max_theta) rejected");
        check(bent.unproject(Eigen::Vector2f(320 + 300 * 0.69f, 240), f), "pixel inside r(max_theta) unprojects");
        check(throws<std::invalid_argument>([] { camera::KannalaBrandt8(0, 300, 320, 240, 0, 0, 0, 0, 640, 480); }),
              "fx = 0 rejected");
        check(throws<std::invalid_argument>([] { camera::KannalaBrandt8(300, 300, 320, 240, 0, 0, 0, 0, 0, 480); }),
              "width = 0 rejected");
    }

    printf("resized\n");
    {
        const camera::KannalaBrandt8 full(640.f, 641.5f, 703.2f, 551.7f, -0.012f, 0.004f, -0.0008f, 0.00005f, 1440, 1080);
        const camera::KannalaBrandt8 half = full.resized(720, 540);
        check(half.width() == 720 && half.height() == 540, "size");
        check(std::abs(half.fx() - 320.f) < 1e-4f && std::abs(half.fy() - 320.75f) < 1e-4f, "fx, fy halved");
        check(std::abs(half.cx() - ((703.2f + 0.5f) / 2 - 0.5f)) < 1e-4f &&
              std::abs(half.cy() - ((551.7f + 0.5f) / 2 - 0.5f)) < 1e-4f, "cx, cy = (c + 0.5) / 2 − 0.5");
        check(half.k(0) == full.k(0) && half.k(3) == full.k(3), "k1..k4 unchanged");
        std::mt19937 rng(2);
        std::uniform_real_distribution<double> uni(0, 1);
        double maxPx = 0;
        for (int i = 0; i < 5000; ++i) {
            const Eigen::Vector3f f = bearingAt(1.3 * std::sqrt(uni(rng)), 2 * M_PI * uni(rng));
            Eigen::Vector2f a, b;
            full.project(f, a);
            half.project(f, b);
            maxPx = std::max(maxPx, double((b - ((a.array() + 0.5f) / 2 - 0.5f).matrix()).norm()));
        }
        char msg[128];
        snprintf(msg, sizeof msg, "half-res pixel = (full-res pixel + 0.5) / 2 − 0.5, max %.2g px", maxPx);
        check(maxPx < 1e-3, msg);
        const camera::KannalaBrandt8 back = half.resized(1440, 1080);
        check(std::abs(back.cx() - full.cx()) < 1e-3f && std::abs(back.fx() - full.fx()) < 1e-3f,
              "resized back to full size");
        check(throws<std::invalid_argument>([&] { full.resized(720, 600); }), "aspect ratio change rejected");
    }

    printf("camera::StereoRig\n");
    {
        const Eigen::Matrix3f R = Eigen::AngleAxisf(0.01f, Eigen::Vector3f(0.3f, 1.f, -0.2f).normalized()).matrix();
        const Eigen::Vector3f t(0.05f, 0.0002f, -0.0003f);
        const camera::KannalaBrandt8 l(640.f, 641.5f, 719.5f, 539.5f, -0.012f, 0.004f, -0.0008f, 0.00005f, 1440, 1080);
        const camera::KannalaBrandt8 r(645.f, 644.f, 721.f, 541.f, -0.011f, 0.0035f, -0.0007f, 0.00004f, 1440, 1080);
        const camera::StereoRig rig(l, r, R, t);
        check(rig.width() == 1440 && rig.height() == 1080, "image size");
        check((rig.R_12() - R).norm() < 1e-6f && (rig.t_12() - t).norm() < 1e-7f, "R_12, t_12");
        check(std::abs(rig.baseline() - t.norm()) < 1e-7f, "baseline");
        const camera::StereoRig half = rig.resized(720, 540);
        check(half.width() == 720 && std::abs(half.camera(1).fx() - 322.5f) < 1e-4f &&
              (half.t_12() - t).norm() == 0, "resized rig");
        check(throws<std::invalid_argument>([&] { camera::StereoRig(l, r, 1.1f * R, t); }), "non-rotation rejected");
        check(throws<std::invalid_argument>([&] { camera::StereoRig(l, r, R, Eigen::Vector3f::Zero()); }), "zero baseline rejected");
    }

    printf("camera::stereoRigFromCalib (calib::StereoImuRig)\n");
    {
        // EuRoC MAV cam0 / cam1 (sensor.yaml), as euroc_mcap_driver sends them
        auto euroc = [] {
            calib::StereoImuRig c;
            c.left = {"cam0", 752, 480, 458.654, 457.296, 367.215, 248.375, "plumb_bob",
                      {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0}};
            c.right = {"cam1", 752, 480, 457.587, 456.134, 379.999, 255.238, "plumb_bob",
                       {-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05, 0.0}};
            Eigen::Matrix4d T0, T1;
            T0 << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                  0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                  -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949, 0, 0, 0, 1;
            T1 << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
                  0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
                  -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038, 0, 0, 0, 1;
            c.T_body_left.matrix() = T0;
            c.T_body_right.matrix() = T1;
            return c;
        };
        camera::ModelFit f0, f1;
        const camera::StereoRig rig = camera::stereoRigFromCalib(euroc(), &f0, &f1);
        char msg[160];
        snprintf(msg, sizeof msg, "KB8 fit to radtan: rms %.3f / %.3f px, p99 %.3f / %.3f px", f0.rms_px, f1.rms_px,
                 f0.p99_px, f1.p99_px);
        check(f0.fitted && f1.fitted && f0.rms_px < 0.2 && f1.rms_px < 0.2 && f0.p99_px < 0.5 && f1.p99_px < 0.5, msg);
        // the KB8 of the reference's EuRoC_KB8.yaml (same fit)
        check(std::abs(rig.camera(0).k(0) - 0.0280537792f) < 1e-4f && std::abs(rig.camera(0).k(3) - 0.565113425f) < 1e-3f &&
              std::abs(rig.camera(1).k(1) - 0.140624762f) < 1e-4f, "k1..k4 as EuRoC_KB8.yaml");
        Eigen::Matrix3f Rref;
        Rref << 0.999997258f, -0.00231713569f, -0.000343393127f, 0.0023120672f, 0.999898076f, -0.0140906684f,
            0.000376008102f, 0.0140898358f, 0.999900639f;
        const Eigen::Vector3f tref(0.11007414f, -0.000156612048f, 0.000889382791f);
        check((rig.R_12() - Rref).norm() < 1e-5f && (rig.t_12() - tref).norm() < 1e-6f,
              "T_c1_c2 = T_body_left^-1 T_body_right (as EuRoC_KB8.yaml)");

        calib::StereoImuRig kb = euroc();
        kb.left.distortion_model = kb.right.distortion_model = "equidistant";
        kb.left.d = {0.01, 0.02, 0.03, 0.04};
        kb.right.d = {0.01, 0.02, 0.03, 0.04};
        camera::ModelFit fk;
        const camera::StereoRig kbr = camera::stereoRigFromCalib(kb, &fk);
        check(!fk.fitted && kbr.camera(0).k(2) == 0.03f && kbr.camera(1).fx() == 457.587f, "equidistant taken as is");

        calib::StereoImuRig bad = euroc();
        bad.right.distortion_model = "rational_polynomial";
        check(throws<std::invalid_argument>([&] { camera::stereoRigFromCalib(bad); }), "unknown distortion model rejected");
        bad = euroc();
        bad.left.d.resize(3);
        check(throws<std::invalid_argument>([&] { camera::stereoRigFromCalib(bad); }), "missing coefficients rejected");
        bad = euroc();
        bad.right.width = 640;
        check(throws<std::invalid_argument>([&] { camera::stereoRigFromCalib(bad); }), "different image sizes rejected");
        bad = euroc();
        bad.T_body_right = bad.T_body_left;
        check(throws<std::invalid_argument>([&] { camera::stereoRigFromCalib(bad); }), "zero baseline rejected");
        calib::StereoImuRig pin = euroc();
        pin.left.distortion_model = pin.right.distortion_model = "none";
        pin.left.d.clear();
        pin.right.d.clear();
        camera::ModelFit fp;
        camera::stereoRigFromCalib(pin, &fp);
        snprintf(msg, sizeof msg, "no distortion: KB8 fit to the pinhole, p99 %.3f px", fp.p99_px);
        check(fp.fitted && fp.p99_px < 0.5, msg);
    }

    printf(failures ? "%d FAILED\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
