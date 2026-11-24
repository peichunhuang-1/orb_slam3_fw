// orb_tracking/shared/odom: the continuous odom frame (not in ORB-SLAM3, so no fixture of its answers).
// A frame of the pose_optimization fixture is published, then the map changes and the same frame (no motion) is
// tracked again in the new map; odom must not move:
//   - nothing changed: D = I
//   - a rigid change of the whole map (a 100° rotation and a translation, as IMU initialization): D = G
//   - a non-rigid change (the points near the frame and the reference keyframe move differently, as a BA):
//     re-localizing the last frame keeps odom still, where the reference keyframe's motion alone would not
//   - too few points left: the reference keyframe's motion
//   - another map: odom goes on from the last pose by the motion the IMU measured since (also over frames that
//     were not published), the rotation alone when the last map's IMU was not initialized, none without IMU
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>

#include "algorithms/orb_tracking/shared/odom.hpp"
#include "algorithms/orb_tracking/shared/pose_optimizer.hpp"
#include "tests/orb_tracking/fixture_frame.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

using Warp = std::function<Eigen::Vector3f(const Eigen::Vector3f&)>;

// The distance between two poses: translation (m) and rotation (rad)
void distance(const Eigen::Isometry3d& A, const Eigen::Isometry3d& B, double& t, double& r) {
    const Eigen::Isometry3d d = A.inverse() * B;
    t = d.translation().norm();
    r = Eigen::AngleAxisd(d.rotation()).angle();
}

geometry::SE3f se3(const Eigen::Isometry3d& T) {
    return geometry::SE3f(Eigen::Quaternionf(T.rotation().cast<float>()), T.translation().cast<float>());
}

Eigen::Isometry3d iso(const geometry::SE3f& T) {
    Eigen::Isometry3d r = Eigen::Isometry3d::Identity();
    r.linear() = T.rotationMatrix().cast<double>();
    r.translation() = T.translation().cast<double>();
    return r;
}

struct Scene {
    fixture::Rig rig;
    fixture::Observations obs;
    map::Atlas atlas;
    tracking::OdomState odom;

    explicit Scene(const char* name) : obs(fixture::frameOf(fixture::File(std::string("pose_optimization/") + name + ".txt"), rig)) {
        const fixture::File f(std::string("pose_optimization/") + name + ".txt");
        obs.frame.setPose(fixture::se3f(f["Tcw_init"]));
        orb_tracking::optimizePose(obs.frame, rig.rig, obs.world);  // a tracked frame: its pose and outliers
        atlas.rigs.edit(0).set(rig.rig);
        atlas.n_rigs = 1;
        setMap(0);
        for (std::size_t m = 0; m < obs.mp_world.size(); ++m) {
            map::MapPoint p;
            p.id = m;
            p.map = 0;
            p.pos = obs.mp_world[m];
            atlas.points.edit(m).set(p);
        }
        // The reference keyframe: 1 m behind the frame
        map::KeyFrame kf;
        kf.id = 0;
        kf.map = 0;
        kf.setPose(geometry::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0, 0, 1)) * obs.frame.pose());
        atlas.keyframes.edit(0).set(kf);
        obs.frame.reference_kf = 0;
    }
    void setMap(map::MapId id, bool imu_initialized = false) {
        map::Map m;
        m.id = id;
        m.imu_initialized = imu_initialized;
        atlas.maps.edit(id).set(m);
        atlas.current = id;
    }
    // Every point through w, the keyframe through G
    void change(const Warp& w, const Eigen::Isometry3d& G_kf) {
        for (std::size_t m = 0; m < obs.mp_world.size(); ++m) {
            map::MapPoint* p = atlas.editPoint(m);
            p->pos = w(p->pos);
        }
        map::KeyFrame* kf = atlas.editKeyFrame(0);
        kf->setPose(se3((G_kf * iso(kf->poseInverse())).inverse()));
    }
    // The same frame tracked again in the atlas now, from a start pose
    tracking::Frame retrack(const geometry::SE3f& Tcw_start) const {
        tracking::Frame f = obs.frame;
        std::vector<const Eigen::Vector3f*> world(std::size_t(f.size()), nullptr);
        for (std::size_t i = 0; i < world.size(); ++i)
            if (const map::MapPoint* p = atlas.goodPoint(f.points[i])) world[i] = &p->pos;
        f.setPose(Tcw_start);
        orb_tracking::optimizePose(f, rig.rig, world);
        return f;
    }
};

const char* sourceName(orb_tracking::OdomSource s) {
    switch (s) {
        case orb_tracking::OdomSource::first: return "first";
        case orb_tracking::OdomSource::unchanged: return "unchanged";
        case orb_tracking::OdomSource::relocalized: return "relocalized";
        case orb_tracking::OdomSource::anchor: return "anchor";
        case orb_tracking::OdomSource::none: return "none";
        case orb_tracking::OdomSource::new_map: return "new_map";
        case orb_tracking::OdomSource::new_map_imu: return "new_map_imu";
        case orb_tracking::OdomSource::new_map_gyro: return "new_map_gyro";
    }
    return "?";
}

void expectStill(const std::string& name, const orb_tracking::OdomUpdate& before, const orb_tracking::OdomUpdate& after,
                 orb_tracking::OdomSource source, double max_t, double max_r) {
    double t, r;
    distance(before.T_odom_cam, after.T_odom_cam, t, r);
    std::printf("%-26s %-11s inliers %3d  odom moved %.2e m %.2e rad\n", name.c_str(), sourceName(after.source),
                after.inliers, t, r);
    check(after.source == source, name + ": source " + sourceName(after.source));
    check(t < max_t && r < max_r, name + ": odom did not stay");
}

// A rigid change of the whole map, as IMU initialization: 100° about a tilted axis and a translation
Eigen::Isometry3d bigRigid() {
    Eigen::Isometry3d G = Eigen::Isometry3d::Identity();
    G.linear() = Eigen::AngleAxisd(100.0 * M_PI / 180.0, Eigen::Vector3d(0.3, -1.0, 0.2).normalized()).toRotationMatrix();
    G.translation() = Eigen::Vector3d(0.4, 2.1, 0.9);
    return G;
}

void unchangedAndRigid() {
    Scene s("nominal");
    const orb_tracking::OdomUpdate u0 = orb_tracking::updateOdom(s.odom, s.obs.frame, s.atlas);
    check(u0.source == orb_tracking::OdomSource::first, "first: source");
    check(u0.T_map_odom.isApprox(Eigen::Isometry3d::Identity()), "first: odom = map");

    const orb_tracking::OdomUpdate u1 = orb_tracking::updateOdom(s.odom, s.obs.frame, s.atlas);
    expectStill("unchanged", u0, u1, orb_tracking::OdomSource::unchanged, 1e-9, 1e-9);

    const Eigen::Isometry3d G = bigRigid();
    s.change([&](const Eigen::Vector3f& p) -> Eigen::Vector3f { return (G * p.cast<double>()).cast<float>(); }, G);
    // tracking in the new map (its start: the motion model, already in the new map)
    const tracking::Frame f = s.retrack(se3(iso(s.obs.frame.pose()) * G.inverse()));
    const orb_tracking::OdomUpdate u2 = orb_tracking::updateOdom(s.odom, f, s.atlas);
    expectStill("rigid 100°", u1, u2, orb_tracking::OdomSource::relocalized, 1e-4, 1e-5);
    double t, r;
    distance(u2.T_map_odom, G, t, r);
    check(t < 1e-4 && r < 1e-5, "rigid: map_to_odom = G");
}

void nonRigid() {
    Scene s("nominal");
    const orb_tracking::OdomUpdate u0 = orb_tracking::updateOdom(s.odom, s.obs.frame, s.atlas);
    // The points: a smooth deformation (a few cm, varying with position); the keyframe: 10 cm another way
    const Warp w = [](const Eigen::Vector3f& p) -> Eigen::Vector3f {
        return Eigen::Vector3f(p + Eigen::Vector3f(0.03f + 0.01f * std::sin(p.y()), -0.02f + 0.01f * p.x() / (1 + std::abs(p.x())),
                                                   0.04f + 0.005f * std::cos(p.z())));
    };
    Eigen::Isometry3d G_kf = Eigen::Isometry3d::Identity();
    G_kf.translation() = Eigen::Vector3d(-0.06, 0.08, 0.0);
    G_kf.linear() = Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY()).toRotationMatrix();
    s.change(w, G_kf);
    const tracking::Frame f = s.retrack(s.obs.frame.pose());
    const orb_tracking::OdomUpdate u1 = orb_tracking::updateOdom(s.odom, f, s.atlas);
    // Not exact: odom re-localizes with the last frame's inliers, tracking used every point of the frame; under a
    // non-rigid change two point sets see slightly different rigid motions (as the next frame's do)
    expectStill("non-rigid", u0, u1, orb_tracking::OdomSource::relocalized, 2e-3, 1e-3);
    // What the reference keyframe's motion alone would have given
    const Eigen::Isometry3d by_anchor = (G_kf * u0.T_map_odom).inverse() * iso(f.pose().inverse());
    double t, r;
    distance(u0.T_odom_cam, by_anchor, t, r);
    std::printf("%-26s %-11s              odom moved %.2e m %.2e rad\n", "non-rigid, anchor only", "(reference)", t, r);
    check(t > 1e-2, "non-rigid: the anchor alone would move odom (the test needs it)");
}

void fewPoints() {
    Scene s("nominal");
    const orb_tracking::OdomUpdate u0 = orb_tracking::updateOdom(s.odom, s.obs.frame, s.atlas);
    Eigen::Isometry3d G = Eigen::Isometry3d::Identity();
    G.translation() = Eigen::Vector3d(0.05, 0, 0);
    s.change([&](const Eigen::Vector3f& p) -> Eigen::Vector3f { return (G * p.cast<double>()).cast<float>(); }, G);
    // all but 10 points go bad
    int kept = 0;
    for (std::size_t i = 0; i < s.obs.frame.points.size(); ++i) {
        const map::MapPointId m = s.obs.frame.points[i];
        if (m == map::kNone) continue;
        if (!s.obs.frame.outlier[i] && kept < 10) {
            ++kept;
            continue;
        }
        if (map::MapPoint* p = s.atlas.editPoint(m)) p->bad = true;
    }
    for (std::size_t m = 0; m < s.obs.mp_world.size(); ++m) {
        bool seen = false;
        for (std::size_t i = 0; i < s.obs.frame.points.size(); ++i) seen = seen || s.obs.frame.points[i] == m;
        if (!seen) s.atlas.editPoint(m)->bad = true;
    }
    tracking::Frame f = s.obs.frame;  // tracked by other means (the IMU): moved with the map
    f.setPose(se3(iso(f.pose()) * G.inverse()));
    const orb_tracking::OdomUpdate u1 = orb_tracking::updateOdom(s.odom, f, s.atlas);
    expectStill("few points", u0, u1, orb_tracking::OdomSource::anchor, 1e-5, 1e-6);
}

void newMap() {
    Scene s("nominal");
    const orb_tracking::OdomUpdate u0 = orb_tracking::updateOdom(s.odom, s.obs.frame, s.atlas);
    s.setMap(1);
    tracking::Frame f = s.obs.frame;  // the new map's first frame: at its origin
    f.setPose(geometry::SE3f());
    const orb_tracking::OdomUpdate u1 = orb_tracking::updateOdom(s.odom, f, s.atlas);
    expectStill("new map", u0, u1, orb_tracking::OdomSource::new_map, 1e-6, 1e-6);
}

// The IMU across a new map. The body: constant angular velocity w (body), constant world acceleration acc_w, from
// the last published frame's state (its pose, a velocity, a bias); the measurements carry that bias. Two frames
// of 1 s each follow: the first is not published (a discontinuity), the second is the new map's first.
struct Motion {
    Eigen::Matrix3d Rwb1;
    Eigen::Vector3d twb1, v1, w, acc_w;
    imu::Bias b;
    Eigen::Matrix3d R(double t) const { if (t == 0) return Rwb1; return Rwb1 * Eigen::AngleAxisd((w * t).norm(), (w * t).normalized()).toRotationMatrix(); }
    Eigen::Vector3d p(double t) const { return twb1 + v1 * t + 0.5 * acc_w * t * t; }
    // The preintegration (zero bias, as imu_preintegration sends it) over [t0, t1] of the last frame's time
    std::shared_ptr<const imu::Preintegrated> measure(double t0, double t1) const {
        const Eigen::Vector3d g(0, 0, -9.81);
        auto p = std::make_shared<imu::Preintegrated>(imu::Bias(), imu::Noise::fromDensities(1.7e-4f, 2e-3f, 2e-5f, 3e-3f));
        const int n = int(std::lround((t1 - t0) * 200));
        const double dt = (t1 - t0) / n;
        for (int k = 0; k < n; ++k) {
            const double tm = t0 + k * dt;  // the integrator rotates a step's sample by ΔR at its start
            const Eigen::Vector3f acc = (R(tm).transpose() * (acc_w - g)).cast<float>() + b.a;
            const Eigen::Vector3f gyro = w.cast<float>() + b.g;
            p->integrate(acc, gyro, float(dt));
        }
        return p;
    }
};

tracking::Frame withImu(const tracking::Frame& f, double timestamp, std::shared_ptr<const imu::Preintegrated> imu) {
    auto st = std::make_shared<orb::StereoFrame>(*f.features->stereo);
    st->imu = std::move(imu);
    tracking::Frame r = f;
    r.features = std::make_shared<orb::FrameFeatures>(st);
    r.timestamp = timestamp;
    return r;
}

void newMapImu(const char* name, bool imu_initialized, bool break_imu, orb_tracking::OdomSource source, double max_t,
               double max_r) {
    Scene s("nominal");
    s.setMap(0, imu_initialized);
    tracking::Frame& last = s.obs.frame;
    last.timestamp = 10;
    last.has_imu_calib = true;
    last.T_cb = geometry::SE3f(Eigen::Matrix3f(Eigen::AngleAxisf(0.5f, Eigen::Vector3f(1, 2, 0).normalized())),
                               Eigen::Vector3f(0.02f, -0.06f, 0.01f));
    Motion m;
    const geometry::SE3f Twb = last.imuPose();
    m.Rwb1 = Twb.rotationMatrix().cast<double>();
    m.twb1 = Twb.translation().cast<double>();
    m.v1 = Eigen::Vector3d(0.4, -0.2, 0.1);
    m.w = Eigen::Vector3d(0.1, -0.3, 0.9);  // 2 s: about 110°, as the V1_02 gap
    m.acc_w = Eigen::Vector3d(0.2, 0.1, -0.05);
    m.b.g = Eigen::Vector3f(0.003f, -0.002f, 0.004f);
    m.b.a = Eigen::Vector3f(0.05f, -0.03f, 0.02f);
    last.bias = m.b;
    last.setVelocity(m.v1.cast<float>());
    const orb_tracking::OdomUpdate u0 = orb_tracking::updateOdom(s.odom, last, s.atlas);

    const tracking::Frame skipped = withImu(last, 11, break_imu ? nullptr : m.measure(0, 1));
    orb_tracking::accumulateOdomImu(s.odom, skipped);
    tracking::Frame f = withImu(last, 12, m.measure(1, 2));
    orb_tracking::accumulateOdomImu(s.odom, f);
    s.setMap(1);
    f.setPose(geometry::SE3f());  // the new map's origin
    const orb_tracking::OdomUpdate u1 = orb_tracking::updateOdom(s.odom, f, s.atlas);

    // The truth: the camera's motion since the last frame, in the last map
    Eigen::Isometry3d Twb2 = Eigen::Isometry3d::Identity();
    Twb2.linear() = m.R(2);
    Twb2.translation() = m.p(2);
    const Eigen::Isometry3d Twc1 = iso(last.pose().inverse()), Twc2 = Twb2 * iso(last.T_cb.inverse());
    const Eigen::Isometry3d expected = u0.T_odom_cam * Twc1.inverse() * Twc2;
    double t, r;
    distance(expected, u1.T_odom_cam, t, r);
    std::printf("%-26s %-12s           odom off  %.2e m %.2e rad\n", name, sourceName(u1.source), t, r);
    check(u1.source == source, std::string(name) + ": source " + sourceName(u1.source));
    check(t < max_t && r < max_r, std::string(name) + ": odom off the true motion");
}

}  // namespace

int main() {
    unchangedAndRigid();
    nonRigid();
    fewPoints();
    newMap();
    // Exact up to the integration in float
    newMapImu("new map, IMU", true, false, orb_tracking::OdomSource::new_map_imu, 1e-3, 1e-4);
    // No translation predicted: off by what the body moved (about 1 m), the rotation still right
    newMapImu("new map, gyro", false, false, orb_tracking::OdomSource::new_map_gyro, 2.0, 1e-4);
    newMapImu("new map, IMU broken", true, true, orb_tracking::OdomSource::new_map, 10.0, 10.0);
    std::printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
