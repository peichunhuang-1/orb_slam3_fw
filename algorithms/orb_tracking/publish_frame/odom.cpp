#include "algorithms/orb_tracking/publish_frame/odom.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "algorithms/orb_tracking/shared/pose_optimizer.hpp"

namespace orb_tracking {

namespace {

Eigen::Isometry3d iso(const geometry::SE3f& T) {
    Eigen::Isometry3d r = Eigen::Isometry3d::Identity();
    r.linear() = T.rotationMatrix().cast<double>();
    r.translation() = T.translation().cast<double>();
    return r;
}

bool valid(const Eigen::Vector3f& p) { return !std::isnan(p.x()); }

// D from the anchor keyframe's motion, if it is still there
bool anchorMotion(const tracking::OdomState& s, const map::Atlas& a, Eigen::Isometry3d& D) {
    const map::KeyFrame* kf = a.goodKeyFrame(s.anchor);
    if (!kf) return false;
    D = iso(kf->poseInverse()) * iso(s.anchor_Twc).inverse();
    return true;
}

void remember(tracking::OdomState& s, const tracking::Frame& f, const map::Atlas& a) {
    s.started = true;
    s.map = a.current;
    s.last = f;
    // Only what PoseOptimization reads is needed; drop the shared IMU state
    s.last.preintegrated.reset();
    s.last.preintegrated_frame.reset();
    s.last.prior.reset();
    const Eigen::Vector3f none = Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
    s.last_world.assign(f.points.size(), none);
    for (std::size_t i = 0; i < f.points.size(); ++i) {
        if (f.outlier[i]) continue;
        if (const map::MapPoint* p = a.goodPoint(f.points[i])) s.last_world[i] = p->pos;
    }
    s.anchor = map::kNone;
    if (const map::KeyFrame* kf = a.goodKeyFrame(f.reference_kf)) {
        s.anchor = kf->id;
        s.anchor_Twc = kf->poseInverse();
    }
}

}  // namespace

OdomUpdate updateOdom(tracking::OdomState& s, const tracking::Frame& f, const map::Atlas& a) {
    OdomUpdate u;
    const Eigen::Isometry3d Twc = iso(f.pose().inverse());
    if (!s.started) {
        s.T_map_odom.setIdentity();
        u.source = OdomSource::first;
    } else if (a.current != s.map) {
        const Eigen::Isometry3d T_odom_last = s.T_map_odom.inverse() * iso(s.last.pose().inverse());
        s.T_map_odom = Twc * T_odom_last.inverse();
        u.source = OdomSource::new_map;
    } else {
        // The last frame's inlier points: then and now; did any move?
        const std::size_t n = s.last.points.size();
        std::vector<const Eigen::Vector3f*> then(n, nullptr), now(n, nullptr);
        bool moved = false;
        for (std::size_t i = 0; i < n; ++i) {
            if (!valid(s.last_world[i])) continue;
            const map::MapPoint* p = a.goodPoint(s.last.points[i]);
            if (!p) continue;
            then[i] = &s.last_world[i];
            now[i] = &p->pos;
            moved = moved || p->pos != s.last_world[i];
        }
        Eigen::Isometry3d D = Eigen::Isometry3d::Identity();
        u.source = OdomSource::unchanged;
        if (moved) {
            Eigen::Isometry3d D0 = Eigen::Isometry3d::Identity();
            const bool has_anchor = anchorMotion(s, a, D0);
            const camera::StereoRig& rig = a.rig(s.last.rig);
            tracking::Frame f_then = s.last, f_now = s.last;
            const int n_then = optimizePose(f_then, rig, then);
            // Start from the last frame moved with the anchor: Twc' = D0 · Twc, Tcw' = Tcw · D0⁻¹
            const Eigen::Isometry3d Tcw_start = iso(s.last.pose()) * D0.inverse();
            f_now.setPose(geometry::SE3f(Eigen::Quaternionf(Tcw_start.rotation().cast<float>()),
                                         Tcw_start.translation().cast<float>()));
            const int n_now = optimizePose(f_now, rig, now);
            if (n_then >= kOdomMinInliers && n_now >= kOdomMinInliers) {
                D = iso(f_now.pose().inverse()) * iso(f_then.pose().inverse()).inverse();
                u.source = OdomSource::relocalized;
                u.inliers = n_now;
            } else if (has_anchor) {
                D = D0;
                u.source = OdomSource::anchor;
            } else {
                u.source = OdomSource::none;
            }
        }
        s.T_map_odom = D * s.T_map_odom;
        s.T_map_odom.linear() = Eigen::Quaterniond(s.T_map_odom.rotation()).normalized().toRotationMatrix();
    }
    remember(s, f, a);
    u.T_map_odom = s.T_map_odom;
    u.T_odom_cam = s.T_map_odom.inverse() * Twc;
    return u;
}

}  // namespace orb_tracking
