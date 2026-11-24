// publish_frame: the end of Tracking::Track(). OK / RECENTLY_LOST: the frame's pose relative to its reference
// keyframe goes to the trajectory (mlRelativeFramePoses, ...; without a pose the last entry again), and the body
// pose is sent in the continuous odom frame with map_to_odom (odom.hpp; not in ORB-SLAM3). The status is sent
// for every frame.
#include <Eigen/Geometry>

#include "algorithms/orb_tracking/shared/odom.hpp"
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "fw/ros/msgs/geometry.hpp"
#include "types/tracking/status.hpp"

namespace {

using tracking::State;

fw::bt::Status publish_frame(const State& state, fw::Write<tracking::Tracker> tracker,
                             fw::Optional<fw::PoseStamped>& pose, fw::Optional<fw::TransformStamped>& map_to_odom,
                             tracking::Status& status) {
    tracking::Tracker& t = *tracker;
    const map::Atlas& a = tracker->view;
    const tracking::Frame& f = t.current;
    const bool tracked = state == State::OK || state == State::RECENTLY_LOST;

    if (tracked) {
        const map::KeyFrame* ref = a.keyframe(f.reference_kf);
        if (f.hasPose() && ref) {
            t.relative_poses.push_back(f.pose() * ref->poseInverse());
            t.references.push_back(f.reference_kf);
            t.frame_times.push_back(f.timestamp);
            t.lost.push_back(state == State::LOST);
        } else if (!t.relative_poses.empty()) {  // "This can happen if tracking is lost"
            t.relative_poses.push_back(t.relative_poses.back());
            t.references.push_back(t.references.back());
            t.frame_times.push_back(t.frame_times.back());
            t.lost.push_back(state == State::LOST);
        }
        if (f.hasPose()) {
            const orb_tracking::OdomUpdate u = orb_tracking::updateOdom(t.odom, f, a);
            // The body in odom: T_odom_cam · T_body_left⁻¹
            const Eigen::Isometry3d T = u.T_odom_cam * t.calib.T_body_left.inverse();
            const Eigen::Quaterniond q(T.rotation());
            fw::PoseStamped p;
            p.frame_id = "odom";
            p.pose.position.x = T.translation().x();
            p.pose.position.y = T.translation().y();
            p.pose.position.z = T.translation().z();
            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            p.pose.orientation.w = q.w();
            pose = p;
            const Eigen::Quaterniond qm(u.T_map_odom.rotation());
            fw::TransformStamped m;
            m.frame_id = "map";
            m.child_frame_id = "odom";
            m.transform.translation.x = u.T_map_odom.translation().x();
            m.transform.translation.y = u.T_map_odom.translation().y();
            m.transform.translation.z = u.T_map_odom.translation().z();
            m.transform.rotation.x = qm.x();
            m.transform.rotation.y = qm.y();
            m.transform.rotation.z = qm.z();
            m.transform.rotation.w = qm.w();
            map_to_odom = m;
        }
    }

    status = tracking::Status();
    status.state = state;
    status.frame_id = f.id;
    status.inliers = t.matches_inliers;
    status.map_id = a.current;
    status.maps = int(a.maps.size());
    if (const map::Map* m = a.findMap(a.current)) {
        status.keyframes = m->keyframes.size();
        status.points = m->n_points;
    }
    status.new_keyframe = t.new_keyframe;
    status.relocalized = f.valid() && t.last_reloc_frame_id == f.id && tracked;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(publish_frame, INPUTS(("state", tracking::State, latest)),
                      RESOURCES(("tracker", tracking::Tracker, write)),
                      OUTPUTS(("pose", fw::PoseStamped), ("map_to_odom", fw::TransformStamped), ("tracking", tracking::Status)));
