// publish_frame: the end of Tracking::Track(). OK / RECENTLY_LOST: the frame's pose relative to its reference
// keyframe goes to the trajectory (mlRelativeFramePoses, ...; without a pose the last entry again), and the body
// pose is sent. The status is sent for every frame.
#include <Eigen/Geometry>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "fw/ros/msgs/geometry.hpp"
#include "types/tracking/status.hpp"

namespace {

using tracking::State;

fw::bt::Status publish_frame(const State& state, fw::Read<map::Atlas> atlas, fw::Write<tracking::Tracker> tracker,
                             fw::Optional<fw::PoseStamped>& pose, tracking::Status& status) {
    tracking::Tracker& t = *tracker;
    const map::Atlas& a = *atlas;
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
            // The body in the world: Twc · T_body_left⁻¹
            const geometry::SE3f Twc = f.pose().inverse();
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.linear() = Twc.rotationMatrix().cast<double>();
            T.translation() = Twc.translation().cast<double>();
            T = T * t.calib.T_body_left.inverse();
            const Eigen::Quaterniond q(T.rotation());
            fw::PoseStamped p;
            p.frame_id = "world";
            p.pose.position.x = T.translation().x();
            p.pose.position.y = T.translation().y();
            p.pose.position.z = T.translation().z();
            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            p.pose.orientation.w = q.w();
            pose = p;
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
        status.points = m->points.size();
    }
    status.new_keyframe = t.new_keyframe;
    status.relocalized = f.valid() && t.last_reloc_frame_id == f.id && tracked;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(publish_frame, INPUTS(("state", tracking::State, latest)),
                      RESOURCES(("atlas", map::Atlas, read), ("tracker", tracking::Tracker, write)),
                      OUTPUTS(("pose", fw::PoseStamped), ("tracking", tracking::Status)));
