// make_frame: Tracking::GrabStereoFeatures (the Frame of a stereo message) and, when the calibration changes,
// newParameterLoader (the stereo rig in the atlas, mTlr, ThDepth, the IMU calibration). Also the settings of the
// run: inertial (System::IMU_STEREO) from params.sensor, mMaxFrames and mnFramesToResetIMU from params.fps.
#include <memory>
#include <stdexcept>

#include <Eigen/Geometry>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "params/orb_tracking_params.hpp"
#include "types/calib/stereo_imu_rig.hpp"
#include "types/camera/rig_from_calib.hpp"
#include "types/map/atlas_ops.hpp"
#include "types/orb/stereo_frame.hpp"

namespace {

bool sameCamera(const calib::PinholeCamera& a, const calib::PinholeCamera& b) {
    return a.width == b.width && a.height == b.height && a.fx == b.fx && a.fy == b.fy && a.cx == b.cx &&
           a.cy == b.cy && a.distortion_model == b.distortion_model && a.d == b.d;
}

bool sameCalib(const calib::StereoImuRig& a, const calib::StereoImuRig& b) {
    return sameCamera(a.left, b.left) && sameCamera(a.right, b.right) &&
           a.T_body_left.matrix() == b.T_body_left.matrix() && a.T_body_right.matrix() == b.T_body_right.matrix() &&
           a.T_body_imu.matrix() == b.T_body_imu.matrix() && a.imu.rate_hz == b.imu.rate_hz &&
           a.imu.gyro_noise == b.imu.gyro_noise && a.imu.gyro_walk == b.imu.gyro_walk &&
           a.imu.accel_noise == b.imu.accel_noise && a.imu.accel_walk == b.imu.accel_walk;
}

// Converter::toSophus of a float matrix as the settings hold it: the quaternion of the float rotation
geometry::SE3f toSE3f(const Eigen::Isometry3d& T) {
    const Eigen::Matrix4f m = T.matrix().cast<float>();
    const Eigen::Quaternionf q(Eigen::Matrix3f(m.topLeftCorner<3, 3>()));
    return geometry::SE3f(q, Eigen::Vector3f(m.block<3, 1>(0, 3)));
}

// newParameterLoader
void loadCalib(map::Atlas& a, tracking::Tracker& t, const calib::StereoImuRig& c) {
    t.rig = map::addRig(a, camera::stereoRigFromCalib(c));
    t.Tlr = toSE3f(c.T_body_left.inverse() * c.T_body_right);  // T_c1_c2
    t.baseline = t.Tlr.translation().norm();
    t.T_cb = toSE3f(c.T_imu_left()).inverse();  // IMU.T_b_c1 -> Tcb
    t.imu_noise = imu::Noise::fromContinuous(c.imu);
    if (!t.preintegrated_from_kf) t.preintegrated_from_kf = std::make_shared<imu::Preintegrated>(imu::Bias(), t.imu_noise);
    t.calib = c;
    t.has_calib = true;
}

fw::bt::Status make_frame(fw::Msg<orb::StereoFrame> frame, const calib::StereoImuRig& calib,
                          const OrbTrackingParams& params, fw::Write<map::Atlas> atlas,
                          fw::Write<tracking::Tracker> tracker, bool& inertial) {
    tracking::Tracker& t = *tracker;
    map::Atlas& a = *atlas;
    if (params.sensor == "stereo_inertial") t.inertial = true;
    else if (params.sensor == "stereo") t.inertial = false;
    else throw std::invalid_argument("orb_tracking: sensor must be stereo or stereo_inertial, not " + params.sensor);
    inertial = t.inertial;
    t.max_frames = params.fps;
    t.frames_to_reset_imu = params.fps;  // the reference leaves mnFramesToResetIMU unset: its settings' fps

    if (!t.has_calib || !sameCalib(t.calib, calib)) loadCalib(a, t, calib);
    t.th_depth = t.baseline * float(params.th_depth);
    if (t.inertial) a.currentMap().inertial = true;  // SetInertialSensor
    else a.currentMap();

    tracking::FrameSetup s;
    s.id = t.next_frame_id++;
    s.timestamp = double(frame.stamp().ns) * 1e-9;
    s.rig = t.rig;
    s.Tlr = t.Tlr;
    s.baseline = t.baseline;
    s.th_depth = t.th_depth;
    s.has_imu_calib = t.inertial;
    s.T_cb = t.T_cb;
    t.current = tracking::makeFrame(std::static_pointer_cast<const orb::StereoFrame>(frame.any().data), s);
    t.new_keyframe = false;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(make_frame,
                      INPUTS(("frame", orb::StereoFrame, latest), ("calib", calib::StereoImuRig, latest),
                             ("params", OrbTrackingParams, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("tracker", tracking::Tracker, write)),
                      OUTPUTS(("inertial", bool)));
