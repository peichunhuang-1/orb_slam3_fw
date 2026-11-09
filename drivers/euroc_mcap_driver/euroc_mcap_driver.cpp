// euroc_mcap_driver: plays a EuRoC MAV .mcap (ROS 2 bag, e.g. bags/V1_02_medium.mcap) into fw ports.
//
//   left, right   fw::Image (mono8)           /cam0, /cam1 image_raw/compressed (PNG), decoded here
//   imu           fw::Imu                     /imu0
//   groundtruth   fw::PoseStamped             /groundtruth/pose (body in world)
//   calib         calib::StereoImuRig         once, before the first image: intrinsics from /camN/camera_info,
//                                             extrinsics from /tf_static (body -> camN, imu), IMU noise from params
//
// Stamps are each message's header.stamp (the log time when zero). Messages go out in log-time order, paced by
// `rate` (0: as fast as possible); `lossless` waits for the box to be idle before each one. It publishes /clock
// at each message's log time, so the graph needs use_sim_time: true. It ends with EOS after the last message.
//
// Why not FW_REGISTER_MCAP_DRIVER: its ports carry the bag's messages as they are, and the images here are PNG
// CompressedImage; the calibration is a fixed set of messages folded into one value.
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <mcap/reader.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "fw/fw.hpp"
#include "fw/ros/msgs/geometry.hpp"
#include "fw/ros/msgs/image.hpp"
#include "fw/ros/msgs/sensor.hpp"
#include "params/euroc_mcap_driver_params.hpp"
#include "types/calib/stereo_imu_rig.hpp"

namespace {

namespace fs = std::filesystem;

// --- files -------------------------------------------------------------------------------------------------------

// The nearest folder at or above `start` with a package.xml
std::optional<fs::path> find_project_root(fs::path start) {
    std::error_code ec;
    start = fs::absolute(start, ec);
    for (fs::path p = start; !p.empty(); p = p.parent_path()) {
        if (fs::exists(p / "package.xml", ec)) return p;
        if (p == p.parent_path()) break;
    }
    return std::nullopt;
}

// The project root: above the graph fw_container runs (--graph <file>), else above the working directory
fs::path project_root() {
    std::ifstream in("/proc/self/cmdline", std::ios::binary);
    std::vector<std::string> args;
    for (std::string a; std::getline(in, a, '\0');) args.push_back(a);
    for (std::size_t i = 0; i + 1 < args.size(); ++i) {
        if (args[i] == "--graph") {
            if (auto r = find_project_root(fs::path(args[i + 1]).parent_path())) return *r;
        }
    }
    if (auto r = find_project_root(fs::current_path())) return *r;
    return fs::current_path();
}

// bags://x.mcap -> <project>/bags/x.mcap; a relative path is relative to the project; an absolute one stays
fs::path resolve_file(const std::string& file) {
    constexpr std::string_view kBags = "bags://";
    fs::path p;
    if (file.rfind(kBags, 0) == 0) {
        p = project_root() / "bags" / file.substr(kBags.size());
    } else {
        p = file;
        if (p.is_relative()) p = project_root() / p;
    }
    if (!fs::is_regular_file(p)) throw std::runtime_error("euroc_mcap_driver: no file '" + p.string() + "' (file: " + file + ")");
    return p;
}

// --- decoding ----------------------------------------------------------------------------------------------------

template <class T>
T deserialize(const mcap::Message& m) {
    rclcpp::SerializedMessage s(m.dataSize);
    auto& raw = s.get_rcl_serialized_message();
    std::memcpy(raw.buffer, m.data, m.dataSize);
    raw.buffer_length = m.dataSize;
    T out;
    static const rclcpp::Serialization<T> serialization;
    serialization.deserialize_message(&s, &out);
    return out;
}

fw::Time stamp_of(const builtin_interfaces::msg::Time& t, mcap::Timestamp log_time) {
    const std::int64_t ns = std::int64_t{t.sec} * 1000000000 + t.nanosec;
    return fw::Time{ns != 0 ? ns : static_cast<std::int64_t>(log_time)};
}

fw::Image decode_image(const sensor_msgs::msg::CompressedImage& m) {
    const cv::Mat buf(1, static_cast<int>(m.data.size()), CV_8UC1, const_cast<std::uint8_t*>(m.data.data()));
    fw::Image img;
    img.mat = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    if (img.mat.empty()) throw std::runtime_error("euroc_mcap_driver: cannot decode a '" + m.format + "' image");
    switch (img.mat.type()) {
        case CV_8UC1: img.encoding = "mono8"; break;
        case CV_16UC1: img.encoding = "mono16"; break;
        case CV_8UC3: img.encoding = "bgr8"; break;
        case CV_8UC4: img.encoding = "bgra8"; break;
        default: throw std::runtime_error("euroc_mcap_driver: unsupported decoded image type");
    }
    img.frame_id = m.header.frame_id;
    return img;
}

calib::PinholeCamera camera_of(const sensor_msgs::msg::CameraInfo& m) {
    calib::PinholeCamera c;
    c.frame_id = m.header.frame_id;
    c.width = static_cast<int>(m.width);
    c.height = static_cast<int>(m.height);
    c.fx = m.k[0];
    c.fy = m.k[4];
    c.cx = m.k[2];
    c.cy = m.k[5];
    c.distortion_model = m.distortion_model;
    c.d = m.d;
    return c;
}

Eigen::Isometry3d isometry_of(const geometry_msgs::msg::Transform& t) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z).normalized().toRotationMatrix();
    T.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
    return T;
}

void throw_on_problem(const mcap::Status& s) {
    throw std::runtime_error("euroc_mcap_driver: reading the mcap: " + s.message);
}

// --- calibration -------------------------------------------------------------------------------------------------

struct Calibration {
    calib::StereoImuRig rig;
    fw::Time stamp;
};

// Reads the first CameraInfo of each camera, /tf_static and the first IMU message (its frame_id), from the start
// of the bag; stops once it has them all
Calibration read_calibration(mcap::McapReader& reader, const EurocMcapParams& p) {
    std::optional<sensor_msgs::msg::CameraInfo> left, right;
    std::optional<tf2_msgs::msg::TFMessage> tf;
    std::optional<std::string> imu_frame;
    mcap::Timestamp tf_log_time = 0;

    mcap::ReadMessageOptions opts;
    opts.topicFilter = [&](std::string_view t) {
        return t == p.left_info_topic || t == p.right_info_topic || t == p.tf_static_topic || t == p.imu_topic;
    };
    for (const auto& v : reader.readMessages(throw_on_problem, opts)) {
        const std::string& t = v.channel->topic;
        if (t == p.left_info_topic && !left) left = deserialize<sensor_msgs::msg::CameraInfo>(v.message);
        else if (t == p.right_info_topic && !right) right = deserialize<sensor_msgs::msg::CameraInfo>(v.message);
        else if (t == p.tf_static_topic && !tf) {
            tf = deserialize<tf2_msgs::msg::TFMessage>(v.message);
            tf_log_time = v.message.logTime;
        } else if (t == p.imu_topic && !imu_frame) imu_frame = deserialize<sensor_msgs::msg::Imu>(v.message).header.frame_id;
        if (left && right && tf && imu_frame) break;
    }
    if (!left) throw std::runtime_error("euroc_mcap_driver: no CameraInfo on '" + p.left_info_topic + "'");
    if (!right) throw std::runtime_error("euroc_mcap_driver: no CameraInfo on '" + p.right_info_topic + "'");
    if (!tf) throw std::runtime_error("euroc_mcap_driver: no TFMessage on '" + p.tf_static_topic + "' (the extrinsics)");
    if (!imu_frame) throw std::runtime_error("euroc_mcap_driver: no Imu on '" + p.imu_topic + "'");

    Calibration c;
    calib::StereoImuRig& rig = c.rig;
    rig.left = camera_of(*left);
    rig.right = camera_of(*right);
    rig.imu_frame = *imu_frame;

    // body -> sensor: the parent of the left camera is the body frame; every sensor must hang from it
    std::map<std::string, geometry_msgs::msg::TransformStamped> by_child;
    for (const auto& x : tf->transforms) by_child[x.child_frame_id] = x;
    const auto extrinsic = [&](const std::string& frame) -> Eigen::Isometry3d {
        if (frame == rig.body_frame) return Eigen::Isometry3d::Identity();
        const auto it = by_child.find(frame);
        if (it == by_child.end())
            throw std::runtime_error("euroc_mcap_driver: no transform to '" + frame + "' on '" + p.tf_static_topic + "'");
        if (it->second.header.frame_id != rig.body_frame)
            throw std::runtime_error("euroc_mcap_driver: '" + frame + "' hangs from '" + it->second.header.frame_id +
                                     "', not from the body frame '" + rig.body_frame + "'");
        return isometry_of(it->second.transform);
    };
    const auto left_tf = by_child.find(rig.left.frame_id);
    if (left_tf == by_child.end())
        throw std::runtime_error("euroc_mcap_driver: no transform to '" + rig.left.frame_id + "' on '" + p.tf_static_topic + "'");
    rig.body_frame = left_tf->second.header.frame_id;
    rig.T_body_left = extrinsic(rig.left.frame_id);
    rig.T_body_right = extrinsic(rig.right.frame_id);
    rig.T_body_imu = extrinsic(rig.imu_frame);

    rig.imu.rate_hz = p.imu_rate_hz;
    rig.imu.gyro_noise = p.gyro_noise;
    rig.imu.gyro_walk = p.gyro_walk;
    rig.imu.accel_noise = p.accel_noise;
    rig.imu.accel_walk = p.accel_walk;

    c.stamp = tf->transforms.empty() ? fw::Time{static_cast<std::int64_t>(tf_log_time)}
                                     : stamp_of(tf->transforms.front().header.stamp, tf_log_time);
    return c;
}

// --- playback ----------------------------------------------------------------------------------------------------

// Wall-clock pacing at `rate` x the bag's time; restarts from the current message when the rate changes or when
// it falls behind by more than kMaxLag (no burst to catch up)
class Pacer {
public:
    // False: asked to stop
    bool wait(fw::DriverContext& ctx, mcap::Timestamp t, double rate) {
        if (rate <= 0) return ctx.ok();
        const auto now = Clock::now();
        if (!started_ || rate != rate_) {
            restart(now, t, rate);
            return ctx.ok();
        }
        const auto due = wall0_ + std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double, std::nano>(static_cast<double>(t - bag0_) / rate));
        if (due > now) return ctx.sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(due - now));
        if (now - due > kMaxLag) restart(now, t, rate);
        return ctx.ok();
    }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr auto kMaxLag = std::chrono::milliseconds(100);
    void restart(Clock::time_point now, mcap::Timestamp t, double rate) {
        started_ = true;
        wall0_ = now;
        bag0_ = t;
        rate_ = rate;
    }
    bool started_ = false;
    Clock::time_point wall0_;
    mcap::Timestamp bag0_ = 0;
    double rate_ = 0;
};

enum class Topic { left, right, imu, groundtruth };

void euroc_mcap_driver(fw::DriverContext& ctx) {
    auto p = ctx.params<EurocMcapParams>();
    const fs::path path = resolve_file(p->file);

    mcap::McapReader reader;
    if (const auto s = reader.open(path.string()); !s.ok())
        throw std::runtime_error("euroc_mcap_driver: cannot open '" + path.string() + "': " + s.message);

    bool lossless = p->lossless;
    ctx.set_lossless(lossless);

    const Calibration cal = read_calibration(reader, *p);
    ctx.publish_clock(cal.stamp);
    ctx.emit("calib", cal.rig, cal.stamp);

    // The topics are read once: changing them takes a restart
    const std::map<std::string, Topic, std::less<>> topics = {
        {p->left_topic, Topic::left}, {p->right_topic, Topic::right},
        {p->imu_topic, Topic::imu}, {p->groundtruth_topic, Topic::groundtruth}};
    mcap::ReadMessageOptions opts;
    opts.topicFilter = [&](std::string_view t) { return topics.find(t) != topics.end(); };

    Pacer pacer;
    for (const auto& v : reader.readMessages(throw_on_problem, opts)) {
        p = ctx.params<EurocMcapParams>();  // rate and lossless follow hot reloads
        if (p->lossless != lossless) ctx.set_lossless(lossless = p->lossless);
        if (!pacer.wait(ctx, v.message.logTime, p->rate)) return;
        if (lossless && !ctx.wait_idle()) return;
        ctx.publish_clock(fw::Time{static_cast<std::int64_t>(v.message.logTime)});

        switch (topics.find(v.channel->topic)->second) {
            case Topic::left:
            case Topic::right: {
                const auto m = deserialize<sensor_msgs::msg::CompressedImage>(v.message);
                const char* port = v.channel->topic == p->left_topic ? "left" : "right";
                ctx.emit(port, decode_image(m), stamp_of(m.header.stamp, v.message.logTime));
                break;
            }
            case Topic::imu: {
                const auto m = deserialize<sensor_msgs::msg::Imu>(v.message);
                fw::Imu imu;
                rclcpp::TypeAdapter<fw::Imu, sensor_msgs::msg::Imu>::convert_to_custom(m, imu);
                ctx.emit("imu", std::move(imu), stamp_of(m.header.stamp, v.message.logTime));
                break;
            }
            case Topic::groundtruth: {
                const auto m = deserialize<geometry_msgs::msg::PoseStamped>(v.message);
                fw::PoseStamped pose;
                rclcpp::TypeAdapter<fw::PoseStamped, geometry_msgs::msg::PoseStamped>::convert_to_custom(m, pose);
                ctx.emit("groundtruth", std::move(pose), stamp_of(m.header.stamp, v.message.logTime));
                break;
            }
        }
    }
    ctx.eos();
}

}  // namespace

FW_REGISTER_DRIVER(euroc_mcap_driver,
    PARAMS(EurocMcapParams),
    OUTPUTS(("left", fw::Image), ("right", fw::Image), ("imu", fw::Imu), ("groundtruth", fw::PoseStamped),
            ("calib", calib::StereoImuRig)));
