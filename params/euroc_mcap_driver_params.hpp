#pragma once
// Parameters of the euroc_mcap_driver block (drivers/euroc_mcap_driver). Every field has a default, so the port may
// stay unconnected; connect config://euroc_mcap_driver.yaml -> <block>.params to set them.
// The IMU noise is not in the bag: the defaults are EuRoC's imu0/sensor.yaml (ADIS16448).
#include <string>

#include "fw/fw.hpp"

struct EurocMcapParams {
    std::string file = "bags://V1_02_medium.mcap";
    double rate = 1.0;
    bool lossless = false;

    std::string left_topic = "/cam0/image_raw/compressed";
    std::string right_topic = "/cam1/image_raw/compressed";
    std::string left_info_topic = "/cam0/camera_info";
    std::string right_info_topic = "/cam1/camera_info";
    std::string imu_topic = "/imu0";
    std::string tf_static_topic = "/tf_static";
    std::string groundtruth_topic = "/groundtruth/pose";

    double gyro_noise = 1.6968e-04;
    double gyro_walk = 1.9393e-05;
    double accel_noise = 2.0e-03;
    double accel_walk = 3.0e-03;
};
FW_PARAMS(EurocMcapParams,
    GROUP("play",
          FIELD(file, DEFAULT(std::string("bags://V1_02_medium.mcap")),
                DESC("the .mcap: bags://x.mcap (<project>/bags/x.mcap), a path relative to the project, or absolute")),
          FIELD(rate, DEFAULT(1.0), RANGE(0.0, 1000.0), DESC("playback speed (1: as recorded, 0: as fast as possible)")),
          FIELD(lossless, DEFAULT(false), DESC("wait for the box to be idle before each message: nothing is dropped"))),
    GROUP("topics",
          FIELD(left_topic, DEFAULT(std::string("/cam0/image_raw/compressed")), DESC("left images (CompressedImage)")),
          FIELD(right_topic, DEFAULT(std::string("/cam1/image_raw/compressed")), DESC("right images (CompressedImage)")),
          FIELD(left_info_topic, DEFAULT(std::string("/cam0/camera_info")), DESC("left intrinsics (CameraInfo)")),
          FIELD(right_info_topic, DEFAULT(std::string("/cam1/camera_info")), DESC("right intrinsics (CameraInfo)")),
          FIELD(imu_topic, DEFAULT(std::string("/imu0")), DESC("IMU (Imu)")),
          FIELD(tf_static_topic, DEFAULT(std::string("/tf_static")), DESC("extrinsics body -> sensors (TFMessage)")),
          FIELD(groundtruth_topic, DEFAULT(std::string("/groundtruth/pose")), DESC("body pose in world (PoseStamped)"))),
    GROUP("imu_noise",
          FIELD(gyro_noise, DEFAULT(1.6968e-04), RANGE(0.0, 1.0), DESC("gyroscope noise density, rad/s/sqrt(Hz)")),
          FIELD(gyro_walk, DEFAULT(1.9393e-05), RANGE(0.0, 1.0), DESC("gyroscope random walk, rad/s^2/sqrt(Hz)")),
          FIELD(accel_noise, DEFAULT(2.0e-03), RANGE(0.0, 10.0), DESC("accelerometer noise density, m/s^2/sqrt(Hz)")),
          FIELD(accel_walk, DEFAULT(3.0e-03), RANGE(0.0, 10.0), DESC("accelerometer random walk, m/s^3/sqrt(Hz)"))))
