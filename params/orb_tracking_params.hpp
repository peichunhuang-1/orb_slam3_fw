#pragma once
// Parameters of the tracking behavior block (algorithms/orb_tracking): the values ORB-SLAM3 reads from its
// settings file for Tracking. Every field has a default; connect config://orb_tracking.yaml -> tracking.params
// to set them. Camera and IMU calibration come from the calib port, not from here.
#include <string>

#include "fw/fw.hpp"

struct OrbTrackingParams {
    std::string sensor = "stereo_inertial";  // System::STEREO / IMU_STEREO
    int fps = 20;                            // Camera.fps
    double th_depth = 60.0;                  // Stereo.ThDepth, in baselines
    bool insert_kfs_when_lost = true;        // IMU.InsertKFsWhenLost
};
FW_PARAMS(OrbTrackingParams,
    GROUP("tracking", PERSIST(ask),
          FIELD(sensor, DEFAULT(std::string("stereo_inertial")),
                DESC("stereo | stereo_inertial (stereo_inertial needs frame.imu from imu_preintegration)")),
          FIELD(fps, DEFAULT(20), RANGE(1, 1000),
                DESC("camera rate (Camera.fps): keyframe insertion (mMaxFrames) and the IMU reset after relocalization")),
          FIELD(th_depth, DEFAULT(60.0), RANGE(1.0, 1000.0),
                DESC("close / far stereo points at this many baselines (Stereo.ThDepth)")),
          FIELD(insert_kfs_when_lost, DEFAULT(true),
                DESC("stereo_inertial: keep inserting keyframes while RECENTLY_LOST (IMU.InsertKFsWhenLost)"))))
