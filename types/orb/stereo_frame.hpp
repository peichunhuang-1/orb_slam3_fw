#pragma once
// One stereo frame for tracking (in-process type "orb/StereoFrame"): both views' features and their stereo
// matches, from orb_matcher (port "stereo", stamp of the left features), and the IMU preintegration from the
// previous frame, added by imu_preintegration (port "frame_imu", same stamp). ORB-SLAM3 builds its Frame
// from the same data (Frame's fisheye stereo constructor: mvKeys, mvKeysRight, descriptors, the matches,
// mpImuPreintegratedFrame).
//
// Indices are positions in keypoints[0] (left) and keypoints[1] (right), as orb::Match.
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "fw/fw.hpp"
#include "types/imu/preintegrated.hpp"

namespace orb {

struct StereoFrame {
  // Features, from orb::Features (full-resolution coordinates, octave = pyramid level)
  std::vector<cv::KeyPoint> keypoints[2];      // 0 left, 1 right
  cv::Mat descriptors[2];                      // N x 32 CV_8U, keypoint order
  std::vector<Eigen::Vector3f> bearings[2];    // unit bearing per keypoint, its camera ((0, 0, 0): outside the model)
  int width = 0, height = 0;                   // image size the keypoints are in
  std::vector<float> scale_factors;            // per pyramid level (level 0: 1)

  // Stereo matches, from orb::Match
  std::vector<int> left_to_right;              // per left keypoint, -1 = no match
  std::vector<int> right_to_left;              // per right keypoint, -1 = no match
  std::vector<Eigen::Vector3f> points;         // per left keypoint: 3D point in the left camera (m), where matched
  std::vector<float> depth;                    // per left keypoint: z in the left camera (m), -1 = no match

  // Preintegration from the previous frame's time to this one's, linearized at zero bias
  // (imu_preintegration). Empty for stereo, for the first frame and when no IMU sample fell in between.
  std::shared_ptr<const imu::Preintegrated> imu;
};

} // orb

FW_REGISTER_TYPE(orb::StereoFrame, "orb/StereoFrame")
