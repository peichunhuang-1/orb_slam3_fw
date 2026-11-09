#pragma once
// ORB features of one image (in-process type "orb/Features"), from orb_extractor.
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "fw/fw.hpp"

namespace orb {

struct Features {
  // Full-resolution coordinates, octave = pyramid level (ORB-SLAM3 conventions)
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;               // N x 32 CV_8U, keypoint order (cv::ORB bit layout)

  int width = 0, height = 0;         // size of the image the keypoints are in
  std::vector<float> scale_factors;  // scale of each pyramid level (level 0: 1)
  // Optional (orb_extractor keep_pyramid): the 8-bit pyramid the keypoints were detected on, level l
  // of size ~(width, height) / scale_factors[l], keypoint at pt / scale_factors[octave] on it. Read by
  // orb_matcher's sub-pixel refinement. Empty when not kept.
  std::vector<cv::Mat> pyramid;

  // std::chrono::steady_clock time (ns since its epoch) when the extraction of this image started: lets a
  // later block measure the pipeline latency in wall time (orb_matcher: extraction -> match). 0 = unknown.
  std::int64_t extract_start_ns = 0;
};

} // orb

FW_REGISTER_TYPE(orb::Features, "orb/Features")
