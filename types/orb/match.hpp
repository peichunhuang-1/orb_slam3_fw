#pragma once
// Stereo matches of a left / right orb::Features pair (in-process type "orb/Match"), from orb_matcher.
// Per-keypoint arrays follow the fields of ORB-SLAM3's fisheye stereo Frame (mvLeftToRightMatch,
// mvRightToLeftMatch, mvStereo3Dpoints, mvDepth): indices are positions in the left / right keypoints.
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "fw/fw.hpp"

namespace orb {

struct MatchStats {
  int left = 0, right = 0;        // keypoints with a usable bearing
  int candidates = 0;             // (left, right) pairs inside the epipolar θ window
  int hamming_evals = 0;          // pairs that passed every geometric test
  int matched = 0;
  int rejected_dist = 0;          // best Hamming > th_dist (or no candidate passed the gates)
  int rejected_ratio = 0;
  int rejected_unique = 0;        // the right keypoint went to a closer left one
  int rejected_parallax = 0;      // disparity angle <= 0, or below min_parallax_deg
  int rejected_reproj = 0;
  int rejected_subpixel = 0;      // no clear minimum along the epipolar line, or the patch leaves the image
  double ms = 0;                  // matching time
  double subpixel_ms = 0;         // part of ms spent in the sub-pixel refinement
  // Wall time from the earlier extraction start of the two images (orb::Features::extract_start_ns) to the
  // match being ready: extraction + waiting for the other image + matching. -1 = unknown.
  double latency_ms = -1;
};

struct Match {
  // One per stereo match: queryIdx = left keypoint, trainIdx = right keypoint, distance = Hamming
  std::vector<cv::DMatch> matches;

  std::vector<int> left_to_right;            // per left keypoint, -1 = no match
  std::vector<int> right_to_left;            // per right keypoint, -1 = no match
  std::vector<Eigen::Vector3f> points;       // per left keypoint: 3D point in the left camera frame (m), valid where matched
  std::vector<float> depth;                  // per left keypoint: z in the left camera (m), -1 = no match
  std::vector<Eigen::Vector2f> right_uv;     // per left keypoint: right pixel used (sub-pixel refined), valid where matched
  std::vector<Eigen::Vector3f> bearing_left;   // per left keypoint: unit bearing, left camera ((0, 0, 0): outside the model)
  std::vector<Eigen::Vector3f> bearing_right;  // per right keypoint: unit bearing, right camera
  MatchStats stats;
};

} // orb

FW_REGISTER_TYPE(orb::Match, "orb/Match")
