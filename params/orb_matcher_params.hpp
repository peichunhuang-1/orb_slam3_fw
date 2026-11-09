#pragma once
// Parameters of the orb_matcher block (algorithms/orb_matcher). Every field has a default, so the port may
// stay unconnected; connect config://orb_matcher.yaml -> <block>.params to set them. A change rebuilds the
// matcher from the next stereo pair on. Defaults and their measurements: algorithms/orb_matcher/README.md.
#include <string>

#include "fw/fw.hpp"

struct OrbMatcherParams {
    // matching
    int th_dist = 75;
    double ratio = 0.7;
    int octave_window = 1;
    double sigma_px = 1.0;
    double gate_k = 3.0;
    double min_depth = 0.15;
    double max_depth = 0.0;
    double min_epipole_deg = 5.0;
    // triangulation
    double min_parallax_deg = 1.146;
    double chi2 = 5.991;
    // sub-pixel refinement
    bool subpixel = true;
    int subpixel_half_window = 5;
    int subpixel_range = 5;
    std::string subpixel_fit = "ssd";
    // calibration
    double max_fit_error_px = 0.5;
};
FW_PARAMS(OrbMatcherParams,
    GROUP("matching", PERSIST(ask),
          FIELD(th_dist, DEFAULT(75), RANGE(0, 256), DESC("Hamming upper limit ((TH_HIGH + TH_LOW) / 2 in ORB-SLAM3)")),
          FIELD(ratio, DEFAULT(0.7), RANGE(0.01, 1.0),
                DESC("best < ratio x second best along the epipolar line (repeated texture); 0.7 = ORB-SLAM3 fisheye")),
          FIELD(octave_window, DEFAULT(1), RANGE(0, 16), DESC("|octave left - octave right| <= this")),
          FIELD(sigma_px, DEFAULT(1.0), RANGE(0.01, 100.0), DESC("keypoint noise at level 0, pixels (level l: x scale^l)")),
          FIELD(gate_k, DEFAULT(3.0), RANGE(0.1, 100.0), DESC("epipolar and disparity gates at gate_k x sigma")),
          FIELD(min_depth, DEFAULT(0.15), RANGE(0.0, 1000.0), DESC("nearest range from the left camera, m (sets the largest disparity)")),
          FIELD(max_depth, DEFAULT(0.0), RANGE(0.0, 100000.0), DESC("farthest range, m; 0 = unlimited")),
          FIELD(min_epipole_deg, DEFAULT(5.0), RANGE(0.0, 89.9), DESC("bearings closer than this to the baseline stay monocular"))),
    GROUP("triangulation", PERSIST(ask),
          FIELD(min_parallax_deg, DEFAULT(1.146), RANGE(0.0, 89.9),
                DESC("disparity angle below this: no stereo point (1.146 = ORB-SLAM3 TriangulateMatches); 0 = off")),
          FIELD(chi2, DEFAULT(5.991), RANGE(0.01, 1000.0), DESC("reprojection test: |e|^2 < chi2 x sigma^2 in both images"))),
    GROUP("subpixel", PERSIST(ask),
          FIELD(subpixel, DEFAULT(true),
                DESC("refine the right keypoint along the epipolar line (needs orb_extractor keep_pyramid)")),
          FIELD(subpixel_half_window, DEFAULT(5), RANGE(1, 32), DESC("patch half size w: (2w+1)^2 patches")),
          FIELD(subpixel_range, DEFAULT(5), RANGE(1, 32), DESC("search +-L level pixels along the epipolar line")),
          FIELD(subpixel_fit, DEFAULT(std::string("ssd")),
                DESC("ssd: SSD + parabola (best); sad: SAD + parabola (ORB-SLAM3 pinhole); sad_v: SAD + V-shaped fit"))),
    GROUP("calibration", PERSIST(ask),
          FIELD(max_fit_error_px, DEFAULT(0.5), RANGE(0.0, 100.0),
                DESC("radtan calibration: KB8 fit p99 error above this (pixels) is reported as degraded"))))
