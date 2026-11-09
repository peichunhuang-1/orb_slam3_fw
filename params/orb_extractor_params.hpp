#pragma once
// Parameters of the orb_extractor block (algorithms/orb_extractor). Every field has a default, so the
// port may stay unconnected; connect config://orb_extractor.yaml -> <block>.params to set them.
#include <string>

#include "fw/fw.hpp"

struct OrbExtractorParams {
    std::string backend = "auto";
    int nfeatures = 1000;
    double scale_factor = 1.2;
    int nlevels = 8;
    int ini_th_fast = 20;
    int min_th_fast = 7;
    bool per_cell_threshold = true;
    bool distribute = true;
    double cells_per_feature = 1.0;
    double abort_corner_fraction = 0.0;
    int edge_threshold = 19;
    bool keep_pyramid = true;
};
FW_PARAMS(OrbExtractorParams,
    FIELD(backend, DEFAULT(std::string("auto")),
          DESC("auto: GPU (Vulkan), CPU if the GPU cannot be used; gpu: GPU only; cpu: CPU only (same results)")),
    GROUP("orb", PERSIST(ask),
          FIELD(nfeatures, DEFAULT(1000), RANGE(1, 100000), DESC("keypoints per image (ORB-SLAM3 nFeatures)")),
          FIELD(scale_factor, DEFAULT(1.2), RANGE(1.01, 4.0), DESC("scale between pyramid levels")),
          FIELD(nlevels, DEFAULT(8), RANGE(1, 16), DESC("pyramid levels")),
          FIELD(ini_th_fast, DEFAULT(20), RANGE(1, 254), DESC("FAST threshold (ORB-SLAM3 iniThFAST)")),
          FIELD(min_th_fast, DEFAULT(7), RANGE(1, 254),
                DESC("FAST threshold in grid cells without an ini_th_fast corner (ORB-SLAM3 minThFAST), <= ini_th_fast"))),
    GROUP("extractor", PERSIST(ask),
          FIELD(per_cell_threshold, DEFAULT(true), DESC("per-cell FAST fallback to min_th_fast, as ORB-SLAM3")),
          FIELD(distribute, DEFAULT(true), DESC("spread keypoints over a grid (ORB-SLAM3 quadtree role)")),
          FIELD(cells_per_feature, DEFAULT(1.0), RANGE(0.05, 16.0), DESC("grid cells per keypoint of a level's quota")),
          FIELD(abort_corner_fraction, DEFAULT(0.0), RANGE(0.0, 1.0),
                DESC("abnormal frame (no keypoints) above this fraction of pyramid pixels as FAST corners; 0 = off")),
          FIELD(edge_threshold, DEFAULT(19), RANGE(19, 200), DESC("border without keypoints"))),
    GROUP("output", PERSIST(ask),
          FIELD(keep_pyramid, DEFAULT(true),
                DESC("copy the image pyramid into the features (orb_matcher's sub-pixel refinement; ~1.2x the image per frame)"))))
