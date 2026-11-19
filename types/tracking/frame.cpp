#include "types/tracking/frame.hpp"

#include <opencv2/core.hpp>

namespace tracking {

void Frame::computeBoW(const map::Vocabulary& voc) {
    if (!bow.empty()) return;
    cv::Mat all;  // mDescriptors: left then right
    cv::vconcat(features->stereo->descriptors[0], features->stereo->descriptors[1], all);
    voc.transform(all, bow, feat, 4);
}

Frame makeFrame(std::shared_ptr<const orb::StereoFrame> stereo, const FrameSetup& s) {
    Frame f;
    f.id = s.id;
    f.timestamp = s.timestamp;
    f.features = std::make_shared<const orb::FrameFeatures>(std::move(stereo));
    f.rig = s.rig;
    f.Tlr = s.Tlr;
    f.Trl = s.Tlr.inverse();
    f.baseline = s.baseline;
    f.th_depth = s.th_depth;
    f.has_imu_calib = s.has_imu_calib;
    f.T_cb = s.T_cb;
    f.points.assign(std::size_t(f.size()), map::kNone);
    f.outlier.assign(std::size_t(f.size()), false);
    return f;
}

}  // namespace tracking
