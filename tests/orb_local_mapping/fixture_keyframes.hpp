#pragma once
// The keyframes of a local mapping fixture (tests/fixtures/orb_slam3/README.md): kfK_Tcw, kfK_timestamp,
// kfK_kp_left / kfK_kp_right (x, y, octave), kfK_desc and, when present, kfK_feat; keyframe k's prev is k - 1.
#include <memory>
#include <string>
#include <vector>

#include "tests/orb_tracking/fixture_frame.hpp"
#include "types/map/atlas_ops.hpp"

namespace fixture {

inline std::shared_ptr<const orb::FrameFeatures> keyframeFeatures(const File& f, const std::string& p, const Rig& rig) {
    auto s = std::make_shared<orb::StereoFrame>();
    s->width = rig.rig.width();
    s->height = rig.rig.height();
    s->scale_factors = rig.scales();
    const Eigen::MatrixXd& D = f[p + "desc"];
    const char* names[2] = {"kp_left", "kp_right"};
    int row = 0;
    for (int v = 0; v < 2; ++v) {
        const Eigen::MatrixXd& k = f[p + names[v]];
        s->descriptors[v] = cv::Mat(int(k.rows()), 32, CV_8U);
        for (int i = 0; i < k.rows(); ++i, ++row) {
            const int oct = int(k(i, 2));
            s->keypoints[v].emplace_back(float(k(i, 0)), float(k(i, 1)), 31.f * s->scale_factors[std::size_t(oct)],
                                         -1.f, 0.f, oct);
            for (int b = 0; b < 32; ++b) s->descriptors[v].at<std::uint8_t>(i, b) = std::uint8_t(D(row, b));
        }
    }
    const std::size_t nl = s->keypoints[0].size();
    s->left_to_right.assign(nl, -1);
    s->right_to_left.assign(s->keypoints[1].size(), -1);
    s->depth.assign(nl, -1.f);
    s->points.assign(nl, Eigen::Vector3f::Zero());
    return std::make_shared<const orb::FrameFeatures>(s);
}

// Keyframes 0..K-1 into a new atlas with the fixture rig, linked prev / next, in the map; their ids
inline std::vector<map::KeyFrameId> addKeyFrames(map::Atlas& a, const File& f, const Rig& rig, int K) {
    a.currentMap();
    map::addRig(a, rig.rig);
    std::vector<map::KeyFrameId> ids;
    for (int k = 0; k < K; ++k) {
        const std::string p = "kf" + std::to_string(k) + "_";
        auto d = std::make_shared<map::KeyFrameData>();
        d->frame_id = k;
        d->timestamp = f.scalar(p + "timestamp");
        d->features = keyframeFeatures(f, p, rig);
        d->rig = 0;
        d->Tlr = rig.Tlr;
        d->Trl = rig.Tlr.inverse();
        d->baseline = rig.Tlr.translation().norm();
        d->th_depth = rig.th_depth;
        if (f.has(p + "feat")) {
            const Eigen::MatrixXd& feat = f[p + "feat"];
            for (int i = 0; i < feat.rows(); ++i) d->feat[map::NodeId(feat(i, 0))].push_back(unsigned(feat(i, 1)));
        }
        map::KeyFrame kf;
        kf.data = std::move(d);
        kf.setPose(se3f(f[p + "Tcw"]));
        if (k > 0) kf.prev = ids.back();
        const map::KeyFrameId id = map::createKeyFrame(a, std::move(kf));
        if (k > 0) a.editKeyFrame(ids.back())->next = id;
        map::insertKeyFrame(a, id);
        ids.push_back(id);
    }
    return ids;
}

}  // namespace fixture
