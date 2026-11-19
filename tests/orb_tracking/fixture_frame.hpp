#pragma once
// A tracking::Frame and its rig from the observation fields of the ORB-SLAM3 fixtures
// (tests/fixtures/orb_slam3/README.md: kp_left, kp_right, mp_of_index, mp_world, left_to_right; rig.txt).
#include <memory>
#include <vector>

#include "tests/common/orb_slam3_fixture.hpp"
#include "types/camera/stereo_rig.hpp"
#include "types/tracking/frame.hpp"

namespace fixture {

inline geometry::SE3f se3f(const Eigen::MatrixXd& T) { return geometry::SE3f(Eigen::Matrix4f(T.cast<float>())); }

struct Rig {
    camera::StereoRig rig;
    geometry::SE3f Tlr, T_bc;  // right in left; left camera in the body
    int levels = 0;
    float scale_factor = 1, th_depth = 0;

    Rig() {
        const File r("rig.txt");
        const Eigen::MatrixXd& L = r["left_kb8"];
        const Eigen::MatrixXd& R = r["right_kb8"];
        const int w = int(r["image_size"](0)), h = int(r["image_size"](1));
        const camera::KannalaBrandt8 left(float(L(0)), float(L(1)), float(L(2)), float(L(3)), float(L(4)), float(L(5)),
                                          float(L(6)), float(L(7)), w, h);
        const camera::KannalaBrandt8 right(float(R(0)), float(R(1)), float(R(2)), float(R(3)), float(R(4)),
                                           float(R(5)), float(R(6)), float(R(7)), w, h);
        Tlr = se3f(r["T_left_right"]);
        T_bc = se3f(r["T_body_left"]);
        rig = camera::StereoRig(left, right, Tlr.rotationMatrix(), Tlr.translation());
        levels = int(r.scalar("n_levels"));
        scale_factor = float(r.scalar("scale_factor"));
        th_depth = float(r.scalar("th_depth_m"));
    }

    std::vector<float> scales() const {
        std::vector<float> s(std::size_t(levels), 1.f);
        for (std::size_t i = 1; i < s.size(); ++i) s[i] = s[i - 1] * scale_factor;
        return s;
    }
};

struct Observations {
    tracking::Frame frame;
    std::vector<Eigen::Vector3f> mp_world;
    std::vector<const Eigen::Vector3f*> world;  // per frame index, nullptr: none
};

inline Observations frameOf(const File& f, const Rig& rig, long id = 1, double timestamp = 0) {
    auto s = std::make_shared<orb::StereoFrame>();
    s->width = rig.rig.width();
    s->height = rig.rig.height();
    s->scale_factors = rig.scales();
    const char* kp_name[2] = {"kp_left", "kp_right"};
    for (int v = 0; v < 2; ++v) {
        const Eigen::MatrixXd& k = f[kp_name[v]];
        s->descriptors[v] = cv::Mat(int(k.rows()), 32, CV_8U, cv::Scalar(0));
        for (int i = 0; i < k.rows(); ++i)
            s->keypoints[v].emplace_back(float(k(i, 0)), float(k(i, 1)), 31.f * s->scale_factors[std::size_t(k(i, 2))],
                                         -1.f, 0.f, int(k(i, 2)));
    }
    const std::size_t n_left = s->keypoints[0].size();
    s->left_to_right.assign(n_left, -1);
    s->right_to_left.assign(s->keypoints[1].size(), -1);
    s->depth.assign(n_left, -1.f);
    s->points.assign(n_left, Eigen::Vector3f::Zero());
    const Eigen::MatrixXd& l2r = f["left_to_right"];
    for (std::size_t i = 0; i < n_left; ++i) {
        s->left_to_right[i] = int(l2r(Eigen::Index(i)));
        if (s->left_to_right[i] >= 0) s->right_to_left[std::size_t(s->left_to_right[i])] = int(i);
    }

    tracking::FrameSetup setup;
    setup.id = id;
    setup.timestamp = timestamp;
    setup.rig = 0;
    setup.Tlr = rig.Tlr;
    setup.baseline = rig.Tlr.translation().norm();
    setup.th_depth = rig.th_depth;
    setup.has_imu_calib = true;
    setup.T_cb = rig.T_bc.inverse();

    Observations o;
    o.frame = tracking::makeFrame(s, setup);
    const Eigen::MatrixXd& mp = f["mp_world"];
    for (int i = 0; i < mp.rows(); ++i) o.mp_world.emplace_back(float(mp(i, 0)), float(mp(i, 1)), float(mp(i, 2)));
    const Eigen::MatrixXd& of = f["mp_of_index"];
    o.world.assign(std::size_t(o.frame.size()), nullptr);
    for (int i = 0; i < o.frame.size(); ++i) {
        const int m = int(of(i));
        if (m < 0) continue;
        o.world[std::size_t(i)] = &o.mp_world[std::size_t(m)];
        o.frame.points[std::size_t(i)] = map::MapPointId(m);
    }
    return o;
}

}  // namespace fixture
