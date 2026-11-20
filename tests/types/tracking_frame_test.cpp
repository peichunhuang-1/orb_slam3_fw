// types/tracking: Frame from a stereo message (features shared, per-feature arrays), its pose from the body
// pose (SetImuPoseVelocity) and back, the stereo unprojection, the preintegration bias hand-over, and that the
// tracking message and resource types compile with the framework's registration.
#include <cmath>
#include <cstdio>
#include <string>

#include "types/map/keyframe_ref.hpp"
#include "types/tracking/frame.hpp"
#include "types/tracking/mapper_command.hpp"
#include "types/tracking/mapper_status.hpp"
#include "types/tracking/status.hpp"
#include "types/tracking/tracker.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

}  // namespace

int main() {
    auto s = std::make_shared<orb::StereoFrame>();
    s->width = 752;
    s->height = 480;
    s->scale_factors = {1.f, 1.2f, 1.44f};
    for (int v = 0; v < 2; ++v) {
        const int n = v == 0 ? 5 : 3;
        s->descriptors[v] = cv::Mat(n, 32, CV_8U, cv::Scalar(v));
        for (int i = 0; i < n; ++i) s->keypoints[v].emplace_back(100.f + 50 * i, 200.f, 31.f, -1.f, 0.f, i % 3);
    }
    s->left_to_right = {0, -1, 1, -1, 2};
    s->right_to_left = {0, 2, 4};
    s->depth = {2.f, -1.f, 3.f, -1.f, 4.f};
    s->points = {Eigen::Vector3f(0.1f, 0, 2), {}, Eigen::Vector3f(0, 0.2f, 3), {}, Eigen::Vector3f(-1, 0, 4)};

    tracking::FrameSetup setup;
    setup.id = 7;
    setup.timestamp = 12.5;
    setup.Tlr = geometry::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0.11f, 0, 0));
    setup.baseline = 0.11f;
    setup.th_depth = 3.85f;
    setup.has_imu_calib = true;
    setup.T_cb = geometry::SE3f(Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitZ()).toRotationMatrix(),
                                Eigen::Vector3f(0.02f, -0.06f, 0.01f));
    tracking::Frame f = tracking::makeFrame(s, setup);
    check(f.valid() && f.size() == 8 && f.nLeft() == 5 && f.points.size() == 8 && f.outlier.size() == 8,
          "per-feature arrays");
    check(f.features->stereo.get() == s.get(), "features shared, not copied");
    check(f.features->descriptor(6)[0] == 1 && f.features->keypoint(6).pt.x == 150.f && f.features->isRight(5),
          "feature index across views");
    check(std::fabs(f.features->level_sigma2[2] - 1.44f * 1.44f) < 1e-6f &&
              std::fabs(f.features->log_scale_factor - std::log(1.2f)) < 1e-7f,
          "scale levels");
    check(f.features->inArea(0, 150.f, 200.f, 10.f) == std::vector<std::size_t>({1}) &&
              f.features->inArea(0, 150.f, 200.f, 60.f, 1, 1) == std::vector<std::size_t>({1}) &&
              f.features->inArea(1, 125.f, 200.f, 30.f).size() == 2,
          "grid search");

    // Body pose -> camera pose -> body pose
    const Eigen::Matrix3f Rwb = Eigen::AngleAxisf(0.7f, Eigen::Vector3f(1, 1, 0).normalized()).toRotationMatrix();
    const Eigen::Vector3f twb(1, 2, 3), vwb(0.5f, 0, -0.1f);
    f.setImuPoseVelocity(Rwb, twb, vwb);
    check((f.imuPosition() - twb).norm() < 1e-5f && (f.imuRotation() - Rwb).norm() < 1e-5f &&
              (f.velocity() - vwb).norm() == 0 && f.hasPose() && f.hasVelocity(),
          "IMU pose round trip");
    const Eigen::Vector3f Pw = f.unprojectStereo(2);
    check(((f.pose() * Pw) - s->points[2]).norm() < 1e-5f, "stereo unprojection");

    // The bias goes to the shared preintegration from the last keyframe
    imu::Bias b;
    b.g = Eigen::Vector3f(0.01f, 0, 0);
    f.preintegrated = std::make_shared<imu::Preintegrated>(imu::Bias(), imu::Noise());
    f.setNewBias(b);
    check(f.bias.g == b.g && f.preintegrated->updatedBias().g == b.g, "setNewBias");

    tracking::Tracker t;
    t.current = f;
    t.last = t.current;
    check(t.last.features == f.features && t.last.id == 7, "tracker copies frames by sharing the features");
    tracking::Status st;
    tracking::MapperStatus ms;
    tracking::MapperCommand mc;
    map::KeyFrameRef kr;
    check(st.state == tracking::State::NO_IMAGES_YET && ms.accept_keyframes && !mc.interrupt_ba &&
              kr.keyframe_id == map::kNone,
          "defaults");

    std::printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
