// types/geometry/se3.hpp: composition, inverse and the group action against plain matrices, and the
// quaternion staying unit through long chains (Sophus normalizes after each product).
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

#include "types/geometry/se3.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

std::mt19937 rng(3);

geometry::SE3d randomPose() {
    std::normal_distribution<double> n(0, 1);
    const Eigen::Vector3d axis = Eigen::Vector3d(n(rng), n(rng), n(rng)).normalized();
    const double angle = std::uniform_real_distribution<double>(-3.1, 3.1)(rng);
    return geometry::SE3d(Eigen::AngleAxisd(angle, axis).toRotationMatrix(), Eigen::Vector3d(n(rng), n(rng), n(rng)));
}

}  // namespace

int main() {
    double worst = 0;
    for (int i = 0; i < 1000; ++i) {
        const geometry::SE3d a = randomPose(), b = randomPose();
        const Eigen::Vector3d p(1.5, -0.3, 2.0);
        const Eigen::Matrix4d A = a.matrix(), B = b.matrix();
        worst = std::max(worst, ((a * b).matrix() - A * B).cwiseAbs().maxCoeff());
        worst = std::max(worst, (a.inverse().matrix() - A.inverse()).cwiseAbs().maxCoeff());
        worst = std::max(worst, (a * p - (A * p.homogeneous()).head<3>()).cwiseAbs().maxCoeff());
        worst = std::max(worst, (geometry::SE3d(A).matrix() - A).cwiseAbs().maxCoeff());
        worst = std::max(worst, (a.matrix3x4() - A.topRows<3>()).cwiseAbs().maxCoeff());
    }
    check(worst < 1e-12, "operations vs matrices, worst " + std::to_string(worst));

    // A chain of 100000 float products stays a rotation
    geometry::SE3f T;
    const geometry::SE3f step(Eigen::AngleAxisf(0.01f, Eigen::Vector3f(1, 2, 3).normalized()).toRotationMatrix(),
                              Eigen::Vector3f(0.01f, 0, 0));
    for (int i = 0; i < 100000; ++i) T = T * step;
    const Eigen::Matrix3f R = T.rotationMatrix();
    check(std::fabs(T.unitQuaternion().norm() - 1.f) < 1e-6f && (R * R.transpose() - Eigen::Matrix3f::Identity()).norm() < 1e-5f,
          "unit quaternion after a long chain");
    const geometry::SE3f I = T * T.inverse();
    check(I.translation().norm() < 1e-2f && std::fabs(std::fabs(I.unitQuaternion().w()) - 1.f) < 1e-6f, "T * T^-1");

    std::printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
