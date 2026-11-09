#include "algorithms/orb_matcher/stereo_rig.hpp"

#include <stdexcept>

#include <Eigen/LU>   // determinant()
#include <Eigen/SVD>

namespace orb_matcher {

StereoRig::StereoRig(const KannalaBrandt8& left, const KannalaBrandt8& right, const Eigen::Matrix3f& R_12,
                     const Eigen::Vector3f& t_12)
    : cam_{left, right}, t_12_(t_12) {
    if (left.width() != right.width() || left.height() != right.height())
        throw std::invalid_argument("StereoRig: left and right image sizes differ");
    const Eigen::Matrix3d R = R_12.cast<double>();
    if ((R * R.transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-3 || R.determinant() < 0)
        throw std::invalid_argument("StereoRig: R_12 is not a rotation");
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R_12_ = (svd.matrixU() * svd.matrixV().transpose()).cast<float>();
    if (t_12.norm() < 1e-4f) throw std::invalid_argument("StereoRig: baseline shorter than 0.1 mm");
}

StereoRig StereoRig::resized(int width, int height) const {
    return StereoRig(cam_[0].resized(width, height), cam_[1].resized(width, height), R_12_, t_12_);
}

}  // namespace orb_matcher
