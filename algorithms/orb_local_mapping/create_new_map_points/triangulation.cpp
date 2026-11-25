#include "algorithms/orb_local_mapping/create_new_map_points/triangulation.hpp"

#include <algorithm>

#include <Eigen/SVD>

#include "types/map/atlas_ops.hpp"
#include "types/orb/hamming.hpp"

namespace orb_local_mapping {

namespace {

constexpr int kThLow = 50;  // ORBmatcher::TH_LOW

Eigen::Matrix<float, 3, 4> matrix3x4(const geometry::SE3f& T) {
    Eigen::Matrix<float, 3, 4> M;
    M << T.rotationMatrix(), T.translation();
    return M;
}

}  // namespace

bool triangulateDlt(const Eigen::Vector3f& x_c1, const Eigen::Vector3f& x_c2, const Eigen::Matrix<float, 3, 4>& Tc1w,
                    const Eigen::Matrix<float, 3, 4>& Tc2w, Eigen::Vector3f& x3D) {
    Eigen::Matrix4f A;
    A.row(0) = x_c1(0) * Tc1w.row(2) - Tc1w.row(0);
    A.row(1) = x_c1(1) * Tc1w.row(2) - Tc1w.row(1);
    A.row(2) = x_c2(0) * Tc2w.row(2) - Tc2w.row(0);
    A.row(3) = x_c2(1) * Tc2w.row(2) - Tc2w.row(1);
    const Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
    const Eigen::Vector4f h = svd.matrixV().col(3);
    if (h(3) == 0) return false;
    x3D = h.head<3>() / h(3);
    return true;
}

float triangulateMatches(const camera::KannalaBrandt8& cam1, const camera::KannalaBrandt8& cam2,
                         const Eigen::Vector3f& r1, const Eigen::Vector3f& r2,
                         const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const Eigen::Matrix3f& R12,
                         const Eigen::Vector3f& t12, float sigma1, float sigma2, Eigen::Vector3f& p3D) {
    const Eigen::Vector3f r21 = R12 * r2;
    const float cos_parallax = r1.dot(r21) / (r1.norm() * r21.norm());
    if (cos_parallax > 0.9998) return -1;

    Eigen::Matrix<float, 3, 4> Tcw1, Tcw2;
    Tcw1 << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();
    const Eigen::Matrix3f R21 = R12.transpose();
    Tcw2 << R21, -R21 * t12;
    // KannalaBrandt8::Triangulate divides by w without checking it
    const Eigen::Vector3f r1_xy(r1[0], r1[1], 1.f), r2_xy(r2[0], r2[1], 1.f);
    Eigen::Matrix4f A;
    A.row(0) = r1_xy(0) * Tcw1.row(2) - Tcw1.row(0);
    A.row(1) = r1_xy(1) * Tcw1.row(2) - Tcw1.row(1);
    A.row(2) = r2_xy(0) * Tcw2.row(2) - Tcw2.row(0);
    A.row(3) = r2_xy(1) * Tcw2.row(2) - Tcw2.row(1);
    const Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
    const Eigen::Vector4f h = svd.matrixV().col(3);
    const Eigen::Vector3f x3D = h.head<3>() / h(3);

    const float z1 = x3D(2);
    if (z1 <= 0) return -2;
    const float z2 = R21.row(2).dot(x3D) + Tcw2(2, 3);
    if (z2 <= 0) return -3;

    const Eigen::Vector2f uv1 = cam1.project_orb(x3D);
    const float ex1 = uv1(0) - kp1.pt.x, ey1 = uv1(1) - kp1.pt.y;
    if ((ex1 * ex1 + ey1 * ey1) > 5.991 * sigma1) return -4;

    const Eigen::Vector3f x3D2 = R21 * x3D + Tcw2.col(3);
    const Eigen::Vector2f uv2 = cam2.project_orb(x3D2);
    const float ex2 = uv2(0) - kp2.pt.x, ey2 = uv2(1) - kp2.pt.y;
    if ((ex2 * ex2 + ey2 * ey2) > 5.991 * sigma2) return -5;

    p3D = x3D;
    return z1;
}

int searchForTriangulation(const map::Atlas& a, const map::KeyFrame& kf1, const map::KeyFrame& kf2, bool coarse,
                           std::vector<std::pair<int, int>>& pairs) {
    Bearings b1(*kf1.data->features, a.rig(kf1.data->rig)), b2(*kf2.data->features, a.rig(kf2.data->rig));
    return searchForTriangulation(a, kf1, kf2, coarse, pairs, b1, b2);
}

int searchForTriangulation(const map::Atlas& a, const map::KeyFrame& kf1, const map::KeyFrame& kf2, bool coarse,
                           std::vector<std::pair<int, int>>& pairs, Bearings& b1, Bearings& b2) {
    const orb::FrameFeatures& f1 = *kf1.data->features;
    const orb::FrameFeatures& f2 = *kf2.data->features;
    const camera::StereoRig& rig1 = a.rig(kf1.data->rig);
    const camera::StereoRig& rig2 = a.rig(kf2.data->rig);

    // Camera 2 in camera 1 for the four (left / right) combinations: T12 = T1w · Tw2
    const geometry::SE3f T1w = kf1.pose(), Tr1w = kf1.rightPose();
    const geometry::SE3f Tw2 = kf2.poseInverse(), Twr2 = kf2.rightPoseInverse();
    const geometry::SE3f T12[2][2] = {{T1w * Tw2, T1w * Twr2}, {Tr1w * Tw2, Tr1w * Twr2}};  // [right1][right2]
    Eigen::Matrix3f R12[2][2];
    Eigen::Vector3f t12[2][2];
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            R12[i][j] = T12[i][j].rotationMatrix();
            t12[i][j] = T12[i][j].translation();
        }

    std::vector<bool> matched2(std::size_t(kf2.size()), false);
    std::vector<int> matches12(std::size_t(kf1.size()), -1);

    auto it1 = kf1.data->feat.begin(), it2 = kf2.data->feat.begin();
    const auto end1 = kf1.data->feat.end(), end2 = kf2.data->feat.end();
    while (it1 != end1 && it2 != end2) {
        if (it1->first == it2->first) {
            for (unsigned idx1 : it1->second) {
                if (kf1.points[idx1] != map::kNone) continue;
                const cv::KeyPoint& kp1 = f1.keypoint(int(idx1));
                const int right1 = f1.isRight(int(idx1)) ? 1 : 0;
                const std::uint8_t* d1 = f1.descriptor(int(idx1));
                int best_dist = kThLow, best_idx2 = -1;
                for (unsigned idx2 : it2->second) {
                    if (matched2[idx2] || kf2.points[idx2] != map::kNone) continue;
                    const int dist = orb::hamming256(d1, f2.descriptor(int(idx2)));
                    if (dist > kThLow || dist > best_dist) continue;
                    const cv::KeyPoint& kp2 = f2.keypoint(int(idx2));
                    const int right2 = f2.isRight(int(idx2)) ? 1 : 0;
                    Eigen::Vector3f p3D;
                    if (coarse ||
                        triangulateMatches(rig1.camera(right1), rig2.camera(right2), b1[int(idx1)], b2[int(idx2)],
                                           kp1, kp2, R12[right1][right2],
                                           t12[right1][right2], f1.level_sigma2[std::size_t(kp1.octave)],
                                           f2.level_sigma2[std::size_t(kp2.octave)], p3D) > 0.0001f) {
                        best_idx2 = int(idx2);
                        best_dist = dist;
                    }
                }
                if (best_idx2 >= 0) matches12[idx1] = best_idx2;
                // ORB-SLAM3 does not set vbMatched2[bestIdx2]: a kf2 feature may be taken by several kf1 features
            }
            ++it1;
            ++it2;
        } else if (it1->first < it2->first) {
            it1 = kf1.data->feat.lower_bound(it2->first);
        } else {
            it2 = kf2.data->feat.lower_bound(it1->first);
        }
    }

    pairs.clear();
    for (std::size_t i = 0; i < matches12.size(); ++i)
        if (matches12[i] >= 0) pairs.emplace_back(int(i), matches12[i]);
    return int(pairs.size());
}

std::vector<map::KeyFrameId> triangulationNeighbours(const map::Atlas& a, map::KeyFrameId id, bool inertial) {
    constexpr int nn = 10;
    const map::KeyFrame& cur = *a.keyframe(id);
    std::vector<map::KeyFrameId> neigh = cur.bestCovisibles(nn);
    if (inertial) {
        const map::KeyFrame* kf = &cur;
        int count = 0;
        while (neigh.size() <= std::size_t(nn) && kf->prev != map::kNone && a.keyframe(kf->prev) && count++ < nn) {
            if (std::find(neigh.begin(), neigh.end(), kf->prev) == neigh.end()) neigh.push_back(kf->prev);
            kf = a.keyframe(kf->prev);
        }
    }
    return neigh;
}

int createNewMapPoints(map::Atlas& a, local_mapping::LocalMapper& m, bool recently_lost) {
    const map::KeyFrameId cur_id = m.current;
    const map::KeyFrame* cur0 = a.keyframe(cur_id);
    if (!cur0) return 0;
    const std::vector<map::KeyFrameId> neigh = triangulationNeighbours(a, cur_id, m.inertial);
    const orb::FrameFeatures& f1 = *cur0->data->features;
    const camera::StereoRig& rig1 = a.rig(cur0->data->rig);
    const float ratio_factor = 1.5f * f1.scale_factor;
    const map::Map* cur_map = a.findMap(cur0->map);
    const bool coarse = m.inertial && recently_lost && cur_map && cur_map->imu_ba2;

    // Ow1 carries over from one neighbour's last match to the next neighbour's baseline check (as in ORB-SLAM3)
    Eigen::Vector3f Ow1 = cur0->cameraCenter();
    Bearings b1(f1, rig1);
    int created = 0;
    std::vector<std::pair<int, int>> pairs;
    for (std::size_t i = 0; i < neigh.size(); ++i) {
        if (i > 0 && !m.queue.empty()) return created;
        const map::KeyFrame* kf2 = a.keyframe(neigh[i]);
        const map::KeyFrame* cur = a.keyframe(cur_id);  // after the previous neighbour's edits
        const orb::FrameFeatures& f2 = *kf2->data->features;
        const camera::StereoRig& rig2 = a.rig(kf2->data->rig);

        Eigen::Vector3f Ow2 = kf2->cameraCenter();
        if ((Ow2 - Ow1).norm() < kf2->data->baseline) continue;

        Bearings b2(f2, rig2);
        searchForTriangulation(a, *cur, *kf2, coarse, pairs, b1, b2);

        for (const auto& [idx1, idx2] : pairs) {
            const cv::KeyPoint& kp1 = f1.keypoint(idx1);
            const cv::KeyPoint& kp2 = f2.keypoint(idx2);
            const bool right1 = f1.isRight(idx1), right2 = f2.isRight(idx2);
            const geometry::SE3f T1 = right1 ? cur->rightPose() : cur->pose();
            const geometry::SE3f T2 = right2 ? kf2->rightPose() : kf2->pose();
            Ow1 = right1 ? cur->rightCameraCenter() : cur->cameraCenter();
            Ow2 = right2 ? kf2->rightCameraCenter() : kf2->cameraCenter();
            const camera::KannalaBrandt8& cam1 = rig1.camera(right1 ? 1 : 0);
            const camera::KannalaBrandt8& cam2 = rig2.camera(right2 ? 1 : 0);
            const Eigen::Matrix<float, 3, 4> Tcw1 = matrix3x4(T1), Tcw2 = matrix3x4(T2);
            const Eigen::Matrix3f Rcw1 = Tcw1.leftCols<3>(), Rcw2 = Tcw2.leftCols<3>();
            const Eigen::Vector3f tcw1 = Tcw1.col(3), tcw2 = Tcw2.col(3);

            // Parallax between the rays
            const Eigen::Vector3f& xn1 = b1[idx1];
            const Eigen::Vector3f& xn2 = b2[idx2];
            const Eigen::Vector3f ray1 = Rcw1.transpose() * xn1, ray2 = Rcw2.transpose() * xn2;
            const float cos_rays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());
            const float cos_stereo = cos_rays + 1;
            Eigen::Vector3f x3D;
            if (!(cos_rays < cos_stereo && cos_rays > 0 &&
                  ((cos_rays < 0.9996 && m.inertial) || (cos_rays < 0.9998 && !m.inertial))))
                continue;  // very low parallax
            if (!triangulateDlt(xn1, xn2, Tcw1, Tcw2, x3D)) continue;

            // In front of both cameras
            const float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
            if (z1 <= 0) continue;
            const float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
            if (z2 <= 0) continue;

            // Reprojection error in both keyframes
            const float sigma1 = f1.level_sigma2[std::size_t(kp1.octave)];
            const Eigen::Vector2f uv1 =
                cam1.project_orb(Eigen::Vector3f(Rcw1.row(0).dot(x3D) + tcw1(0), Rcw1.row(1).dot(x3D) + tcw1(1), z1));
            const float ex1 = uv1.x() - kp1.pt.x, ey1 = uv1.y() - kp1.pt.y;
            if ((ex1 * ex1 + ey1 * ey1) > 5.991 * sigma1) continue;
            const float sigma2 = f2.level_sigma2[std::size_t(kp2.octave)];
            const Eigen::Vector2f uv2 =
                cam2.project_orb(Eigen::Vector3f(Rcw2.row(0).dot(x3D) + tcw2(0), Rcw2.row(1).dot(x3D) + tcw2(1), z2));
            const float ex2 = uv2.x() - kp2.pt.x, ey2 = uv2.y() - kp2.pt.y;
            if ((ex2 * ex2 + ey2 * ey2) > 5.991 * sigma2) continue;

            // Scale consistency
            const float dist1 = (x3D - Ow1).norm(), dist2 = (x3D - Ow2).norm();
            if (dist1 == 0 || dist2 == 0) continue;
            if (m.far_points && (dist1 >= m.th_far_points || dist2 >= m.th_far_points)) continue;
            const float ratio_dist = dist2 / dist1;
            const float ratio_octave =
                f1.scale_factors[std::size_t(kp1.octave)] / f2.scale_factors[std::size_t(kp2.octave)];
            if (ratio_dist * ratio_factor < ratio_octave || ratio_dist > ratio_octave * ratio_factor) continue;

            // Triangulation is successful
            const map::KeyFrameId kf2_id = kf2->id;
            const map::MapPointId p = map::createMapPoint(a, x3D, cur_id);
            map::addObservation(a, p, cur_id, idx1);
            map::addObservation(a, p, kf2_id, idx2);
            a.editKeyFrame(cur_id)->points[std::size_t(idx1)] = p;
            a.editKeyFrame(kf2_id)->points[std::size_t(idx2)] = p;
            map::computeDistinctiveDescriptors(a, p);
            map::updateNormalAndDepth(a, p);
            map::insertMapPoint(a, p);
            m.recent_points.push_back(p);
            ++created;
            // the edits may have copied the tiles: take them again
            cur = a.keyframe(cur_id);
            kf2 = a.keyframe(kf2_id);
        }
    }
    return created;
}

}  // namespace orb_local_mapping
