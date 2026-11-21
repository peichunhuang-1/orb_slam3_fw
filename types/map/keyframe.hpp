#pragma once
// A keyframe (ORB-SLAM3's KeyFrame), data only; the operations that keep it consistent with its points and
// the covisibility graph are in atlas_ops.hpp. Other keyframes and points are referred to by id.
//
// It is an atlas tile (types/map/docs/DESIGN.md), copied whole on its first edit in a version. So it holds what
// changes; what never changes once the keyframe is made (features, BoW, rig, ...) is in KeyFrameData, shared by
// every version. Its features are shared with the frame it was made from (orb::FrameFeatures).
//
// Not here: the query scratch ORB-SLAM3 keeps on the keyframe (mnRelocQuery, mnTrackReferenceForFrame,
// mnBALocalForKF, ...); it belongs to whoever queries.
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "types/geometry/se3.hpp"
#include "types/imu/preintegrated.hpp"
#include "types/map/flat.hpp"
#include "types/map/ids.hpp"
#include "types/map/tiles.hpp"
#include "types/map/vocabulary.hpp"
#include "types/orb/frame_features.hpp"

namespace map {

// What never changes once the keyframe is made. Replaced whole, never changed in place: other versions of the
// atlas share it.
struct KeyFrameData {
    long frame_id = 0;  // mnFrameId
    double timestamp = 0;
    // Features and the stereo rig (atlas rigs[rig]: mpCamera, mpCamera2; Tlr: right camera in the left, Trl its
    // inverse, both as the frame had them)
    std::shared_ptr<const orb::FrameFeatures> features;
    int rig = -1;
    geometry::SE3f Tlr, Trl;
    float baseline = 0;  // mb
    float th_depth = 0;  // mThDepth: close/far
    BowVector bow;       // empty until computeBoW
    FeatureVector feat;
    // T_cb (body in the left camera) when the rig has an IMU
    bool has_imu_calib = false;
    geometry::SE3f T_cb;
};

// KeyFrame's pose and what is derived from it (one member, so KeyFrame stays within FW_TILE_TYPE's 24)
struct KeyFramePose {
    geometry::SE3f Tcw, Twc;
    Eigen::Vector3f Owb = Eigen::Vector3f::Zero();
    Eigen::Vector3f Vw = Eigen::Vector3f::Zero();
    bool has_velocity = false;
};
FW_TILE_TYPE(KeyFramePose, Tcw, Twc, Owb, Vw, has_velocity);

struct KeyFrame {
    KeyFrameId id = kNone;
    MapId map = kNone;         // the map it is in (kNone once its map was cleared)
    MapId origin_map = kNone;  // mnOriginMapId
    std::shared_ptr<const KeyFrameData> data;

    // Map point per feature index (kNone: none)
    std::vector<MapPointId> points;

    // IMU: the preintegration from the previous keyframe (replaced, never changed in place)
    imu::Bias bias;
    bool imu = false;  // bImu: made after the IMU was initialized
    std::shared_ptr<const imu::Preintegrated> preintegrated;
    KeyFrameId prev = kNone, next = kNone;  // mPrevKF, mNextKF

    // Covisibility graph and spanning tree
    FlatMap<KeyFrameId, int> connected;         // mConnectedKeyFrameWeights
    std::vector<KeyFrameId> ordered_connected;  // by weight, descending
    std::vector<int> ordered_weights;
    bool first_connection = true;
    KeyFrameId parent = kNone;
    FlatSet<KeyFrameId> children, loop_edges, merge_edges;
    bool not_erase = false, to_be_erased = false, bad = false;
    geometry::SE3f Tcp;  // pose relative to the parent, once bad

    int size() const { return data && data->features ? data->features->size() : 0; }  // N
    int nLeft() const { return data && data->features ? data->features->n_left : 0; }

    // Pose (SetPose keeps the derived values; data first)
    void setPose(const geometry::SE3f& T) {
        pose_.Tcw = T;
        pose_.Twc = T.inverse();
        if (data && data->has_imu_calib)
            pose_.Owb = pose_.Twc.rotationMatrix() * data->T_cb.translation() + pose_.Twc.translation();
    }
    const geometry::SE3f& pose() const { return pose_.Tcw; }    // Tcw
    const geometry::SE3f& poseInverse() const { return pose_.Twc; }  // Twc
    Eigen::Vector3f cameraCenter() const { return pose_.Twc.translation(); }
    Eigen::Vector3f rightCameraCenter() const { return (pose_.Twc * data->Tlr).translation(); }
    geometry::SE3f rightPose() const { return data->Trl * pose_.Tcw; }         // Trw
    geometry::SE3f rightPoseInverse() const { return pose_.Twc * data->Tlr; }  // Twr
    Eigen::Vector3f imuPosition() const { return pose_.Owb; }
    Eigen::Matrix3f imuRotation() const { return (pose_.Twc * data->T_cb).rotationMatrix(); }
    geometry::SE3f imuPose() const { return pose_.Twc * data->T_cb; }  // Twb
    void setVelocity(const Eigen::Vector3f& v) {
        pose_.Vw = v;
        pose_.has_velocity = true;
    }
    const Eigen::Vector3f& velocity() const { return pose_.Vw; }
    // SetNewBias: the bias, and the preintegration's (a new copy: the old one may be shared)
    void setNewBias(const imu::Bias& b) {
        bias = b;
        if (preintegrated) {
            auto p = std::make_shared<imu::Preintegrated>(*preintegrated);
            p->setNewBias(b);
            preintegrated = std::move(p);
        }
    }
    bool hasVelocity() const { return pose_.has_velocity; }

    // Covisibles: the n best, and those with weight >= w
    std::vector<KeyFrameId> bestCovisibles(int n) const {
        if (int(ordered_connected.size()) < n) return ordered_connected;
        return std::vector<KeyFrameId>(ordered_connected.begin(), ordered_connected.begin() + n);
    }
    int weight(KeyFrameId kf) const {
        const auto it = connected.find(kf);
        return it == connected.end() ? 0 : it->second;
    }

private:
    KeyFramePose pose_;
    friend void fw_tile_type_declared(const KeyFrame*);  // FW_TILE_TYPE below reads pose_
};

FW_TILE_TYPE(KeyFrame, id, map, origin_map, data, points, bias, imu, preintegrated, prev, next, connected,
             ordered_connected, ordered_weights, first_connection, parent, children, loop_edges, merge_edges,
             not_erase, to_be_erased, bad, Tcp, pose_);

}  // namespace map
