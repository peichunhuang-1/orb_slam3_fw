#pragma once
// Tracking steps that more than one leaf of the tracking tree uses (ORB-SLAM3's Tracking methods and the
// System calls they make). The leaves are in algorithms/orb_tracking/<leaf>/; design: ../docs/DESIGN.md.
#include <vector>

#include "fw/fw.hpp"
#include "types/map/atlas.hpp"
#include "types/tracking/frame.hpp"
#include "types/tracking/mapper_status.hpp"
#include "types/tracking/state.hpp"
#include "types/tracking/tracker.hpp"

namespace orb_tracking {

// ---- the current map
int keyFramesInMap(const map::Atlas& a);  // KeyFramesInMap (0 without a current map)
bool imuInitialized(const map::Atlas& a);  // isImuInitialized
bool inertialBA2(const map::Atlas& a);     // GetIniertialBA2

// ---- local mapping, seen through its status (none: idle, accepting keyframes, the IMU not initialized)
const tracking::MapperStatus& mapperOf(const fw::Optional<const tracking::MapperStatus&>& m);

// ---- resets
// System::ResetActiveMap: done at the start of the next frame (apply_pending_reset); a full reset wins
void requestResetActiveMap(tracking::Tracker& t);
// Tracking::CreateMapInAtlas: a new current map, the state back to NO_IMAGES_YET, no last/current frame
void createMapInAtlas(map::Atlas& a, tracking::Tracker& t, tracking::State& state);

// ---- IMU
// Tracking::PredictStateIMU: the current frame's body state from the last keyframe (map updated) or the last
// frame. False without a previous frame or when neither applies.
bool predictStateImu(const map::Atlas& a, tracking::Tracker& t);
// A new preintegration from the last keyframe at bias b (linearized at zero, corrected to b)
std::shared_ptr<imu::Preintegrated> newPreintegration(const tracking::Tracker& t, const imu::Bias& b);

// ---- per point / keyframe scratch (Tracker)
tracking::PointScratch& pointScratch(tracking::Tracker& t, map::MapPointId p);
tracking::RelocScratch& relocScratch(tracking::Tracker& t, map::KeyFrameId k);

// After a pose optimization (TrackReferenceKeyFrame, TrackWithMotionModel): outliers leave the frame, their
// points are marked seen in this frame and out of view on the side of the outlier (TrackReferenceKeyFrame: also
// the left side, always), and nmatches goes down by them. Returns the matches to points with observations
// (nmatchesMap).
enum class OutlierRule { reference_keyframe, motion_model };
int discardPoseOutliers(const map::Atlas& a, tracking::Tracker& t, int& nmatches, OutlierRule rule);

// World positions of point ids (nullptr: none, missing, or bad when good_only), for the optimizers and MLPnP
std::vector<const Eigen::Vector3f*> worldPositions(const map::Atlas& a, const std::vector<map::MapPointId>& points,
                                                   bool good_only);
const camera::StereoRig& rigOf(const map::Atlas& a, const tracking::Frame& f);

// KeyFrame(Frame&): the keyframe of a frame (not yet in the atlas: createKeyFrame)
map::KeyFrame keyFrameOf(const tracking::Frame& f);

}  // namespace orb_tracking
