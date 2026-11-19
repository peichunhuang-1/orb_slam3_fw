#pragma once
// The operations that keep the atlas consistent: maps, keyframes and their covisibility graph and spanning
// tree, map points and their observations (ORB-SLAM3's Atlas, Map, KeyFrame, MapPoint and KeyFrameDatabase
// methods that change more than one object). Same logic as there; where ORB-SLAM3 iterates a container keyed
// by pointer, this one goes by id.
//
// Only the KB8 stereo case: every keyframe has two cameras, so each observation counts one (MapPoint's nObs).
#include <set>
#include <vector>

#include <Eigen/Core>

#include "types/map/atlas.hpp"
#include "types/map/vocabulary.hpp"

namespace map {

// ---- maps
MapId createNewMap(Atlas& a);                     // CreateNewMap: the current map is stored, a new one is current
void clearMap(Atlas& a, MapId m);                 // Map::clear: its keyframes leave it (map = kNone), sets emptied
void eraseMapFromBowIndex(Atlas& a, MapId m);     // KeyFrameDatabase::clearMap (before clearMap)
void clearAtlas(Atlas& a);                        // clearAtlas, KeyFrameDatabase::clear and the id restart of Reset
// AddCamera for a stereo pair: the index of an equal rig (same intrinsics, size and extrinsics) or of the new one
int addRig(Atlas& a, const camera::StereoRig& rig);

// ---- keyframes
// The new keyframe gets the next id and the current map (KeyFrame's constructor: bImu = the map's IMU is
// initialized). It is not in the map until insertKeyFrame.
KeyFrameId createKeyFrame(Atlas& a, KeyFrame kf);
void insertKeyFrame(Atlas& a, KeyFrameId kf);     // Atlas::AddKeyFrame (Map::AddKeyFrame)
void computeBoW(KeyFrame& kf, const Vocabulary& voc);  // if not done yet (levels up 4)
void addToBowIndex(Atlas& a, KeyFrameId kf);      // KeyFrameDatabase::add
void addConnection(Atlas& a, KeyFrameId kf, KeyFrameId other, int weight);
void eraseConnection(Atlas& a, KeyFrameId kf, KeyFrameId other);
void updateBestCovisibles(Atlas& a, KeyFrameId kf);
void updateConnections(Atlas& a, KeyFrameId kf);  // from the points' observations; first time: the parent
std::vector<KeyFrameId> covisiblesByWeight(const KeyFrame& kf, int w);  // weight >= w, best first
void changeParent(Atlas& a, KeyFrameId kf, KeyFrameId parent);
void setKeyFrameBad(Atlas& a, KeyFrameId kf);    // KeyFrame::SetBadFlag (not the map's first keyframe)
int trackedMapPoints(const Atlas& a, const KeyFrame& kf, int min_obs);  // good points (with >= min_obs if > 0)

// ---- map points
// A new point at pos (world) with ref_kf as reference, in that keyframe's map (MapPoint(Pos, pRefKF, pMap));
// not in the map until insertMapPoint
MapPointId createMapPoint(Atlas& a, const Eigen::Vector3f& pos, KeyFrameId ref_kf);
void insertMapPoint(Atlas& a, MapPointId p);      // Atlas::AddMapPoint
void addObservation(Atlas& a, MapPointId p, KeyFrameId kf, int index);
void eraseObservation(Atlas& a, MapPointId p, KeyFrameId kf);  // bad when 2 or fewer remain
void setPointBad(Atlas& a, MapPointId p);                     // MapPoint::SetBadFlag
void replace(Atlas& a, MapPointId p, MapPointId by);           // MapPoint::Replace
void computeDistinctiveDescriptors(Atlas& a, MapPointId p);
void updateNormalAndDepth(Atlas& a, MapPointId p);
// A stereo point of keyframe kf at its left feature i, at pos (world): made, observed at i and at its right
// match, set in the keyframe, its descriptor, normal and depth computed, put in the map. As
// StereoInitialization and CreateNewKeyFrame make them.
MapPointId createStereoPoint(Atlas& a, KeyFrameId kf, int i, const Eigen::Vector3f& pos);

}  // namespace map
