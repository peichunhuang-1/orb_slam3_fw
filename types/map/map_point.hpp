#pragma once
// A map point (ORB-SLAM3's MapPoint), data only; the operations that keep it consistent with its keyframes are
// in atlas_ops.hpp. Observations are keyed by keyframe id; ORB-SLAM3 keys them by pointer, so where its order
// matters (the representative descriptor, the new reference keyframe) this one is by id. It is an atlas tile
// (types/map/docs/DESIGN.md).
//
// The per-frame scratch ORB-SLAM3 keeps on the point (mTrackProjX, mbTrackInView, mnLastFrameSeen,
// mnTrackReferenceForFrame, ...) belongs to tracking and lives there.
#include <array>
#include <cmath>
#include <cstdint>

#include <Eigen/Core>

#include "types/map/flat.hpp"
#include "types/map/ids.hpp"

namespace map {

// Where a keyframe sees the point: its feature indices, -1 where not seen (right: n_left + right index)
struct Observation {
    int left = -1, right = -1;
};

struct MapPoint {
    MapPointId id = kNone;
    MapId map = kNone;          // the map it is in (kNone once its map was cleared)
    MapId origin_map = kNone;   // mnOriginMapId
    KeyFrameId first_kf = kNone;  // mnFirstKFid
    long first_frame = 0;         // mnFirstFrame

    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();  // mean viewing direction
    std::array<std::uint8_t, 32> descriptor{};         // the representative one
    KeyFrameId ref_kf = kNone;

    FlatMap<KeyFrameId, Observation> observations;
    int n_obs = 0;  // features that see it (a stereo observation counts 2)

    int visible = 1, found = 1;  // mnVisible, mnFound
    bool bad = false;
    bool in_map = false;          // counted in its map's n_points (Map::AddMapPoint, until EraseMapPoint)
    MapPointId replaced = kNone;  // set by replace (fused into another point)

    float min_distance = 0, max_distance = 0;  // scale invariance

    float foundRatio() const { return float(found) / float(visible); }
    float minDistanceInvariance() const { return 0.8f * min_distance; }
    float maxDistanceInvariance() const { return 1.2f * max_distance; }
    // The pyramid level the point should be seen at from distance dist (PredictScale)
    int predictScale(float dist, float log_scale_factor, int levels) const {
        const float ratio = max_distance / dist;
        int s = int(std::ceil(std::log(ratio) / log_scale_factor));
        if (s < 0) s = 0;
        else if (s >= levels) s = levels - 1;
        return s;
    }
    bool inKeyFrame(KeyFrameId kf) const { return observations.count(kf) != 0; }
    Observation indexIn(KeyFrameId kf) const {
        const auto it = observations.find(kf);
        return it == observations.end() ? Observation() : it->second;
    }
};

}  // namespace map
