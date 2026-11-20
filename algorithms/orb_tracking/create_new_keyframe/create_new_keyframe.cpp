// create_new_keyframe: Tracking::CreateNewKeyFrame. The keyframe of the frame (its points as the frame has them,
// outliers too), a new preintegration from it, and new stereo points for the closest left features without a
// point: all closer than ThDepth, at least the 100 closest. It is not in the map, the covisibility graph or the BoW
// index yet: local mapping does that (ProcessNewKeyFrame) when it gets the keyframe output (InsertKeyFrame).
#include <algorithm>
#include <utility>
#include <vector>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "types/map/atlas_ops.hpp"
#include "types/map/keyframe_ref.hpp"

namespace {

using namespace orb_tracking;

fw::bt::Status create_new_keyframe(fw::Optional<const tracking::MapperStatus&> mapper, fw::Write<map::Atlas> atlas,
                                   fw::Write<tracking::Tracker> tracker, fw::Optional<map::KeyFrameRef>& keyframe) {
    tracking::Tracker& t = *tracker;
    map::Atlas& a = *atlas;
    tracking::Frame& f = t.current;
    const tracking::MapperStatus& m = mapperOf(mapper);
    if (m.initializing && !imuInitialized(a)) return fw::bt::Status::FAILURE;
    if (m.stopped) return fw::bt::Status::FAILURE;  // SetNotStop(true) fails

    // SetNewBias of the new keyframe: its bias is the frame's (keyFrameOf), and the preintegration it shares with
    // the frame gets it too, before the atlas holds it
    if (f.preintegrated) f.preintegrated->setNewBias(f.bias);
    const map::KeyFrameId id = map::createKeyFrame(a, keyFrameOf(f));  // bImu: the map's IMU is initialized
    map::KeyFrame& kf = *a.editKeyFrame(id);
    t.reference_kf = id;
    f.reference_kf = id;
    if (map::KeyFrame* last = a.editKeyFrame(t.last_keyframe)) {
        kf.prev = last->id;
        last->next = id;
    }
    if (t.inertial) t.preintegrated_from_kf = newPreintegration(t, kf.bias);

    // The closest stereo points
    std::vector<std::pair<float, int>> by_depth;
    by_depth.reserve(std::size_t(f.size()));
    for (int i = 0; i < f.nLeft(); ++i) {
        const float z = f.features->depth(i);
        if (z > 0) by_depth.emplace_back(z, i);
    }
    std::sort(by_depth.begin(), by_depth.end());
    int n_points = 0;
    for (const auto& [z, i] : by_depth) {
        map::MapPointId& slot = f.points[std::size_t(i)];
        bool create = slot == map::kNone;
        if (!create) {
            const map::MapPoint* p = a.point(slot);
            if (!p || p->n_obs < 1) {
                create = true;
                slot = map::kNone;
            }
        }
        if (create) {
            const map::MapPointId p = map::createStereoPoint(a, id, i, f.unprojectStereo(i));
            const int r = f.features->rightOf(i);
            if (r >= 0) f.points[std::size_t(f.nLeft() + r)] = p;
            f.points[std::size_t(i)] = p;
        }
        ++n_points;
        if (z > f.th_depth && n_points > 100) break;
    }

    keyframe = map::KeyFrameRef{kf.map, id};
    t.last_keyframe_frame_id = f.id;
    t.last_keyframe = id;
    t.new_keyframe = true;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(create_new_keyframe, INPUTS(("mapper", tracking::MapperStatus, latest)),
                      RESOURCES(("atlas", map::Atlas, write), ("tracker", tracking::Tracker, write)),
                      OUTPUTS(("keyframe", map::KeyFrameRef)));
