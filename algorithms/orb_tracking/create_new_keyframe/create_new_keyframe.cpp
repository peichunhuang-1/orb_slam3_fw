// create_new_keyframe: Tracking::CreateNewKeyFrame. The keyframe of the frame (its points as the frame has them,
// outliers too), a new preintegration from it, and new stereo points for the closest left features without a
// point: all closer than ThDepth, at least the 100 closest. It is not in the map, the covisibility graph or the BoW
// index yet: local mapping does that (ProcessNewKeyFrame) when it gets the keyframe's edit (InsertKeyFrame).
#include <algorithm>
#include <utility>
#include <vector>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "types/map/atlas_ops.hpp"

namespace {

using namespace orb_tracking;

fw::bt::Status create_new_keyframe(fw::Optional<const tracking::MapperStatus&> mapper,
                                   fw::Write<tracking::Tracker> tracker,
                                   fw::Optional<tracking::AtlasEdits>& atlas_edits) {
    tracking::Tracker& t = *tracker;
    const map::Atlas& a = t.view;
    tracking::Frame& f = t.current;
    const tracking::MapperStatus& m = mapperOf(mapper);
    if (m.initializing && !imuInitialized(a)) return fw::bt::Status::FAILURE;
    if (m.stopped) return fw::bt::Status::FAILURE;  // SetNotStop(true) fails
    const std::uint64_t seq = t.next_edit_seq;

    // SetNewBias of the new keyframe: its bias is the frame's (keyFrameOf), and the preintegration it shares with
    // the frame gets it too, before the atlas holds it
    if (f.preintegrated) f.preintegrated->setNewBias(f.bias);
    tracking::AtlasEdit e;
    e.kind = tracking::AtlasEdit::Kind::keyframe;
    e.keyframe = keyFrameOf(f);
    map::prepareKeyFrame(a, e.keyframe);  // bImu: the map's IMU is initialized
    const map::KeyFrameId id = e.keyframe.id;
    if (const map::KeyFrame* last = a.keyframe(t.last_keyframe)) e.keyframe.prev = last->id;  // its next: id
    t.reference_kf = id;
    f.reference_kf = id;
    if (t.inertial) t.preintegrated_from_kf = newPreintegration(t, e.keyframe.bias);

    // The closest stereo points
    std::vector<std::pair<float, int>> by_depth;
    by_depth.reserve(std::size_t(f.size()));
    for (int i = 0; i < f.nLeft(); ++i) {
        const float z = f.features->depth(i);
        if (z > 0) by_depth.emplace_back(z, i);
    }
    std::sort(by_depth.begin(), by_depth.end());
    int n_points = 0;
    map::MapPointId next = a.next_tracking_point_id;
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
            const map::MapPointId p = next;
            next += 2;
            e.points.push_back({p, i, f.unprojectStereo(i)});
            const int r = f.features->rightOf(i);
            if (r >= 0) f.points[std::size_t(f.nLeft() + r)] = p;
            f.points[std::size_t(i)] = p;
        }
        ++n_points;
        if (z > f.th_depth && n_points > 100) break;
    }

    flushCounts(t);
    addEdit(t, std::move(e));  // InsertKeyFrame
    sendEdits(t, seq, atlas_edits);
    t.last_keyframe_frame_id = f.id;
    t.last_keyframe = id;
    t.new_keyframe = true;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(create_new_keyframe, INPUTS(("mapper", tracking::MapperStatus, latest)),
                      RESOURCES(("tracker", tracking::Tracker, write)),
                      OUTPUTS(("atlas_edits", tracking::AtlasEdits)));
