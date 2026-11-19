// track_reference_keyframe: Tracking::TrackReferenceKeyFrame. BoW matches with the reference keyframe (at least
// 15), the pose from the last frame's, optimized; outliers leave the frame. Stereo: at least 10 matches to points
// with observations; inertial: success whatever is left.
#include "algorithms/orb_tracking/shared/pose_optimizer.hpp"
#include "algorithms/orb_tracking/shared/search_by_bow.hpp"
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using namespace orb_tracking;

fw::bt::Status track_reference_keyframe(fw::Write<map::Atlas> atlas, fw::Write<tracking::Tracker> tracker,
                                        fw::Read<map::Vocabulary> vocabulary) {
    tracking::Tracker& t = *tracker;
    const map::Atlas& a = *atlas;
    tracking::Frame& f = t.current;
    f.computeBoW(*vocabulary);
    const map::KeyFrame* ref = a.keyframe(t.reference_kf);
    if (!ref) return fw::bt::Status::FAILURE;

    std::vector<map::MapPointId> matches;
    int nmatches = searchByBoW(*ref, a, f, 0.7f, true, matches);
    if (nmatches < 15) return fw::bt::Status::FAILURE;

    f.points = matches;
    f.setPose(t.last.pose());
    optimizePose(f, a);
    const int nmatches_map = discardPoseOutliers(a, t, nmatches, OutlierRule::reference_keyframe);
    if (t.inertial) return fw::bt::Status::SUCCESS;
    return nmatches_map >= 10 ? fw::bt::Status::SUCCESS : fw::bt::Status::FAILURE;
}

}  // namespace

FW_REGISTER_BT_ACTION(track_reference_keyframe,
                      RESOURCES(("atlas", map::Atlas, write), ("tracker", tracking::Tracker, write),
                                ("vocabulary", map::Vocabulary, read)));
