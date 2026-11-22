// relocalize: Tracking::Relocalization. Candidates from the BoW index, BoW matches with each (at least 15), then
// 5 MLPnP RANSAC iterations per candidate in turn until one gives a pose that, optimized and completed by
// projection (windows 10 then 3), keeps at least 50 inliers.
#include <memory>
#include <set>
#include <vector>

#include "algorithms/orb_tracking/relocalize/mlpnp_solver.hpp"
#include "algorithms/orb_tracking/relocalize/relocalization_candidates.hpp"
#include "algorithms/orb_tracking/relocalize/search_by_projection.hpp"
#include "algorithms/orb_tracking/shared/pose_optimizer.hpp"
#include "algorithms/orb_tracking/shared/search_by_bow.hpp"
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace {

using namespace orb_tracking;

void dropOutliers(tracking::Frame& f) {
    for (std::size_t i = 0; i < f.points.size(); ++i)
        if (f.outlier[i]) f.points[i] = map::kNone;
}

fw::bt::Status relocalize(const map::Vocabulary& vocabulary, fw::Write<tracking::Tracker> tracker) {
    tracking::Tracker& t = *tracker;
    const map::Atlas& a = tracker->view;
    tracking::Frame& f = t.current;
    f.computeBoW(vocabulary);

    const std::vector<map::KeyFrameId> candidates = relocalizationCandidates(a, t, f);
    if (candidates.empty()) return fw::bt::Status::FAILURE;

    const std::size_t n = candidates.size();
    std::vector<std::unique_ptr<MLPnPSolver>> solvers(n);
    std::vector<std::vector<map::MapPointId>> matches(n);
    std::vector<std::vector<const Eigen::Vector3f*>> world(n);  // kept alive for the solvers
    std::vector<bool> discarded(n, false);
    int n_candidates = 0;
    const camera::KannalaBrandt8& cam = rigOf(a, f).camera(0);
    for (std::size_t i = 0; i < n; ++i) {
        const map::KeyFrame* kf = a.keyframe(candidates[i]);
        if (!kf || kf->bad) {
            discarded[i] = true;
            continue;
        }
        if (searchByBoW(*kf, a, f, 0.75f, true, matches[i]) < 15) {
            discarded[i] = true;
            continue;
        }
        world[i] = worldPositions(a, matches[i], true);
        solvers[i] = std::make_unique<MLPnPSolver>(f, cam, world[i]);
        solvers[i]->setRansacParameters(0.99, 10, 300, 6, 0.5f, 5.991f);  // at least 6 points
        ++n_candidates;
    }

    bool match = false;
    while (n_candidates > 0 && !match) {
        for (std::size_t i = 0; i < n; ++i) {
            if (discarded[i]) continue;
            std::vector<bool> inliers;
            int n_inliers = 0;
            bool no_more = false;
            Eigen::Matrix4f Tcw;
            const bool has_pose = solvers[i]->iterate(5, no_more, inliers, n_inliers, Tcw);
            if (no_more) {
                discarded[i] = true;
                --n_candidates;
            }
            if (!has_pose) continue;

            f.setPose(geometry::SE3f(Tcw));
            std::set<map::MapPointId> found;
            for (std::size_t j = 0; j < inliers.size(); ++j) {
                if (inliers[j]) {
                    f.points[j] = matches[i][j];
                    found.insert(matches[i][j]);
                } else {
                    f.points[j] = map::kNone;
                }
            }

            int good = optimizePose(f, a);
            if (good < 10) continue;
            dropOutliers(f);

            // Few inliers: a coarse projection search, optimize, and if still short a narrower one
            if (good < 50) {
                const map::KeyFrame& kf = *a.keyframe(candidates[i]);
                int additional = searchByProjection(f, kf, a, found, 10, 100);
                if (additional + good >= 50) {
                    good = optimizePose(f, a);
                    if (good > 30 && good < 50) {
                        found.clear();
                        for (map::MapPointId p : f.points)
                            if (p != map::kNone) found.insert(p);
                        additional = searchByProjection(f, kf, a, found, 3, 64);
                        if (good + additional >= 50) {
                            good = optimizePose(f, a);
                            dropOutliers(f);
                        }
                    }
                }
            }
            if (good >= 50) {
                match = true;
                break;
            }
        }
    }
    if (!match) return fw::bt::Status::FAILURE;
    t.last_reloc_frame_id = f.id;
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(relocalize,
                      INPUTS(("vocabulary", map::Vocabulary, latest)),
                      RESOURCES(("tracker", tracking::Tracker, write)));
