#include "algorithms/orb_tracking/relocalize/search_by_projection.hpp"

#include "algorithms/orb_tracking/shared/rotation_histogram.hpp"
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "types/orb/hamming.hpp"

namespace orb_tracking {

int searchByProjection(tracking::Frame& f, const map::KeyFrame& kf, const map::Atlas& a,
                       const std::set<map::MapPointId>& already_found, float th, int orb_dist) {
    int nmatches = 0;
    RotationHistogram hist;
    const orb::FrameFeatures& ff = *f.features;
    const camera::KannalaBrandt8& cam = rigOf(a, f).camera(0);
    const float w = float(ff.stereo->width), h = float(ff.stereo->height);
    const geometry::SE3f& Tcw = f.pose();
    const Eigen::Vector3f Ow = Tcw.inverse().translation();

    for (int i = 0; i < kf.size(); ++i) {
        const map::MapPoint* p = a.goodPoint(kf.points[std::size_t(i)]);
        if (!p || already_found.count(p->id)) continue;

        const Eigen::Vector3f x3Dc = Tcw * p->pos;
        const Eigen::Vector2f uv = cam.project_orb(x3Dc);
        if (uv(0) < 0 || uv(0) > w || uv(1) < 0 || uv(1) > h) continue;

        const float dist = (p->pos - Ow).norm();
        if (dist < p->minDistanceInvariance() || dist > p->maxDistanceInvariance()) continue;
        const int level = p->predictScale(dist, ff.log_scale_factor, ff.levels);
        const float radius = th * ff.scale_factors[std::size_t(level)];
        const std::vector<std::size_t> cands = ff.inArea(0, uv(0), uv(1), radius, level - 1, level + 1);
        if (cands.empty()) continue;

        int best_dist = 256, best = -1;
        for (std::size_t c : cands) {
            if (f.points[c] != map::kNone) continue;
            const int d = orb::hamming256(p->descriptor.data(), ff.descriptor(int(c)));
            if (d < best_dist) {
                best_dist = d;
                best = int(c);
            }
        }
        if (best_dist <= orb_dist) {
            f.points[std::size_t(best)] = p->id;
            ++nmatches;
            hist.add(kf.data->features->keypoint(i).angle, ff.keypoint(best).angle, best);
        }
    }

    hist.forEachRejected([&](int index) {
        f.points[std::size_t(index)] = map::kNone;
        --nmatches;
    });
    return nmatches;
}

}  // namespace orb_tracking
