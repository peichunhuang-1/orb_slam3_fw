#include "algorithms/orb_tracking/track_with_motion_model/search_by_projection.hpp"

#include "algorithms/orb_tracking/shared/rotation_histogram.hpp"
#include "algorithms/orb_tracking/shared/tracking_ops.hpp"
#include "types/orb/hamming.hpp"

namespace orb_tracking {

namespace {

constexpr int kThHigh = 100;  // TH_HIGH

// The best current feature of one view (view 1: right indices) for a descriptor; -1 if none <= TH_HIGH
int bestMatch(const tracking::Frame& current, const map::Atlas& atlas, const std::vector<std::size_t>& candidates,
              int view, const std::uint8_t* d) {
    const orb::FrameFeatures& ff = *current.features;
    const int offset = view == 0 ? 0 : ff.n_left;
    int best_dist = 256, best = -1;
    for (std::size_t c : candidates) {
        const int i2 = int(c) + offset;
        if (const map::MapPoint* p = atlas.point(current.points[std::size_t(i2)]); p && p->n_obs > 0) continue;
        const int dist = orb::hamming256(d, ff.descriptor(i2));
        if (dist < best_dist) {
            best_dist = dist;
            best = i2;
        }
    }
    return best_dist <= kThHigh ? best : -1;
}

std::vector<std::size_t> window(const orb::FrameFeatures& ff, int view, const Eigen::Vector2f& uv, float radius,
                                int octave, bool forward, bool backward) {
    if (forward) return ff.inArea(view, uv(0), uv(1), radius, octave);
    if (backward) return ff.inArea(view, uv(0), uv(1), radius, 0, octave);
    return ff.inArea(view, uv(0), uv(1), radius, octave - 1, octave + 1);
}

}  // namespace

int searchByProjection(tracking::Frame& current, const tracking::Frame& last, const map::Atlas& atlas, float th) {
    int nmatches = 0;
    RotationHistogram hist;
    const orb::FrameFeatures& ff = *current.features;
    const orb::FrameFeatures& lf = *last.features;
    const camera::KannalaBrandt8& cam = rigOf(atlas, current).camera(0);
    const float w = float(ff.stereo->width), h = float(ff.stereo->height);

    const geometry::SE3f& Tcw = current.pose();
    const Eigen::Vector3f twc = Tcw.inverse().translation();
    const Eigen::Vector3f tlc = last.pose() * twc;
    const bool forward = tlc(2) > current.baseline;
    const bool backward = -tlc(2) > current.baseline;

    for (int i = 0; i < last.size(); ++i) {
        const map::MapPoint* p = atlas.point(last.points[std::size_t(i)]);
        if (!p || last.outlier[std::size_t(i)]) continue;

        const Eigen::Vector3f x3Dc = Tcw * p->pos;
        const float invzc = 1.0f / x3Dc(2);
        if (invzc < 0) continue;
        const Eigen::Vector2f uv = cam.project_orb(x3Dc);
        if (uv(0) < 0 || uv(0) > w || uv(1) < 0 || uv(1) > h) continue;

        const cv::KeyPoint& kl = lf.keypoint(i);
        const int octave = kl.octave;
        const float radius = th * ff.scale_factors[std::size_t(octave)];
        const std::vector<std::size_t> left = window(ff, 0, uv, radius, octave, forward, backward);
        if (left.empty()) continue;

        const std::uint8_t* d = p->descriptor.data();
        const int best = bestMatch(current, atlas, left, 0, d);
        if (best >= 0) {
            current.points[std::size_t(best)] = p->id;
            ++nmatches;
            hist.add(kl.angle, ff.keypoint(best).angle, best);
        }

        // The right view, with the left camera's model
        const Eigen::Vector3f x3Dr = current.Trl * x3Dc;
        const Eigen::Vector2f uvr = cam.project_orb(x3Dr);
        const std::vector<std::size_t> right = window(ff, 1, uvr, radius, octave, forward, backward);
        const int best_r = bestMatch(current, atlas, right, 1, d);
        if (best_r >= 0) {
            current.points[std::size_t(best_r)] = p->id;
            ++nmatches;
            hist.add(kl.angle, ff.keypoint(best_r).angle, best_r);
        }
    }

    hist.forEachRejected([&](int index) {
        current.points[std::size_t(index)] = map::kNone;
        --nmatches;
    });
    return nmatches;
}

}  // namespace orb_tracking
