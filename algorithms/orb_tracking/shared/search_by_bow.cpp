#include "algorithms/orb_tracking/shared/search_by_bow.hpp"

#include "algorithms/orb_tracking/shared/rotation_histogram.hpp"
#include "types/orb/hamming.hpp"

namespace orb_tracking {

namespace {
constexpr int kThLow = 50;  // ORBmatcher::TH_LOW
}

int searchByBoW(const map::KeyFrame& kf, const map::Atlas& atlas, const tracking::Frame& frame, float nn_ratio,
                bool check_orientation, std::vector<map::MapPointId>& matches) {
    const orb::FrameFeatures& fk = *kf.data->features;
    const orb::FrameFeatures& ff = *frame.features;
    const unsigned n_left_f = unsigned(ff.n_left);
    matches.assign(std::size_t(frame.size()), map::kNone);
    int nmatches = 0;
    RotationHistogram hist;

    auto kf_it = kf.data->feat.begin();
    auto f_it = frame.feat.begin();
    const auto kf_end = kf.data->feat.end();
    const auto f_end = frame.feat.end();
    while (kf_it != kf_end && f_it != f_end) {
        if (kf_it->first == f_it->first) {
            const std::vector<unsigned>& indices_kf = kf_it->second;
            const std::vector<unsigned>& indices_f = f_it->second;
            for (unsigned idx_kf : indices_kf) {
                const map::MapPointId mp = kf.points[idx_kf];
                if (!atlas.goodPoint(mp)) continue;
                const std::uint8_t* d_kf = fk.descriptor(int(idx_kf));

                int best1 = 256, best2 = 256, best_idx = -1;
                int best1_r = 256, best2_r = 256, best_idx_r = -1;
                for (unsigned idx_f : indices_f) {
                    if (matches[idx_f] != map::kNone) continue;
                    const int dist = orb::hamming256(d_kf, ff.descriptor(int(idx_f)));
                    if (idx_f < n_left_f && dist < best1) {
                        best2 = best1;
                        best1 = dist;
                        best_idx = int(idx_f);
                    } else if (idx_f < n_left_f && dist < best2) {
                        best2 = dist;
                    }
                    if (idx_f >= n_left_f && dist < best1_r) {
                        best2_r = best1_r;
                        best1_r = dist;
                        best_idx_r = int(idx_f);
                    } else if (idx_f >= n_left_f && dist < best2_r) {
                        best2_r = dist;
                    }
                }

                if (best1 <= kThLow) {
                    const cv::KeyPoint& kp = fk.keypoint(int(idx_kf));
                    if (float(best1) < nn_ratio * float(best2)) {
                        matches[std::size_t(best_idx)] = mp;
                        if (check_orientation) hist.add(kp.angle, ff.keypoint(best_idx).angle, best_idx);
                        ++nmatches;
                    }
                    if (best1_r <= kThLow) {
                        matches[std::size_t(best_idx_r)] = mp;
                        if (check_orientation) hist.add(kp.angle, ff.keypoint(best_idx_r).angle, best_idx_r);
                        ++nmatches;
                    }
                }
            }
            ++kf_it;
            ++f_it;
        } else if (kf_it->first < f_it->first) {
            kf_it = kf.data->feat.lower_bound(f_it->first);
        } else {
            f_it = frame.feat.lower_bound(kf_it->first);
        }
    }

    if (check_orientation)
        hist.forEachRejected([&](int i) {
            matches[std::size_t(i)] = map::kNone;
            --nmatches;
        });
    return nmatches;
}

}  // namespace orb_tracking
