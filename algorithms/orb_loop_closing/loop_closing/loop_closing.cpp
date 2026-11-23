// loop_closing: one round of ORB-SLAM3's LoopClosing::RunLoop per keyframe local mapping hands over
// (algorithms/orb_loop_closing/docs/DESIGN.md). It only reads the atlas (versioned: it never waits); what it
// changes goes to local mapping as a loop_closing/LoopResult.
//
// A round: ResetIfRequested (the reset epoch changed: the state is cleared), NewDetectCommonRegions
// (place_recognition/), then what Run does with a detection before CorrectLoop / MergeLocal:
//   merge  stereo-inertial with the current map's IMU not initialized: aborted. Else Tmw (mg2oMergeSmw) and
//          T_old_new (mSold_new, yaw only when both maps are inertial and the current one has BA1); the scale
//          check of the original is gone (fixed scale: always 1).
//   loop   inertial map: the correction's roll and pitch under 0.008 rad and yaw under 0.349, else a bad loop;
//          after BA2, yaw only.
//   then   the region's state reset (SetErase of its last current and matched keyframes), a merge resets the
//          loop region too.
// A loop to correct: the flags of the region's reset are marked as coming after the correction
// (erase_after_correction), and big_change records the map's mnBigChangeIdx this detection saw. Local mapping
// corrects (correct_loop/); merges are only counted (phase 4).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "algorithms/orb_loop_closing/place_recognition/place_recognition.hpp"
#include "fw/fw.hpp"
#include "params/orb_loop_closing_params.hpp"
#include "types/geometry/so3.hpp"
#include "types/loop_closing/loop_keyframe.hpp"
#include "types/loop_closing/loop_result.hpp"
#include "types/map/atlas.hpp"

namespace {

using geometry::SE3d;
using loop_closing::LoopResult;

class LoopClosing {
public:
    ~LoopClosing() {
        if (times_ms_.empty()) return;
        std::vector<double> t = times_ms_;
        std::sort(t.begin(), t.end());
        double sum = 0;
        for (double v : t) sum += v;
        const auto& s = pr_.stats();
        std::fprintf(stderr,
                     "loop_closing: %zu keyframes (%ld skipped early), detection %.1f ms mean, %.1f p95, %.1f max; "
                     "candidates loop %ld merge %ld; loops %ld (bad %ld), merges %ld (aborted %ld); regions lost %ld\n",
                     t.size(), s.skipped, sum / double(t.size()), t[std::size_t(double(t.size() - 1) * 0.95)], t.back(),
                     s.candidates_loop, s.candidates_merge, loops_, bad_loops_, merges_, aborted_merges_,
                     s.regions_lost);
    }

    void on_keyframe(const loop_closing::LoopKeyFrame& kf, const OrbLoopClosingParams& params,
                     fw::Read<map::Atlas> atlas, LoopResult& result) {
        if (kf.epoch != epoch_) {  // ResetIfRequested: the atlas was reset meanwhile
            epoch_ = kf.epoch;
            pr_.clear();
        }
        LoopResult r;
        r.seq = ++seq_;
        r.epoch = kf.epoch;
        r.keyframe = kf.keyframe;
        const map::Atlas& a = *atlas;
        if (!params.active) {
            // send_to_loop_closing's SetNotErase undone (the original never sets it when inactive)
            r.erase_flags.push_back({kf.keyframe, false});
            result = std::move(r);
            return;
        }

        pr_.debug = params.debug;
        const auto t0 = std::chrono::steady_clock::now();
        if (pr_.detect(a, kf.keyframe, r.erase_flags)) handleDetection(a, r);
        times_ms_.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        result = std::move(r);
    }

private:
    void handleDetection(const map::Atlas& a, LoopResult& r) {
        const map::KeyFrame& cur = *a.keyframe(r.keyframe);
        const map::Map& cur_map = *a.findMap(cur.map);
        orb_loop_closing::CommonRegion& merge = pr_.merge();
        orb_loop_closing::CommonRegion& loop = pr_.loop();

        if (merge.detected) {
            const map::KeyFrame& matched = *a.keyframe(merge.matched_kf);
            if (cur_map.inertial && !cur_map.imu_initialized) {
                std::fprintf(stderr, "loop_closing: kf %lu: IMU is not initialized, merge is aborted\n",
                             (unsigned long)cur.id);
                ++aborted_merges_;
            } else {
                const SE3d Tmw2 = matched.pose().cast<double>();
                const SE3d Tcw1 = cur.pose().cast<double>();
                const SE3d Tw2c = merge.Tlw.inverse();
                SE3d T_old_new = Tw2c * Tcw1;
                const map::Map* matched_map = a.findMap(matched.map);
                if (cur_map.inertial && matched_map && matched_map->inertial && cur_map.imu_ba1) {
                    Eigen::Vector3d phi = geometry::log(T_old_new.rotationMatrix());
                    phi(0) = 0;
                    phi(1) = 0;
                    T_old_new = SE3d(geometry::exp(phi), T_old_new.translation());
                }
                r.kind = LoopResult::Kind::merge;
                r.matched_kf = matched.id;
                r.Tmw = Tmw2 * Tw2c * Tcw1;
                r.Tcw = merge.Tlw;
                r.T_old_new = T_old_new;
                r.last_current_kf = merge.last_current_kf;
                r.matched_points = merge.matched_points;
                r.region_points = merge.points;
                ++merges_;
                std::fprintf(stderr, "loop_closing: *Merge detected: kf %lu (map %lu) with kf %lu (map %lu)\n",
                             (unsigned long)cur.id, (unsigned long)cur.map, (unsigned long)matched.id,
                             (unsigned long)matched.map);
            }
            pr_.resetRegion(merge, r.erase_flags);
            if (loop.detected) pr_.resetRegion(loop, r.erase_flags);
        }

        if (loop.detected) {
            const map::KeyFrame& matched = *a.keyframe(loop.matched_kf);
            SE3d Tcw = loop.Tlw;
            bool good = true;
            if (cur_map.inertial) {
                const SE3d Twc = cur.poseInverse().cast<double>();
                SE3d Tww_new = Twc * Tcw;
                Eigen::Vector3d phi = geometry::log(Tww_new.rotationMatrix());
                if (std::fabs(phi(0)) < 0.008f && std::fabs(phi(1)) < 0.008f && std::fabs(phi(2)) < 0.349f) {
                    if (cur_map.imu_ba2) {  // yaw only
                        phi(0) = 0;
                        phi(1) = 0;
                        Tww_new = SE3d(geometry::exp(phi), Tww_new.translation());
                        Tcw = Twc.inverse() * Tww_new;
                    }
                } else {
                    good = false;
                }
            }
            if (good) {
                r.kind = LoopResult::Kind::loop;
                r.erase_after_correction = r.erase_flags.size();
                r.big_change = cur_map.big_change_index;
                r.matched_kf = matched.id;
                r.Tcw = Tcw;
                r.matched_points = loop.matched_points;
                r.region_points = loop.points;
                ++loops_;
                std::fprintf(stderr, "loop_closing: *Loop detected: kf %lu with kf %lu (map %lu)\n",
                             (unsigned long)cur.id, (unsigned long)matched.id, (unsigned long)cur.map);
            } else {
                ++bad_loops_;
                std::fprintf(stderr, "loop_closing: kf %lu with kf %lu: BAD LOOP\n", (unsigned long)cur.id,
                             (unsigned long)matched.id);
            }
            pr_.resetRegion(loop, r.erase_flags);
        }
    }

    orb_loop_closing::PlaceRecognition pr_;
    std::uint64_t epoch_ = 0;
    std::uint64_t seq_ = 0;
    std::vector<double> times_ms_;
    long loops_ = 0, bad_loops_ = 0, merges_ = 0, aborted_merges_ = 0;
};

}  // namespace

FW_REGISTER_ALGORITHM_CLASS(LoopClosing,
    RESOURCES(("atlas", map::Atlas, read)),
    INPUTS(("params", OrbLoopClosingParams, latest)),
    OUTPUTS(("result", loop_closing::LoopResult)),
    ON(("keyframe", loop_closing::LoopKeyFrame, trigger, QUEUE(16, drop_oldest)), &LoopClosing::on_keyframe,
       USES("params", "atlas"), EMITS("result")));
