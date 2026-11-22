// imu_preintegration: the IMU preintegration from the previous frame to each stereo frame, for stereo-inertial
// tracking. ORB-SLAM3 does this inside Tracking (PreintegrateIMU); here it runs upstream, linearized at zero bias,
// and tracking corrects it to its bias estimate through the Jacobians (algorithms/orb_tracking/docs/DESIGN.md,
// "IMU 預積分").
//
//   imu        : fw::Imu (trigger), from euroc_mcap_driver: the raw samples, in the IMU frame
//   frame      : orb::StereoFrame (trigger), from orb_matcher.stereo
//   calib      : calib::StereoImuRig (latest): the IMU noise (continuous densities, as ORB-SLAM3's
//                IMU.NoiseGyro etc.). No IMU rate: each step's dt comes from the sample timestamps, and the
//                noise is discretized at it (types/imu/preintegrated.hpp)
//   frame_imu  : orb::StereoFrame, stamp of the frame: the frame with `imu` set
//
// A frame waits until a sample at or after its time has arrived, then the samples spanning the previous frame
// to it are taken, interpolated to the two frame times (interval.hpp) and integrated at zero bias. Only the
// timestamps decide; no IMU rate or period is assumed. Then:
//   first frame                          sent without `imu` (PreintegrateIMU's "non prev frame")
//   no sample on one side of the interval  sent without `imu`
//   a frame older than the previous one  the samples are dropped and it starts over as a first frame (Track()'s
//                                        mlQueueImuData.clear() when the time goes back)
//   end of the data                      a frame still waiting is integrated with the samples there are
// One frame is sent per call (an output sends at most one message per call); frames that became ready
// together go out on the next samples, and at the end of the data only the first waiting one goes out (the
// rest are counted in the summary).
//
// Health (reported on change; the worst condition wins):
//   error     IMU stalled: more than kMaxPending frames waiting for a sample after them; the oldest is dropped
//             (not sent without `imu`, which would leave a hole in tracking's preintegration from the keyframe)
//   error     IMU gap: two consecutive samples more than kGapFactor × the median of the last kGapWindow sample
//             intervals apart (lost samples: the step over it is interpolated, so the preintegration is wrong)
//   degraded  the time went back (the frames waiting then are dropped), or a frame sent without `imu`
//   ok        again at the first frame integrated from samples after all of the above
#include <algorithm>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "algorithms/imu_preintegration/interval.hpp"
#include "fw/fw.hpp"
#include "fw/ros/msgs/sensor.hpp"
#include "types/calib/stereo_imu_rig.hpp"
#include "types/imu/preintegrated.hpp"
#include "types/orb/stereo_frame.hpp"

namespace {

using imu_preintegration::Sample;

constexpr std::size_t kMaxPending = 16;  // frames (as the frame input's queue)
constexpr double kGapFactor = 3.0;       // × the median sample interval
constexpr std::size_t kGapWindow = 64;   // sample intervals for the median
constexpr std::size_t kGapMinIntervals = 8;  // before this many, no gap check

double seconds(fw::Time t) { return double(t.ns) * 1e-9; }

class ImuPreintegration {
public:
    void on_imu(fw::Context& ctx, fw::Msg<fw::Imu> m, fw::Optional<fw::Msg<orb::StereoFrame>>& out) {
        Sample s;
        s.t = seconds(m.stamp());
        s.a = Eigen::Vector3f(float(m->linear_acceleration.x), float(m->linear_acceleration.y), float(m->linear_acceleration.z));
        s.w = Eigen::Vector3f(float(m->angular_velocity.x), float(m->angular_velocity.y), float(m->angular_velocity.z));
        if (have_last_ && s.t <= last_t_) {
            ++out_of_order_;
            return;
        }
        if (have_last_) check_gap(ctx, s.t - last_t_, s.t);
        last_t_ = s.t;
        have_last_ = true;
        samples_.push_back(s);
        ++n_samples_;
        try_send(ctx, out, false);
    }

    void on_frame(fw::Context& ctx, fw::Msg<orb::StereoFrame> f, const calib::StereoImuRig& cal,
                  fw::Optional<fw::Msg<orb::StereoFrame>>& out) {
        noise_ = imu::Noise::fromContinuous(cal.imu);
        const double t = seconds(f.stamp());
        const double last = pending_.empty() ? (have_prev_ ? t_prev_ : -1e300) : seconds(pending_.back().stamp());
        if (t < last) {
            // the time went back: start over from this frame
            dropped_restart_ += long(pending_.size());
            samples_.clear();
            pending_.clear();
            intervals_.clear();
            have_prev_ = have_last_ = false;
            gap_end_ = -1e300;
            stalled_ = false;
            ++restarts_;
            degraded_ = "the time went back: started over (" + std::to_string(dropped_restart_) +
                        " waiting frames dropped so far)";
            update_health(ctx); // health enter degarded
        }
        pending_.push_back(std::move(f));
        if (pending_.size() > kMaxPending) {
            // no sample after the oldest frame for a while: the IMU stalled
            pending_.pop_front();
            ++dropped_stalled_;
            stalled_ = true;
            update_health(ctx); // health enter error
        }
        try_send(ctx, out, false);
    }

    void on_eos(fw::Context& ctx, fw::Optional<fw::Msg<orb::StereoFrame>>& out) {
        if (!pending_.empty()) try_send(ctx, out, true);
        if (!pending_.empty())
            std::printf("imu_preintegration: %zu frames still waiting at the end of the data, not sent\n", pending_.size());
        std::printf("imu_preintegration: %ld frames (%ld with a preintegration, %.1f steps each), %ld samples, "
                    "%ld out of order, %ld gaps, %ld restarts (%ld frames dropped), %ld frames dropped waiting for "
                    "the IMU\n",
                    sent_, integrated_, integrated_ ? double(steps_) / double(integrated_) : 0.0, n_samples_,
                    out_of_order_, gaps_, restarts_, dropped_restart_, dropped_stalled_);
        std::fflush(stdout);
        (void)ctx;
    }

private:
    // Sends the oldest waiting frame if it is ready (or at the end of the data)
    void try_send(fw::Context& ctx, fw::Optional<fw::Msg<orb::StereoFrame>>& out, bool eos) {
        if (pending_.empty()) return;
        const fw::Msg<orb::StereoFrame>& f = pending_.front();
        const double t_cur = seconds(f.stamp());
        if (!have_prev_) {
            drop_before(t_cur);
            send(out, f, nullptr);  // a first frame: nothing to integrate, not a fault
            t_prev_ = t_cur;
            have_prev_ = true;
            pending_.pop_front();
            return;
        }
        if (!eos && (samples_.empty() || samples_.back().t < t_cur)) return;

        const imu_preintegration::Interval iv = imu_preintegration::intervalMeasurements(samples_, t_prev_, t_cur);
        samples_.erase(samples_.begin(), samples_.begin() + long(iv.consumed));
        std::shared_ptr<imu::Preintegrated> p;
        if (!iv.steps.empty()) {
            p = std::make_shared<imu::Preintegrated>(imu::Bias(), noise_);
            for (const imu::Measurement& m : iv.steps) p->integrate(m);
            ++integrated_;
            steps_ += long(iv.steps.size());
            stalled_ = false;
            if (t_prev_ >= gap_end_) {  // integrated from samples after any gap: all clear
                gap_end_ = -1e300;
                degraded_.clear();
            }
        } else {
            degraded_ = "no IMU sample on one side of a frame interval: frame sent without a preintegration";
        }
        send(out, f, std::move(p));
        t_prev_ = t_cur;
        pending_.pop_front();
        update_health(ctx);
    }

    void send(fw::Optional<fw::Msg<orb::StereoFrame>>& out, const fw::Msg<orb::StereoFrame>& f,
              std::shared_ptr<const imu::Preintegrated> p) {
        orb::StereoFrame copy = *f;
        copy.imu = std::move(p);
        out = fw::make_msg(std::move(copy), f.stamp());
        ++sent_;
    }

    // A sample interval far above the usual one: samples were lost
    void check_gap(fw::Context& ctx, double dt, double t) {
        if (intervals_.size() >= kGapMinIntervals) {
            std::vector<double> v(intervals_.begin(), intervals_.end());
            std::nth_element(v.begin(), v.begin() + long(v.size() / 2), v.end());
            const double median = v[v.size() / 2];
            if (dt > kGapFactor * median) {
                ++gaps_;
                gap_end_ = t;
                gap_ = "IMU gap: " + std::to_string(dt * 1e3) + " ms between samples (usually " +
                       std::to_string(median * 1e3) + " ms) at t = " + std::to_string(t) + " s; " +
                       std::to_string(gaps_) + " gaps so far";
                update_health(ctx); // health enter error
                return;  // not in the median
            }
        }
        intervals_.push_back(dt);
        if (intervals_.size() > kGapWindow) intervals_.pop_front();
    }

    // Reports the worst current condition when it changes
    void update_health(fw::Context& ctx) {
        fw::Health h = fw::Health::ok;
        std::string why = "ok";
        if (stalled_) {
            h = fw::Health::error;
            why = "IMU stalled: no sample after the frames for more than " + std::to_string(kMaxPending) +
                  " frames; dropping the oldest (" + std::to_string(dropped_stalled_) + " so far)";
        } else if (gap_end_ > -1e300) {
            h = fw::Health::error;
            why = gap_;
        } else if (!degraded_.empty()) {
            h = fw::Health::degraded;
            why = degraded_;
        }
        if (h == health_ && why == why_) return;
        if (h == health_ && h == fw::Health::error && stalled_ && why_.rfind("IMU stalled", 0) == 0) return;  // counts only
        health_ = h;
        why_ = why;
        ctx.report(h, why);
    }

    // Samples before the last one at or before the first frame are of no use (the next interval starts there)
    void drop_before(double t) {
        if (samples_.empty()) return;
        samples_.erase(samples_.begin(), samples_.begin() + long(imu_preintegration::lastAtOrBefore(samples_, t)));
    }

    std::vector<Sample> samples_;                    // the queue, ascending in t
    std::deque<fw::Msg<orb::StereoFrame>> pending_;  // frames waiting for their samples
    imu::Noise noise_;
    bool have_prev_ = false;
    double t_prev_ = 0;
    bool have_last_ = false;  // the last sample accepted (order and gap checks)
    double last_t_ = 0;
    std::deque<double> intervals_;  // the last sample intervals, gaps left out

    // Health conditions
    bool stalled_ = false;
    double gap_end_ = -1e300;  // the sample after the last gap (none: -1e300)
    std::string gap_, degraded_;
    fw::Health health_ = fw::Health::ok;
    std::string why_ = "ok";

    long sent_ = 0, integrated_ = 0, steps_ = 0, n_samples_ = 0, out_of_order_ = 0, restarts_ = 0;
    long gaps_ = 0, dropped_restart_ = 0, dropped_stalled_ = 0;
};

}  // namespace

FW_REGISTER_ALGORITHM_CLASS(ImuPreintegration,
    INPUTS(("calib", calib::StereoImuRig, latest)),
    OUTPUTS(("frame_imu", orb::StereoFrame)),
    ON(("imu", fw::Imu, trigger, QUEUE(2000, drop_oldest)), &ImuPreintegration::on_imu, EMITS("frame_imu"),
       MAX_WAIT(fw::unbounded)),
    ON(("frame", orb::StereoFrame, trigger, QUEUE(16, drop_oldest)), &ImuPreintegration::on_frame, USES("calib"),
       EMITS("frame_imu")),
    ON_EOS(&ImuPreintegration::on_eos, EMITS("frame_imu")));
