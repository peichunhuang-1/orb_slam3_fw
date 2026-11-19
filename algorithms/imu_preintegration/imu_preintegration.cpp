// imu_preintegration: the IMU preintegration from the previous frame to each stereo frame, for stereo-inertial
// tracking. ORB-SLAM3 does this inside Tracking (PreintegrateIMU); here it runs upstream, linearized at zero bias,
// and tracking corrects it to its bias estimate through the Jacobians (algorithms/orb_tracking/docs/DESIGN.md,
// "IMU 預積分").
//
//   imu        : fw::Imu (trigger), from euroc_mcap_driver: the raw samples, in the IMU frame
//   frame      : orb::StereoFrame (trigger), from orb_matcher.stereo
//   calib      : calib::StereoImuRig (latest): the IMU noise (continuous densities and rate, as ORB-SLAM3's
//                IMU.NoiseGyro etc.)
//   frame_imu  : orb::StereoFrame, stamp of the frame: the frame with `imu` set
//
// A frame waits until a sample at or after its time − 1 ms has arrived (the sample that ends PreintegrateIMU's
// selection), then the measurements between the previous frame and it are taken and interpolated as there
// (interval.hpp) and integrated at zero bias. Then:
//   first frame                          sent without `imu` (PreintegrateIMU's "non prev frame")
//   no sample in between (one step or less)  sent without `imu`
//   a frame older than the previous one  the samples are dropped and it starts over as a first frame (Track()'s
//                                        mlQueueImuData.clear() when the time goes back)
//   end of the data                      a frame still waiting is integrated with the samples there are
// One frame is sent per call (an output sends at most one message per call); frames that became ready
// together go out on the next samples.
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

double seconds(fw::Time t) { return double(t.ns) * 1e-9; }

class ImuPreintegration {
public:
    void on_imu(fw::Context& ctx, fw::Msg<fw::Imu> m, fw::Optional<fw::Msg<orb::StereoFrame>>& out) {
        Sample s;
        s.t = seconds(m.stamp());
        s.a = Eigen::Vector3f(float(m->linear_acceleration.x), float(m->linear_acceleration.y), float(m->linear_acceleration.z));
        s.w = Eigen::Vector3f(float(m->angular_velocity.x), float(m->angular_velocity.y), float(m->angular_velocity.z));
        if (!samples_.empty() && s.t <= samples_.back().t) {
            ++out_of_order_;
            return;
        }
        samples_.push_back(s);
        ++n_samples_;
        try_send(ctx, out, false);
    }

    void on_frame(fw::Context& ctx, fw::Msg<orb::StereoFrame> f, const calib::StereoImuRig& cal,
                  fw::Optional<fw::Msg<orb::StereoFrame>>& out) {
        if (cal.imu.rate_hz <= 0) {
            ctx.report(fw::Health::error, "calib: no IMU rate (imu.rate_hz)");
            throw std::invalid_argument("imu_preintegration: calib has no IMU rate");
        }
        noise_ = imu::Noise::fromContinuous(cal.imu);
        const double t = seconds(f.stamp());
        const double last = pending_.empty() ? (have_prev_ ? t_prev_ : -1e300) : seconds(pending_.back().stamp());
        if (t < last) {
            // the time went back: start over from this frame
            samples_.clear();
            pending_.clear();
            have_prev_ = false;
            ++restarts_;
        }
        pending_.push_back(std::move(f));
        try_send(ctx, out, false);
    }

    void on_eos(fw::Context& ctx, fw::Optional<fw::Msg<orb::StereoFrame>>& out) {
        if (!pending_.empty()) try_send(ctx, out, true);
        if (!pending_.empty())
            std::printf("imu_preintegration: %zu frames still waiting at the end of the data, not sent\n", pending_.size());
        std::printf("imu_preintegration: %ld frames (%ld with a preintegration, %.1f steps each), %ld samples, "
                    "%ld out of order, %ld restarts\n",
                    sent_, integrated_, integrated_ ? double(steps_) / double(integrated_) : 0.0, n_samples_,
                    out_of_order_, restarts_);
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
            send(out, f, nullptr);
            t_prev_ = t_cur;
            have_prev_ = true;
            pending_.pop_front();
            return;
        }
        if (!eos && (samples_.empty() || samples_.back().t < t_cur - imu_preintegration::kImuPeriod)) return;

        const imu_preintegration::Interval iv = imu_preintegration::intervalMeasurements(samples_, t_prev_, t_cur);
        samples_.erase(samples_.begin(), samples_.begin() + long(iv.consumed));
        std::shared_ptr<imu::Preintegrated> p;
        if (!iv.steps.empty()) {
            p = std::make_shared<imu::Preintegrated>(imu::Bias(), noise_);
            for (const imu::Measurement& m : iv.steps) p->integrate(m);
            ++integrated_;
            steps_ += long(iv.steps.size());
            if (health_ != fw::Health::ok) {
                health_ = fw::Health::ok;
                ctx.report(health_, "ok");
            }
        } else if (health_ != fw::Health::degraded) {
            health_ = fw::Health::degraded;
            ctx.report(health_, "no IMU samples between two frames: frame sent without a preintegration");
        }
        send(out, f, std::move(p));
        t_prev_ = t_cur;
        pending_.pop_front();
    }

    void send(fw::Optional<fw::Msg<orb::StereoFrame>>& out, const fw::Msg<orb::StereoFrame>& f,
              std::shared_ptr<const imu::Preintegrated> p) {
        orb::StereoFrame copy = *f;
        copy.imu = std::move(p);
        out = fw::make_msg(std::move(copy), f.stamp());
        ++sent_;
    }

    // Samples before the first frame are of no use (the next interval starts at it)
    void drop_before(double t) {
        std::size_t k = 0;
        while (k < samples_.size() && samples_[k].t < t - imu_preintegration::kImuPeriod) ++k;
        samples_.erase(samples_.begin(), samples_.begin() + long(k));
    }

    std::vector<Sample> samples_;                    // the queue, ascending in t
    std::deque<fw::Msg<orb::StereoFrame>> pending_;  // frames waiting for their samples
    imu::Noise noise_;
    bool have_prev_ = false;
    double t_prev_ = 0;
    fw::Health health_ = fw::Health::ok;
    long sent_ = 0, integrated_ = 0, steps_ = 0, n_samples_ = 0, out_of_order_ = 0, restarts_ = 0;
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
