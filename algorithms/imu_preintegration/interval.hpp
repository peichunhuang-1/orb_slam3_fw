#pragma once
// The IMU measurements between two frames, as ORB-SLAM3's Tracking::PreintegrateIMU makes them from its sample
// queue: which samples it takes, and the interpolation of the first and last steps to the frame times.
//
// Selection, against the period kImuPeriod (mImuPer, 1 ms):
//   samples with t < t_prev − kImuPeriod are dropped
//   samples with t < t_cur − kImuPeriod are taken
//   the first sample with t ≥ t_cur − kImuPeriod is taken too, and stays in the queue for the next interval
// With n + 1 samples taken there are n steps; n = 0 (or no sample) gives no preintegration.
// Step i between samples i and i+1 (time differences taken in double, then stored in float, as the original):
//   first (i = 0, n > 1)   the mean of the samples interpolated to t_prev and the second one; dt = t₁ − t_prev
//   middle                 the mean of the two samples; dt = t_{i+1} − t_i
//   last (i = n−1, n > 1)  the mean of sample n−1 and the samples interpolated to t_cur; dt = t_cur − t_{n−1}
//   only (n = 1)           sample 0 as is; dt = t_cur − t_prev
#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "types/imu/preintegrated.hpp"

namespace imu_preintegration {

constexpr double kImuPeriod = 0.001;  // s (ORB-SLAM3's mImuPer)

struct Sample {
    double t = 0;                                 // s
    Eigen::Vector3f a = Eigen::Vector3f::Zero();  // specific force (m/s²)
    Eigen::Vector3f w = Eigen::Vector3f::Zero();  // angular velocity (rad/s)
};

struct Interval {
    std::vector<imu::Measurement> steps;  // empty: no preintegration for this frame
    std::size_t consumed = 0;             // samples [0, consumed) of the queue are done with (pop them)
};

// samples: the queue, ascending in t, from its front
inline Interval intervalMeasurements(const std::vector<Sample>& samples, double t_prev, double t_cur) {
    Interval out;
    std::vector<const Sample*> v;
    std::size_t k = 0;
    for (; k < samples.size(); ++k) {
        const Sample& m = samples[k];
        if (m.t < t_prev - kImuPeriod) continue;
        v.push_back(&m);
        if (m.t >= t_cur - kImuPeriod) break;  // kept in the queue
    }
    out.consumed = k;
    const int n = int(v.size()) - 1;
    if (n <= 0) return out;
    out.steps.reserve(std::size_t(n));
    for (int i = 0; i < n; ++i) {
        const Sample& s0 = *v[i];
        const Sample& s1 = *v[i + 1];
        imu::Measurement m;
        if (i == 0 && i < n - 1) {
            const float tab = float(s1.t - s0.t), tini = float(s0.t - t_prev);
            m.a = (s0.a + s1.a - (s1.a - s0.a) * (tini / tab)) * 0.5f;
            m.w = (s0.w + s1.w - (s1.w - s0.w) * (tini / tab)) * 0.5f;
            m.dt = float(s1.t - t_prev);
        } else if (i < n - 1) {
            m.a = (s0.a + s1.a) * 0.5f;
            m.w = (s0.w + s1.w) * 0.5f;
            m.dt = float(s1.t - s0.t);
        } else if (i > 0 && i == n - 1) {
            const float tab = float(s1.t - s0.t), tend = float(s1.t - t_cur);
            m.a = (s0.a + s1.a - (s1.a - s0.a) * (tend / tab)) * 0.5f;
            m.w = (s0.w + s1.w - (s1.w - s0.w) * (tend / tab)) * 0.5f;
            m.dt = float(t_cur - s0.t);
        } else {  // i == 0 && n == 1
            m.a = s0.a;
            m.w = s0.w;
            m.dt = float(t_cur - t_prev);
        }
        out.steps.push_back(m);
    }
    return out;
}

}  // namespace imu_preintegration
