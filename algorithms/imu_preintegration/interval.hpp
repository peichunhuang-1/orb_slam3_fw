#pragma once
// The IMU measurements between two frames: the samples that span [t_prev, t_cur], cut to the frame times by
// linear interpolation. Only the timestamps decide; no IMU rate or period is assumed.
//
// Selection, from the queue (ascending in t):
//   from    the last sample at or before t_prev (the first sample if there is none: its first step
//           extrapolates back to t_prev)
//   to      the first sample at or after t_cur (the last sample if there is none, at the end of the data: its
//           last step extrapolates to t_cur)
//   kept    the queue from the last sample at or before t_cur on: it starts the next interval
// With samples v_0 … v_n taken there are n steps; n = 0 gives no preintegration (no sample on one side of the
// interval, or none at all). Step i covers [max(t_prev, t_i), min(t_cur, t_{i+1})]: the mean of the two samples
// interpolated to its ends, and the length of it as dt (differences in double, stored in float). The dts add up
// to t_cur − t_prev.
//
// ORB-SLAM3's PreintegrateIMU differs: it selects against a fixed 1 ms margin (mImuPer: from the first sample
// at or after t_prev − 1 ms, to the first at or after t_cur − 1 ms) and takes a single step's sample as is. On
// frames that fall on samples (EuRoC: cameras triggered on the IMU clock) both give the same steps.
#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "types/imu/preintegrated.hpp"

namespace imu_preintegration {

struct Sample {
    double t = 0;                                 // s
    Eigen::Vector3f a = Eigen::Vector3f::Zero();  // specific force (m/s²)
    Eigen::Vector3f w = Eigen::Vector3f::Zero();  // angular velocity (rad/s)
};

struct Interval {
    std::vector<imu::Measurement> steps;  // empty: no preintegration for this frame
    std::size_t consumed = 0;             // samples [0, consumed) of the queue are done with (pop them)
};

// Index of the last sample at or before t (0 if none)
inline std::size_t lastAtOrBefore(const std::vector<Sample>& s, double t, std::size_t from = 0) {
    std::size_t k = from;
    while (k + 1 < s.size() && s[k + 1].t <= t) ++k;
    return k;
}

// samples: the queue, ascending in t (strictly), from its front
inline Interval intervalMeasurements(const std::vector<Sample>& s, double t_prev, double t_cur) {
    Interval out;
    if (s.empty() || !(t_cur > t_prev)) return out;
    const std::size_t i0 = lastAtOrBefore(s, t_prev);
    std::size_t i1 = i0;
    while (i1 + 1 < s.size() && s[i1].t < t_cur) ++i1;
    out.consumed = lastAtOrBefore(s, t_cur, i0);
    const std::size_t n = i1 - i0;
    if (n == 0) return out;

    // the step's sample pair linearly at time x (exact at the samples themselves)
    const auto at = [](const Sample& s0, const Sample& s1, double x, Eigen::Vector3f& a, Eigen::Vector3f& w) {
        if (x == s0.t) {
            a = s0.a, w = s0.w;
        } else if (x == s1.t) {
            a = s1.a, w = s1.w;
        } else {
            const float r = float((x - s0.t) / (s1.t - s0.t));
            a = s0.a + (s1.a - s0.a) * r;
            w = s0.w + (s1.w - s0.w) * r;
        }
    };
    out.steps.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Sample& s0 = s[i0 + i];
        const Sample& s1 = s[i0 + i + 1];
        const double ta = i == 0 ? t_prev : s0.t;
        const double tb = i == n - 1 ? t_cur : s1.t;
        Eigen::Vector3f aa, wa, ab, wb;
        at(s0, s1, ta, aa, wa);
        at(s0, s1, tb, ab, wb);
        imu::Measurement m;
        m.a = (aa + ab) * 0.5f;
        m.w = (wa + wb) * 0.5f;
        m.dt = float(tb - ta);
        out.steps.push_back(m);
    }
    return out;
}

}  // namespace imu_preintegration
