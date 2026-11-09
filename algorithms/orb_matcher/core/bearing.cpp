#include "algorithms/orb_matcher/core/bearing.hpp"

#include <algorithm>
#include <cmath>

namespace orb_matcher {

void BearingSet::reserve(int n) {
    f.reserve(n);
    theta.reserve(n);
    phi.reserve(n);
    sigma.reserve(n);
    sigma_theta.reserve(n);
    octave.reserve(n);
    valid.reserve(n);
}

void computeBearings(const KannalaBrandt8& cam, const Eigen::Matrix3f& R_rc, const cv::KeyPoint* kp, int n,
                     const std::vector<float>& scale_factors, float sigma_px, float sin_phi_min, BearingSet& out) {
    out.f.resize(n);
    out.theta.resize(n);
    out.phi.resize(n);
    out.sigma.resize(n);
    out.sigma_theta.resize(n);
    out.octave.resize(n);
    out.valid.resize(n);
    out.count = 0;
    const int top = int(scale_factors.size()) - 1;
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3f f;
        out.octave[i] = kp[i].octave;
        if (!cam.unproject(Eigen::Vector2f(kp[i].pt.x, kp[i].pt.y), f)) {
            out.f[i].setZero();
            out.valid[i] = 0;
            continue;
        }
        out.f[i] = f;
        const Eigen::Vector3f p = R_rc * f;
        const float sinPhi = std::sqrt(p.y() * p.y() + p.z() * p.z());
        out.phi[i] = std::acos(std::clamp(p.x(), -1.f, 1.f));
        out.theta[i] = std::atan2(p.y(), p.z());
        const float scale = top >= 0 ? scale_factors[std::clamp(kp[i].octave, 0, top)] : 1.f;
        out.sigma[i] = sigma_px * scale / cam.pixels_per_radian(std::acos(std::clamp(f.z(), -1.f, 1.f)));
        out.sigma_theta[i] = out.sigma[i] / std::max(sinPhi, 1e-6f);
        out.valid[i] = sinPhi >= sin_phi_min;
        out.count += out.valid[i];
    }
}

}  // namespace orb_matcher
