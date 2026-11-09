#pragma once
// Closed-form triangulation in the epipolar plane (docs/ALGORITHM.md section 2).
#include <cmath>

#include <Eigen/Core>

namespace orb_matcher {

// Law of sines in the triangle (C1, C2, point): the point is r_L = B · sin φ_R / sin γ along the
// left bearing f_L, γ = φ_R − φ_L. False for γ <= 0 (parallel or diverging rays).
inline bool triangulateClosedForm(float baseline, float phiL, float phiR, const Eigen::Vector3f& fL,
                                  Eigen::Vector3f& point) {
    const float gamma = phiR - phiL;
    if (!(gamma > 1e-7f)) return false;
    point = fL * (baseline * std::sin(phiR) / std::sin(gamma));
    return true;
}

}  // namespace orb_matcher
