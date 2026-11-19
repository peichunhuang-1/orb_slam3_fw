#include "algorithms/orb_matcher/stereo_matcher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>   // cross()

#include "algorithms/orb_matcher/core/bearing.hpp"
#include "algorithms/orb_matcher/core/epipolar_index.hpp"
#include "types/orb/hamming.hpp"
#include "algorithms/orb_matcher/core/subpixel.hpp"
#include "algorithms/orb_matcher/core/triangulate.hpp"

namespace orb_matcher {

struct StereoMatcher::Impl {
    camera::StereoRig rig;
    StereoMatcherParams p;
    Eigen::Matrix3f R_rL, R_rR;   // left / right camera -> rectification frame
    Eigen::Matrix3f R_21;         // left -> right camera
    Eigen::Vector3f b1, b2;       // baseline direction (C1 -> C2) in the left / right camera frame
    float B = 0, sinPhiMin = 0, minParallax = 0;   // minParallax: radians

    BearingSet bl, br;
    EpipolarIndex index;
    std::vector<int> candR, candD;   // per left: tentative right keypoint and its Hamming distance
    std::vector<int> bestD, bestL;   // per right: the closest left keypoint claiming it
    SubpixelRefiner sub;

    // Image direction of the epipolar curve at bearing f: the tangent of the great circle through
    // f and the baseline direction b (camera frame), towards b, projected.
    static bool epipolarDirection(const camera::KannalaBrandt8& cam, const Eigen::Vector3f& f, const Eigen::Vector3f& b,
                                  Eigen::Vector2f& dir) {
        Eigen::Vector3f t = b - b.dot(f) * f;
        if (t.squaredNorm() < 1e-12f) return false;
        t.normalize();
        const float d = 1e-3f;
        Eigen::Vector2f p, q;
        if (!cam.project(f * std::cos(d) + t * std::sin(d), p) || !cam.project(f * std::cos(d) - t * std::sin(d), q))
            return false;
        dir = p - q;
        if (dir.squaredNorm() < 1e-12f) return false;
        dir.normalize();
        return true;
    }

    // Moves the right pixel uvR (and its bearing fR, right camera frame) onto the left keypoint's
    // epipolar curve (θ_R := θ_L, φ_R kept) and sets up the search along it, on the left keypoint's
    // level (patches aligned with the epipolar directions of their image). Starting on the curve
    // matters: an offset across it would be partly traded for an offset along it by oblique texture
    // (aperture problem). The search itself: cpuRefine or a SubpixelBackend.
    bool prepareRight(const cv::KeyPoint& kl, const Eigen::Vector3f& fL, float thetaL, const std::vector<float>& scale,
                      Eigen::Vector2f& uvR, Eigen::Vector3f& fR, SubpixelJob& job) const {
        const int lvl = kl.octave;
        if (lvl < 0 || lvl >= int(scale.size())) return false;
        const camera::KannalaBrandt8& camR = rig.camera(1);
        {
            const Eigen::Vector3f pr = R_rR * fR;   // rectification frame
            const float sinPhi = std::sqrt(pr.y() * pr.y() + pr.z() * pr.z());
            const Eigen::Vector3f onCurve(pr.x(), sinPhi * std::sin(thetaL), sinPhi * std::cos(thetaL));
            fR = R_rR.transpose() * onCurve;
            if (!camR.project(fR, uvR)) return false;
        }
        Eigen::Vector2f dirL, dirR;
        if (!epipolarDirection(rig.camera(0), fL, b1, dirL) || !epipolarDirection(camR, fR, b2, dirR)) return false;
        const float s = scale[lvl];
        job.level = lvl;
        job.left = Eigen::Vector2f(kl.pt.x, kl.pt.y) / s;
        job.dirL = dirL;
        job.right = uvR / s;
        job.dirR = dirR;
        return true;
    }

    // The search of one job on the CPU, on the views' pyramids.
    bool cpuRefine(const SubpixelJob& j, const ViewFeatures& L, const ViewFeatures& R, float& off) {
        const int lvl = j.level;
        if (lvl >= L.pyramid_levels || lvl >= R.pyramid_levels) return false;
        const cv::Mat& imL = L.pyramid[lvl];
        const cv::Mat& imR = R.pyramid[lvl];
        if (imL.type() != CV_8UC1 || imR.type() != CV_8UC1) return false;
        return sub.refine(imL, j.left, j.dirL, imR, j.right, j.dirR, off);
    }

    // Sub-pixel jobs of one match() call, in the order they were prepared
    struct Pending {
        int i, r;
        Eigen::Vector2f uvR;   // right pixel on the left keypoint's epipolar curve
    };
    std::vector<SubpixelJob> jobs;
    std::vector<Pending> pending;
    std::vector<float> offsets;
    std::vector<uint8_t> refined;

    void reserve(int n) {
        bl.reserve(n);
        br.reserve(n);
        index.reserve(n);
        candR.reserve(n);
        candD.reserve(n);
        bestD.reserve(n);
        bestL.reserve(n);
        jobs.reserve(n);
        pending.reserve(n);
        offsets.reserve(n);
        refined.reserve(n);
    }

    // Disparity angle of a point at `range` from C1 seen at angle phiL to the baseline.
    float gammaAt(float range, float phiL) const {
        return std::atan2(range * std::sin(phiL), range * std::cos(phiL) - B) - phiL;
    }
};

StereoMatcher::StereoMatcher(const camera::StereoRig& rig, const StereoMatcherParams& params, int max_features)
    : impl_(std::make_unique<Impl>()) {
    const StereoMatcherParams& p = params;
    if (p.th_dist < 0 || p.th_dist > 256) throw std::invalid_argument("StereoMatcher: th_dist must be in [0, 256]");
    if (!(p.ratio > 0 && p.ratio <= 1)) throw std::invalid_argument("StereoMatcher: ratio must be in (0, 1]");
    if (p.octave_window < 0) throw std::invalid_argument("StereoMatcher: octave_window must be >= 0");
    if (!(p.sigma_px > 0) || !(p.gate_k > 0) || !(p.chi2 > 0))
        throw std::invalid_argument("StereoMatcher: sigma_px, gate_k and chi2 must be > 0");
    if (p.min_depth < 0 || p.max_depth < 0 || (p.max_depth > 0 && p.max_depth <= p.min_depth))
        throw std::invalid_argument("StereoMatcher: need 0 <= min_depth < max_depth (or max_depth = 0)");
    if (!(p.min_epipole_deg >= 0 && p.min_epipole_deg < 90))
        throw std::invalid_argument("StereoMatcher: min_epipole_deg must be in [0, 90)");
    if (!(p.min_parallax_deg >= 0 && p.min_parallax_deg < 90))
        throw std::invalid_argument("StereoMatcher: min_parallax_deg must be in [0, 90)");
    if (max_features < 0) throw std::invalid_argument("StereoMatcher: max_features must be >= 0");
    if (p.subpixel_half_window < 1 || p.subpixel_range < 1)
        throw std::invalid_argument("StereoMatcher: subpixel_half_window and subpixel_range must be >= 1");

    Impl& m = *impl_;
    m.rig = rig;
    m.p = p;
    m.B = rig.baseline();
    m.sinPhiMin = std::sin(p.min_epipole_deg * float(M_PI) / 180);
    m.minParallax = p.min_parallax_deg * float(M_PI) / 180;
    // e_x along the baseline, e_z = the left optical axis made orthogonal to it, e_y = e_z × e_x
    const Eigen::Vector3f ex = rig.t_12() / m.B;
    Eigen::Vector3f ez = Eigen::Vector3f::UnitZ() - ex.z() * ex;
    if (ez.norm() < 1e-3f) throw std::invalid_argument("StereoMatcher: baseline along the optical axis");
    ez.normalize();
    const Eigen::Vector3f ey = ez.cross(ex);
    m.R_rL.row(0) = ex;
    m.R_rL.row(1) = ey;
    m.R_rL.row(2) = ez;
    m.R_rR = m.R_rL * rig.R_12();
    m.R_21 = rig.R_12().transpose();
    m.b1 = ex;
    m.b2 = m.R_21 * ex;
    m.sub.configure(p.subpixel_half_window, p.subpixel_range, p.subpixel_fit);
    m.reserve(max_features);
}

StereoMatcher::~StereoMatcher() = default;

const camera::StereoRig& StereoMatcher::rig() const { return impl_->rig; }
const StereoMatcherParams& StereoMatcher::params() const { return impl_->p; }

void StereoMatcher::check_image_size(int width, int height) const {
    const camera::StereoRig& r = impl_->rig;
    if (width != r.width() || height != r.height())
        throw std::invalid_argument("StereoMatcher: images are " + std::to_string(width) + "x" + std::to_string(height) +
                                    ", the rig is calibrated for " + std::to_string(r.width()) + "x" +
                                    std::to_string(r.height()) + " (camera::StereoRig::resized)");
}

void StereoMatcher::match(const ViewFeatures& left, const ViewFeatures& right, const std::vector<float>& scale_factors,
                          StereoMatches& out, SubpixelBackend* backend) {
    const auto t0 = std::chrono::steady_clock::now();
    Impl& m = *impl_;
    const StereoMatcherParams& p = m.p;
    const int nl = left.n, nr = right.n;
    if ((nl > 0 && (!left.keypoints || !left.descriptors)) || (nr > 0 && (!right.keypoints || !right.descriptors)))
        throw std::invalid_argument("StereoMatcher::match: keypoints without descriptors");
    if (p.subpixel && !backend && ((nl > 0 && !left.pyramid) || (nr > 0 && !right.pyramid)))
        throw std::invalid_argument("StereoMatcher::match: subpixel needs the image pyramids (orb_extractor keep_pyramid)");

    StereoMatchStats st;
    computeBearings(m.rig.camera(0), m.R_rL, left.keypoints, nl, scale_factors, p.sigma_px, m.sinPhiMin, m.bl);
    computeBearings(m.rig.camera(1), m.R_rR, right.keypoints, nr, scale_factors, p.sigma_px, m.sinPhiMin, m.br);
    st.left = m.bl.count;
    st.right = m.br.count;
    m.index.build(m.br, right.descriptors, right.stride);
    const EpipolarIndex& ix = m.index;

    const float k = p.gate_k, k2 = k * k;
    m.candR.assign(nl, -1);
    m.candD.assign(nl, 256);

    // 1. per left keypoint: best and second best right keypoint inside the gates
    for (int i = 0; i < nl; ++i) {
        if (!m.bl.valid[i]) continue;
        const float thL = m.bl.theta[i], phiL = m.bl.phi[i], sL = m.bl.sigma[i], stL = m.bl.sigma_theta[i];
        const int octL = left.keypoints[i].octave;
        const float window = k * std::sqrt(stL * stL + ix.max_sigma_theta * ix.max_sigma_theta);
        const float gMax = p.min_depth > 0 ? m.gammaAt(p.min_depth, phiL) : float(M_PI);
        const float gMin = p.max_depth > 0 ? m.gammaAt(p.max_depth, phiL) : 0.f;
        const uint8_t* dL = left.descriptors + size_t(i) * left.stride;
        int best = 256, second = 256, bestK = -1;
        for (int j = ix.lower_bound(thL - window); j < ix.size && ix.theta[j] <= thL + window; ++j) {
            ++st.candidates;
            if (std::abs(ix.octave[j] - octL) > p.octave_window) continue;
            const float dTheta = ix.theta[j] - thL;
            if (dTheta * dTheta > k2 * (stL * stL + ix.sigma_theta[j] * ix.sigma_theta[j])) continue;
            const float eps = k * std::sqrt(sL * sL + ix.sigma[j] * ix.sigma[j]);
            const float gamma = ix.phi[j] - phiL;
            if (gamma < gMin - eps || gamma > gMax + eps) continue;
            ++st.hamming_evals;
            const int d = orb::hamming256(dL, &ix.desc[size_t(j) * 32]);
            if (d < best) {
                second = best;
                best = d;
                bestK = j;
            } else if (d < second) {
                second = d;
            }
        }
        if (bestK < 0 || best > p.th_dist) { ++st.rejected_dist; continue; }
        if (second < 256 && float(best) >= p.ratio * float(second)) { ++st.rejected_ratio; continue; }
        m.candR[i] = ix.index[bestK];
        m.candD[i] = best;
    }

    // 2. one to one: each right keypoint keeps its closest left keypoint (the first on ties)
    m.bestD.assign(nr, 257);
    m.bestL.assign(nr, -1);
    for (int i = 0; i < nl; ++i) {
        const int r = m.candR[i];
        if (r >= 0 && m.candD[i] < m.bestD[r]) {
            m.bestD[r] = m.candD[i];
            m.bestL[r] = i;
        }
    }

    // 3. triangulate and check the reprojection in both images
    out.left_to_right.assign(nl, -1);
    out.right_to_left.assign(nr, -1);
    out.points.resize(nl);
    out.depth.assign(nl, -1.f);
    out.distance.assign(nl, -1);
    out.right_uv.resize(nl);
    out.bearing[0].assign(m.bl.f.begin(), m.bl.f.end());
    out.bearing[1].assign(m.br.f.begin(), m.br.f.end());
    const camera::KannalaBrandt8& camL = m.rig.camera(0);
    const camera::KannalaBrandt8& camR = m.rig.camera(1);
    const int top = int(scale_factors.size()) - 1;
    auto sigma2 = [&](int octave) {
        const float s = p.sigma_px * (top >= 0 ? scale_factors[std::clamp(octave, 0, top)] : 1.f);
        return s * s;
    };
    // too little parallax for a reliable depth (checked last, on the final γ, so a far true match
    // becomes a monocular point instead of letting a nearer wrong candidate win)
    auto accept = [&](int i, int r, const Eigen::Vector2f& uvR, float phiR, const Eigen::Vector3f& P) {
        if (phiR - m.bl.phi[i] < m.minParallax) { ++st.rejected_parallax; return; }
        out.right_uv[i] = uvR;
        out.left_to_right[i] = r;
        out.right_to_left[r] = i;
        out.points[i] = P;
        out.depth[i] = P.z();
        out.distance[i] = m.candD[i];
        ++st.matched;
    };
    m.jobs.clear();
    m.pending.clear();
    for (int i = 0; i < nl; ++i) {
        const int r = m.candR[i];
        if (r < 0) continue;
        if (m.bestL[r] != i) { ++st.rejected_unique; continue; }
        const cv::KeyPoint& kl = left.keypoints[i];
        const cv::KeyPoint& kr = right.keypoints[r];
        Eigen::Vector2f uvRk(kr.pt.x, kr.pt.y);
        const Eigen::Vector3f fR = m.R_rR * m.br.f[r];
        float phiR = std::acos(std::clamp(fR.x(), -1.f, 1.f));
        Eigen::Vector3f P;
        if (!triangulateClosedForm(m.B, m.bl.phi[i], phiR, m.bl.f[i], P)) {
            ++st.rejected_parallax;
            continue;
        }
        // reprojection test on the keypoints as detected (the refinement moves the right one onto
        // the epipolar curve, which would hide an offset across it)
        Eigen::Vector2f uvL, uvR;
        const Eigen::Vector3f P2 = m.R_21 * (P - m.rig.t_12());
        if (!camL.project(P, uvL) || !camR.project(P2, uvR) ||
            (uvL - Eigen::Vector2f(kl.pt.x, kl.pt.y)).squaredNorm() > p.chi2 * sigma2(kl.octave) ||
            (uvR - uvRk).squaredNorm() > p.chi2 * sigma2(kr.octave)) {
            ++st.rejected_reproj;
            continue;
        }
        if (p.subpixel) {   // prepared here, searched for all matches at once below
            const auto ts = std::chrono::steady_clock::now();
            Eigen::Vector3f fRcam = m.br.f[r];
            SubpixelJob job;
            const bool ok = m.prepareRight(kl, m.bl.f[i], m.bl.theta[i], scale_factors, uvRk, fRcam, job);
            st.subpixel_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ts).count();
            if (!ok) { ++st.rejected_subpixel; continue; }
            m.jobs.push_back(job);
            m.pending.push_back({i, r, uvRk});
            continue;
        }
        accept(i, r, uvRk, phiR, P);
    }

    // 4. sub-pixel search along the epipolar lines, then triangulate again
    if (!m.jobs.empty()) {
        const auto ts = std::chrono::steady_clock::now();
        const int nj = int(m.jobs.size());
        m.offsets.resize(nj);
        m.refined.resize(nj);
        if (backend) {
            backend->refine(m.jobs.data(), nj, m.offsets.data(), m.refined.data());
        } else {
            for (int k = 0; k < nj; ++k) m.refined[k] = m.cpuRefine(m.jobs[k], left, right, m.offsets[k]);
        }
        for (int k = 0; k < nj; ++k) {
            const Impl::Pending& q = m.pending[k];
            const SubpixelJob& j = m.jobs[k];
            Eigen::Vector3f P, f;
            float phiR = 0;
            bool ok = m.refined[k];
            Eigen::Vector2f uv;
            if (ok) {
                uv = q.uvR + m.offsets[k] * scale_factors[j.level] * j.dirR;
                ok = camR.unproject(uv, f);
            }
            if (ok) {
                const Eigen::Vector3f fRr = m.R_rR * f;
                phiR = std::acos(std::clamp(fRr.x(), -1.f, 1.f));
                ok = triangulateClosedForm(m.B, m.bl.phi[q.i], phiR, m.bl.f[q.i], P);
            }
            if (!ok) { ++st.rejected_subpixel; continue; }
            accept(q.i, q.r, uv, phiR, P);
        }
        st.subpixel_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ts).count();
    }
    st.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    out.stats = st;
}

}  // namespace orb_matcher
