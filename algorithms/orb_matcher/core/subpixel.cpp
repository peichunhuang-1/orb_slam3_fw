#include "algorithms/orb_matcher/core/subpixel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

namespace orb_matcher {

namespace {

// rows x cols bilinear samples on a grid centred on `centre`: columns step along `dir`, rows along
// its normal (−dir.y, dir.x). False if any sample needs a pixel outside the image.
bool sampleGrid(const cv::Mat& img, const Eigen::Vector2f& centre, const Eigen::Vector2f& dir, int rows, int cols,
                int stride, float* out) {
    const Eigen::Vector2f nrm(-dir.y(), dir.x());
    const Eigen::Vector2f origin = centre - 0.5f * float(cols - 1) * dir - 0.5f * float(rows - 1) * nrm;
    // the grid is a parallelogram: its corners bound every sample
    const Eigen::Vector2f ec = float(cols - 1) * dir, er = float(rows - 1) * nrm;
    const float xs[4] = {origin.x(), origin.x() + ec.x(), origin.x() + er.x(), origin.x() + ec.x() + er.x()};
    const float ys[4] = {origin.y(), origin.y() + ec.y(), origin.y() + er.y(), origin.y() + ec.y() + er.y()};
    if (*std::min_element(xs, xs + 4) < 0 || *std::min_element(ys, ys + 4) < 0 ||
        *std::max_element(xs, xs + 4) >= img.cols - 1 || *std::max_element(ys, ys + 4) >= img.rows - 1)
        return false;
    const uint8_t* data = img.ptr<uint8_t>();
    const size_t step = img.step[0];
    for (int r = 0; r < rows; ++r) {
        float x = origin.x() + r * nrm.x(), y = origin.y() + r * nrm.y();
        float* o = out + r * stride;
        for (int c = 0; c < cols; ++c, x += dir.x(), y += dir.y()) {
            const int ix = int(x), iy = int(y);   // x, y >= 0: truncation = floor
            const float ax = x - ix, ay = y - iy;
            const uint8_t* p = data + size_t(iy) * step + ix;
            const float top = p[0] + ax * (p[1] - p[0]);
            const float bot = p[step] + ax * (p[step + 1] - p[step]);
            o[c] = top + ay * (bot - top);
        }
    }
    return true;
}

}  // namespace

void SubpixelRefiner::configure(int halfWindow, int range, SubpixelFit fit) {
    fit_ = fit;
    w_ = halfWindow;
    L_ = range;
    const int S = 2 * w_ + 1, SW = S + 2 * L_;
    strideL_ = (S + 3) / 4 * 4;
    strideR_ = (SW + 3) / 4 * 4 + 4;   // a window at the last offset reads up to 3 past SW
    patchL_.assign(size_t(S) * strideL_, 0.f);   // the padding stays 0
    strip_.assign(size_t(S) * strideR_, 0.f);
    cost_.resize(2 * L_ + 1);
}

bool SubpixelRefiner::refine(const cv::Mat& leftImg, const Eigen::Vector2f& left, const Eigen::Vector2f& dirL,
                             const cv::Mat& rightImg, const Eigen::Vector2f& right, const Eigen::Vector2f& dirR,
                             float& offset) {
    const int S = 2 * w_ + 1, SW = S + 2 * L_;
    if (!sampleGrid(leftImg, left, dirL, S, S, strideL_, patchL_.data())) return false;
    if (!sampleGrid(rightImg, right, dirR, S, SW, strideR_, strip_.data())) return false;
    // Patches minus their centre value (as ORB-SLAM3), compared window by window:
    // d = (a − cL) − (b − cR) = a − b + (cR − cL)
    const float cL = patchL_[w_ * strideL_ + w_];
    const bool ssd = fit_ == SubpixelFit::SsdParabola;
    int best = 0;
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    const int chunks = (S + 3) / 4;
    const int tail = S - 4 * (chunks - 1);   // valid lanes in the last chunk
    const float m[4] = {1.f, tail > 1 ? 1.f : 0.f, tail > 2 ? 1.f : 0.f, tail > 3 ? 1.f : 0.f};
    const float32x4_t lastMask = vld1q_f32(m);
    for (int k = 0; k <= 2 * L_; ++k) {
        const float32x4_t dc = vdupq_n_f32(strip_[w_ * strideR_ + k + w_] - cL);
        float32x4_t acc = vdupq_n_f32(0.f);
        for (int r = 0; r < S; ++r) {
            const float* a = &patchL_[r * strideL_];
            const float* b = &strip_[r * strideR_ + k];
            for (int j = 0; j < chunks; ++j) {
                float32x4_t d = vaddq_f32(vsubq_f32(vld1q_f32(a + 4 * j), vld1q_f32(b + 4 * j)), dc);
                if (j == chunks - 1) d = vmulq_f32(d, lastMask);
                acc = ssd ? vfmaq_f32(acc, d, d) : vaddq_f32(acc, vabsq_f32(d));
            }
        }
        cost_[k] = vaddvq_f32(acc);
        if (cost_[k] < cost_[best]) best = k;
    }
#else
    for (int k = 0; k <= 2 * L_; ++k) {
        const float dc = strip_[w_ * strideR_ + k + w_] - cL;
        float s = 0;
        for (int r = 0; r < S; ++r) {
            const float* a = &patchL_[r * strideL_];
            const float* b = &strip_[r * strideR_ + k];
            for (int c = 0; c < S; ++c) {
                const float d = a[c] - b[c] + dc;
                s += ssd ? d * d : std::abs(d);
            }
        }
        cost_[k] = s;
        if (cost_[k] < cost_[best]) best = k;
    }
#endif
    (void)SW;
    if (best == 0 || best == 2 * L_) return false;
    const float d1 = cost_[best - 1], d2 = cost_[best], d3 = cost_[best + 1];
    float delta;
    if (fit_ == SubpixelFit::SadV) {
        // two lines of equal and opposite slope through the minimum and its neighbours
        const float den = 2 * (std::max(d1, d3) - d2);
        if (!(den > 0)) return false;
        delta = (d1 - d3) / den;
    } else {
        const float den = 2 * (d1 + d3 - 2 * d2);
        if (!(den > 0)) return false;
        delta = (d1 - d3) / den;
    }
    if (delta < -1 || delta > 1) return false;
    offset = float(best - L_) + delta;
    return true;
}

}  // namespace orb_matcher
