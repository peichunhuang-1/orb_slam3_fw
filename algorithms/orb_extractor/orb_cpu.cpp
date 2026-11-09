// CPU ORB extraction, see orb_cpu.hpp. Compiled with -ffp-contract=off (CMakeLists.txt).
#include "orb_cpu.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

#include "orb_pattern.hpp"

namespace orb_cpu {

namespace {

constexpr int kPatchSize = 31;   // rBRIEF patch; keypoint size = kPatchSize * level scale
constexpr int kHalfPatch = 15;   // orientation patch radius
constexpr int kMaxRank = 7;      // rank within a grid cell is capped
const int K7[7] = {18, 34, 49, 54, 49, 34, 18};   // 7x7 Gaussian, sum 256 per pass
const int CIRCLE[16][2] = {{0, 3},  {1, 3},   {2, 2},   {3, 1},   {3, 0},   {3, -1}, {2, -2}, {1, -3},
                           {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}};

// 9 contiguous set bits in a 16-bit circular mask.
inline bool has9(uint32_t m) {
    uint32_t m32 = m | (m << 16), r = m32;
    for (int k = 1; k < 9; ++k) r &= (m32 >> k);
    return (r & 0xFFFFu) != 0;
}

// umax: half-width of each row of the radius-15 circular patch (same algorithm as OpenCV).
struct Umax {
    int v[kHalfPatch + 2];
    Umax() {
        const int HP = kHalfPatch;
        int vmax = (int)std::floor(HP * std::sqrt(2.0) / 2 + 1);
        int vmin = (int)std::ceil(HP * std::sqrt(2.0) / 2);
        for (int i = 0; i <= vmax; ++i) v[i] = (int)std::lround(std::sqrt((double)HP * HP - i * i));
        for (int i = HP, v0 = 0; i >= vmin; --i) {   // make the circle symmetric
            while (v[v0] == v[v0 + 1]) ++v0;
            v[i] = v0;
            ++v0;
        }
    }
};
const Umax& umax() {
    static const Umax u;
    return u;
}

void run(const ParallelFor& parallel, int n, const std::function<void(int)>& fn) {
    if (n <= 0) return;
    if (parallel && n > 1) parallel(0, n, fn);
    else
        for (int i = 0; i < n; ++i) fn(i);
}

struct Corner {
    int x, y;
    bool strong;
};

struct Candidate {
    int level, px, py;
    float response;
};

// Bilinear downscale with the source coordinate (2x + 1) * src / (2 * dst) - 0.5 as an exact
// fraction and 11-bit weights.
void downscale(const cv::Mat& src, cv::Mat& dst) {
    const int sw = src.cols, sh = src.rows, w = dst.cols, h = dst.rows;
    std::vector<int> x0s(w), x1s(w), fxs(w);
    for (int x = 0; x < w; ++x) {
        int qx = (2 * x + 1) * sw - w;
        int x0 = qx / (2 * w);
        fxs[x] = ((qx - x0 * 2 * w) * 1024) / w;
        x0s[x] = x0;
        x1s[x] = std::min(x0 + 1, sw - 1);
    }
    for (int y = 0; y < h; ++y) {
        int qy = (2 * y + 1) * sh - h;
        int y0 = qy / (2 * h), fy = ((qy - y0 * 2 * h) * 1024) / h;
        int y1 = std::min(y0 + 1, sh - 1);
        const uint8_t* r0 = src.ptr<uint8_t>(y0);
        const uint8_t* r1 = src.ptr<uint8_t>(y1);
        uint8_t* d = dst.ptr<uint8_t>(y);
        for (int x = 0; x < w; ++x) {
            const uint32_t fx = (uint32_t)fxs[x];
            uint32_t top = r0[x0s[x]] * (2048 - fx) + r0[x1s[x]] * fx;
            uint32_t bot = r1[x0s[x]] * (2048 - fx) + r1[x1s[x]] * fx;
            uint32_t v = top * (2048 - (uint32_t)fy) + bot * (uint32_t)fy;
            d[x] = (uint8_t)((v + (1u << 21)) >> 22);
        }
    }
}

// 7x7 separable Gaussian, clamp-to-edge.
void gaussian7(const cv::Mat& img, cv::Mat& out) {
    const int w = img.cols, h = img.rows;
    std::vector<uint32_t> tmp((size_t)w * h);
    for (int y = 0; y < h; ++y) {
        const uint8_t* r = img.ptr<uint8_t>(y);
        uint32_t* t = &tmp[(size_t)y * w];
        for (int x = 0; x < w; ++x) {
            uint32_t s = 0;
            if (x >= 3 && x < w - 3)
                for (int i = 0; i < 7; ++i) s += K7[i] * r[x + i - 3];
            else
                for (int i = 0; i < 7; ++i) s += K7[i] * r[std::clamp(x + i - 3, 0, w - 1)];
            t[x] = s;
        }
    }
    out.create(h, w, CV_8UC1);
    for (int y = 0; y < h; ++y) {
        const uint32_t* rows[7];
        for (int i = 0; i < 7; ++i) rows[i] = &tmp[(size_t)std::clamp(y + i - 3, 0, h - 1) * w];
        uint8_t* o = out.ptr<uint8_t>(y);
        for (int x = 0; x < w; ++x) {
            uint32_t s = 0;
            for (int i = 0; i < 7; ++i) s += K7[i] * rows[i][x];
            o[x] = (uint8_t)((s + 32768u) >> 16);
        }
    }
}

}  // namespace

bool Config::operator==(const Config& o) const {
    return nfeatures == o.nfeatures && scale_factor == o.scale_factor && nlevels == o.nlevels &&
           ini_th_fast == o.ini_th_fast && min_th_fast == o.min_th_fast &&
           per_cell_threshold == o.per_cell_threshold && distribute == o.distribute &&
           cells_per_feature == o.cells_per_feature && abort_corner_fraction == o.abort_corner_fraction &&
           edge_threshold == o.edge_threshold && harris_k == o.harris_k;
}

std::vector<int> Extractor::level_quotas(int nfeatures, float scale_factor, int nlevels) {
    std::vector<int> q(nlevels);
    double factor = 1.0 / scale_factor;
    double n = nfeatures * (1 - factor) / (1 - std::pow(factor, nlevels));
    int sum = 0;
    for (int l = 0; l < nlevels - 1; ++l) {
        q[l] = (int)std::lround(n);
        sum += q[l];
        n *= factor;
    }
    q[nlevels - 1] = std::max(nfeatures - sum, 0);
    return q;
}

Extractor::Extractor(const Config& cfg) : cfg_(cfg) {
    auto bad = [](const std::string& m) { throw std::invalid_argument("orb_cpu::Extractor: " + m); };
    if (cfg_.nfeatures < 1) bad("nfeatures must be >= 1");
    if (cfg_.nlevels < 1 || cfg_.nlevels > 16) bad("nlevels must be 1..16");
    if (!(cfg_.scale_factor > 1.f)) bad("scale_factor must be > 1");
    if (cfg_.ini_th_fast < 1 || cfg_.ini_th_fast > 254) bad("ini_th_fast must be 1..254");
    if (cfg_.min_th_fast < 1 || cfg_.min_th_fast > cfg_.ini_th_fast) bad("need 1 <= min_th_fast <= ini_th_fast");
    if (cfg_.edge_threshold < 19) bad("edge_threshold must be >= 19 (rotated BRIEF patch radius)");
    if (!(cfg_.cells_per_feature > 0.f)) bad("cells_per_feature must be > 0");
    if (!(cfg_.abort_corner_fraction >= 0.f)) bad("abort_corner_fraction must be >= 0");
    for (int l = 0; l < cfg_.nlevels; ++l) {   // as ORB-SLAM3: scaleFactor^l, sigma^2 = scale^2
        float s = std::pow(cfg_.scale_factor, (float)l);
        scale_.push_back(s);
        inv_scale_.push_back(1.f / s);
        sigma2_.push_back(s * s);
        inv_sigma2_.push_back(1.f / (s * s));
    }
}

void Extractor::setup(int width, int height) {
    if (width == w0_ && height == h0_ && !levels_.empty()) return;
    const int B = cfg_.edge_threshold;
    levels_.clear();
    pyramid_pixels_ = 0;
    for (int l = 0; l < cfg_.nlevels; ++l) {
        float s = std::pow(cfg_.scale_factor, (float)l);
        int w = l == 0 ? width : (int)std::lround(width / s);
        int h = l == 0 ? height : (int)std::lround(height / s);
        if (w < 2 * B + 1 || h < 2 * B + 1) break;   // later levels cannot hold a keypoint
        Level L;
        L.w = w;
        L.h = h;
        L.scale = s;
        levels_.push_back(L);
        pyramid_pixels_ += (long)w * h;
    }
    if (levels_.empty())
        throw std::invalid_argument("orb_cpu::Extractor: image " + std::to_string(width) + "x" + std::to_string(height) +
                                    " too small for edge_threshold " + std::to_string(B));
    quotas_ = level_quotas(cfg_.nfeatures, cfg_.scale_factor, (int)levels_.size());
    // Grid: per level about quota * cells_per_feature square cells inside the border (8..511 px).
    for (size_t l = 0; l < levels_.size(); ++l) {
        Level& L = levels_[l];
        int aw = L.w - 2 * B, ah = L.h - 2 * B;
        double cells = std::max(1.0, quotas_[l] * (double)cfg_.cells_per_feature);
        L.cell = std::clamp((int)std::lround(std::sqrt((double)aw * ah / cells)), 8, 511);
        L.gw = (aw + L.cell - 1) / L.cell;
        L.gh = (ah + L.cell - 1) / L.cell;
    }
    pyr_.assign(levels_.size(), cv::Mat());
    blur_.assign(levels_.size(), cv::Mat());
    for (size_t l = 0; l < levels_.size(); ++l) {
        pyr_[l].create(levels_[l].h, levels_[l].w, CV_8UC1);
        blur_[l].create(levels_[l].h, levels_[l].w, CV_8UC1);
    }
    w0_ = width;
    h0_ = height;
}

void Extractor::extract(const cv::Mat& gray, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                        const ParallelFor& parallel) {
    if (gray.empty() || gray.type() != CV_8UC1)
        throw std::invalid_argument("orb_cpu::Extractor: need a non-empty 8-bit single-channel image");
    setup(gray.cols, gray.rows);
    info_ = FrameInfo{};
    keypoints.clear();
    descriptors.release();

    const int nl = (int)levels_.size();
    const int B = cfg_.edge_threshold;
    const int tHi = cfg_.ini_th_fast;
    const int tLo = cfg_.per_cell_threshold ? std::min(cfg_.min_th_fast, tHi) : tHi;
    const bool perCell = tLo < tHi;
    auto cellOf = [&](const Level& L, int x, int y) {
        int cx = std::min((x - B) / L.cell, L.gw - 1), cy = std::min((y - B) / L.cell, L.gh - 1);
        return cy * L.gw + cx;
    };

    // 1. pyramid (sequential: each level comes from the previous one). A level still held by someone
    // else (the previous frame's pyramid() handed downstream) is replaced by a new image, not overwritten.
    for (cv::Mat& m : pyr_)
        if (m.u && CV_XADD(&m.u->refcount, 0) > 1) m = cv::Mat(m.rows, m.cols, CV_8UC1);
    gray.copyTo(pyr_[0]);
    for (int l = 1; l < nl; ++l) downscale(pyr_[l - 1], pyr_[l]);

    // 2. FAST per level. Corners are tested at the low threshold; a cell with a corner of the high
    // threshold keeps only those (ORB-SLAM3's per-cell fallback, decided within the frame).
    std::vector<std::vector<Corner>> corners(nl);
    std::vector<int> weakCells(nl, 0), kept(nl, 0);
    run(parallel, nl, [&](int l) {
        const Level& L = levels_[l];
        const cv::Mat& img = pyr_[l];
        const int step = (int)img.step;
        int off[16];
        for (int i = 0; i < 16; ++i) off[i] = CIRCLE[i][1] * step + CIRCLE[i][0];
        std::vector<char> cellStrong(perCell ? (size_t)L.gw * L.gh : 0, 0);
        auto& out = corners[l];
        for (int y = B; y < L.h - B; ++y) {
            const uint8_t* row = img.ptr<uint8_t>(y);
            for (int x = B; x < L.w - B; ++x) {
                const uint8_t* p = row + x;
                const int c = p[0], hi = c + tLo, lo = c - tLo;
                // An arc of 9 of 16 covers at least two of the pixels 0, 4, 8, 12.
                int nb = 0, nd = 0;
                for (int i = 0; i < 16; i += 4) {
                    int v = p[off[i]];
                    nb += v > hi;
                    nd += v < lo;
                }
                if (nb < 2 && nd < 2) continue;
                uint32_t br = 0, dk = 0;
                int vals[16];
                for (int i = 0; i < 16; ++i) {
                    int v = p[off[i]];
                    vals[i] = v;
                    if (v > hi) br |= 1u << i;
                    if (v < lo) dk |= 1u << i;
                }
                if (!has9(br) && !has9(dk)) continue;
                bool strong = true;
                if (perCell) {
                    const int hi2 = c + tHi, lo2 = c - tHi;
                    uint32_t b2 = 0, d2 = 0;
                    for (int i = 0; i < 16; ++i) {
                        if (vals[i] > hi2) b2 |= 1u << i;
                        if (vals[i] < lo2) d2 |= 1u << i;
                    }
                    strong = has9(b2) || has9(d2);
                    if (strong) cellStrong[cellOf(L, x, y)] = 1;
                }
                out.push_back({x, y, strong});
            }
        }
        // Drop the weak corners of cells that have a strong one.
        if (perCell) {
            size_t n = 0;
            for (const Corner& k : out)
                if (k.strong || !cellStrong[cellOf(L, k.x, k.y)]) out[n++] = k;
            out.resize(n);
            int weak = 0;
            for (char s : cellStrong) weak += !s;
            weakCells[l] = weak;
        }
        kept[l] = (int)out.size();
    });
    for (int l = 0; l < nl; ++l) {
        info_.corners += kept[l];
        info_.weak_cells += weakCells[l];
    }

    // Abnormal frame: far more corners than any real scene (sensor noise, gravel): no keypoints.
    if (cfg_.abort_corner_fraction > 0.f) {
        const long limit = std::max(1L, (long)(cfg_.abort_corner_fraction * pyramid_pixels_));
        if (info_.corners > limit) {
            info_.aborted = true;
            return;
        }
    }

    // 3. blur (for BRIEF), Harris response of the corners, 3x3 NMS
    const float hs = 1.0f / (4.0f * 7.0f * 255.0f);
    const float hs4 = hs * hs * hs * hs;
    std::vector<std::vector<Candidate>> cands(nl);
    run(parallel, nl, [&](int l) {
        const Level& L = levels_[l];
        const cv::Mat& img = pyr_[l];
        gaussian7(img, blur_[l]);
        const int w = L.w;
        auto I = [&](int x, int y) { return (int)img.ptr<uint8_t>(y)[x]; };
        std::vector<float> score((size_t)w * L.h, 0.f);
        for (const Corner& k : corners[l]) {
            int a = 0, b = 0, cc = 0;
            for (int dy = -3; dy <= 3; ++dy)
                for (int dx = -3; dx <= 3; ++dx) {
                    int X = k.x + dx, Y = k.y + dy;
                    int ix = (I(X + 1, Y - 1) + 2 * I(X + 1, Y) + I(X + 1, Y + 1)) -
                             (I(X - 1, Y - 1) + 2 * I(X - 1, Y) + I(X - 1, Y + 1));
                    int iy = (I(X - 1, Y + 1) + 2 * I(X, Y + 1) + I(X + 1, Y + 1)) -
                             (I(X - 1, Y - 1) + 2 * I(X, Y - 1) + I(X + 1, Y - 1));
                    a += ix * ix;
                    b += iy * iy;
                    cc += ix * iy;
                }
            float fa = (float)a, fb = (float)b, fc = (float)cc;
            float tr = fa + fb;
            float r = (fa * fb - fc * fc - cfg_.harris_k * tr * tr) * hs4;
            score[(size_t)k.y * w + k.x] = r > 0.f ? r : 0.f;
        }
        // Keep a corner unless a neighbour is stronger; on a tie the one first in raster order wins.
        for (const Corner& k : corners[l]) {   // raster order
            const float s = score[(size_t)k.y * w + k.x];
            if (s <= 0.f) continue;
            bool keep = true;
            for (int dy = -1; dy <= 1 && keep; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    if (!dx && !dy) continue;
                    float n = score[(size_t)(k.y + dy) * w + k.x + dx];
                    bool before = dy < 0 || (dy == 0 && dx < 0);
                    if (before ? n >= s : n > s) {
                        keep = false;
                        break;
                    }
                }
            if (keep) cands[l].push_back({l, k.x, k.y, s});
        }
    });

    // 4. selection: per level its quota, by (rank in grid cell, response, y, x) with distribute,
    // else by (response, y, x)
    auto stronger = [](const Candidate& a, const Candidate& b) {
        if (a.response != b.response) return a.response > b.response;
        if (a.py != b.py) return a.py < b.py;
        return a.px < b.px;
    };
    std::vector<Candidate> sel;
    for (int l = 0; l < nl; ++l) {
        auto& v = cands[l];
        info_.candidates += (int)v.size();
        const int keep = std::min<int>(quotas_[l], (int)v.size());
        if (cfg_.distribute) {
            const Level& L = levels_[l];
            std::vector<std::vector<int>> cell((size_t)L.gw * L.gh);
            for (int i = 0; i < (int)v.size(); ++i) cell[cellOf(L, v[i].px, v[i].py)].push_back(i);
            std::vector<int> rank(v.size(), 0);
            for (auto& c : cell) {
                std::sort(c.begin(), c.end(), [&](int ia, int ib) { return stronger(v[ia], v[ib]); });
                for (int k = 0; k < (int)c.size(); ++k) rank[c[k]] = std::min(k, kMaxRank);
            }
            std::vector<int> idx(v.size());
            for (int i = 0; i < (int)v.size(); ++i) idx[i] = i;
            auto before = [&](int ia, int ib) {
                return rank[ia] != rank[ib] ? rank[ia] < rank[ib] : stronger(v[ia], v[ib]);
            };
            std::partial_sort(idx.begin(), idx.begin() + keep, idx.end(), before);
            for (int i = 0; i < keep; ++i) sel.push_back(v[idx[i]]);
        } else {
            std::partial_sort(v.begin(), v.begin() + keep, v.end(), stronger);
            sel.insert(sel.end(), v.begin(), v.begin() + keep);
        }
    }

    // 5. orientation (intensity centroid on the image) and rotated BRIEF (on the blurred image)
    const int n = (int)sel.size();
    info_.keypoints = n;
    if (n == 0) return;
    keypoints.resize(n);
    descriptors.create(n, 32, CV_8U);
    const int* um = umax().v;
    constexpr int kChunk = 64;
    run(parallel, (n + kChunk - 1) / kChunk, [&](int chunk) {
        const int end = std::min(n, (chunk + 1) * kChunk);
        for (int i = chunk * kChunk; i < end; ++i) {
            const Candidate& c = sel[i];
            const Level& L = levels_[c.level];
            const int step = (int)pyr_[c.level].step, bstep = (int)blur_[c.level].step;
            const uint8_t* base = pyr_[c.level].ptr<uint8_t>(c.py) + c.px;
            const uint8_t* bb = blur_[c.level].ptr<uint8_t>(c.py) + c.px;
            int m01 = 0, m10 = 0;
            for (int u = -kHalfPatch; u <= kHalfPatch; ++u) m10 += u * (int)base[u];
            for (int v = 1; v <= kHalfPatch; ++v) {
                int d = um[v], vs = 0;
                for (int u = -d; u <= d; ++u) {
                    int vp = (int)base[u + v * step], vm = (int)base[u - v * step];
                    vs += vp - vm;
                    m10 += u * (vp + vm);
                }
                m01 += v * vs;
            }
            const float ang = (m01 == 0 && m10 == 0) ? 0.f : std::atan2((float)m01, (float)m10);
            const float ca = std::cos(ang), sa = std::sin(ang);
            uint8_t* d = descriptors.ptr<uint8_t>(i);
            std::memset(d, 0, 32);
            for (int j = 0; j < 256; ++j) {   // bit j of byte j/8: cv::ORB layout
                const int* pt = &kOpenCvBitPattern31[j * 4];
                int x0 = (int)std::nearbyint(pt[0] * ca - pt[1] * sa), y0 = (int)std::nearbyint(pt[0] * sa + pt[1] * ca);
                int x1 = (int)std::nearbyint(pt[2] * ca - pt[3] * sa), y1 = (int)std::nearbyint(pt[2] * sa + pt[3] * ca);
                if (bb[y0 * bstep + x0] < bb[y1 * bstep + x1]) d[j >> 3] |= (uint8_t)(1u << (j & 7));
            }
            float deg = ang * 180.0f / (float)M_PI;
            keypoints[i] = cv::KeyPoint(c.px * L.scale, c.py * L.scale, kPatchSize * L.scale,
                                        deg < 0 ? deg + 360.0f : deg, c.response, c.level);
        }
    });
}

}  // namespace orb_cpu
