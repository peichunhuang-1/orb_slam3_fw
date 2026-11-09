// GPU ORB engine (Vulkan compute). Compiled only when CMake finds Vulkan and glslangValidator
// (ORB_HAVE_VULKAN); otherwise the orb_extractor block uses its CPU implementation.
#ifdef ORB_HAVE_VULKAN
#include "orb_vk.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include "algorithms/orb_extractor/orb_pattern.hpp"
#include "orb_vk_log.hpp"
#include "types/gpu/context.hpp"
#include "spv_blur.h"
#include "spv_describe.h"
#include "spv_fast.h"
#include "spv_fast_full.h"
#include "spv_cell_list.h"
#include "spv_fast_cells.h"
#include "spv_fast_mark.h"
#include "spv_grid_rank.h"
#include "spv_grid_top.h"
#include "spv_grid_scan.h"
#include "spv_grid_scatter.h"
#include "spv_harris.h"
#include "spv_nms.h"
#include "spv_post_nms.h"
#include "spv_rank_thresh.h"
#include "spv_reset.h"
#include "spv_resize.h"
#include "spv_resize_any.h"
#include "spv_select.h"
#include "spv_select_tie.h"
#include "spv_thresh.h"

// Debug hardware counters (CMake option ORB_PERF_COUNTERS): without it no counter code is compiled in.
#ifndef ORB_PERF_COUNTERS
#define ORB_PERF_COUNTERS 0
#endif

#define VK_CHECK(x)                                                                                    \
    do {                                                                                               \
        VkResult r_ = (x);                                                                             \
        if (r_ != VK_SUCCESS) {                                                                        \
            char m_[256];                                                                              \
            snprintf(m_, sizeof m_, "%s failed (VkResult %d) at %s:%d", #x, (int)r_, __FILE__, __LINE__); \
            throw std::runtime_error(m_);                                                              \
        }                                                                                              \
    } while (0)

namespace {

// Must match the push_constant block in shaders/common.glsl.
struct PushConsts {
    int32_t w, h, off, stride;
    int32_t srcW, srcH, srcOff, srcStride;
    int32_t xTab, yTab, level, fastThr;
    int32_t border, maxKp, maxCorners;
    float   harrisK, harrisScale4;
    int32_t imgStride, nlevels, selCap;
    int32_t gridCells, distribute, fastLo, fastPass2, abortCorners;
};

constexpr int kGpuKpStride = 8;    // sizeof(KP) / 4 in common.glsl
constexpr int kPatchSize = 31;     // rBRIEF patch; keypoint size = kPatchSize * level scale
constexpr uint32_t kFineQueries = 64;   // profile: per-kernel-group timestamps (lastKernelMs)
constexpr uint32_t kQueries = 5;   // timestamps (profile only): start, after FAST, after Harris, after selection, end
// Workgroup tiles in pixels of the image kernels (local size x pixels per thread in vk/tools/gen_shaders.py).
constexpr int kBlurTileW = 64, kBlurTileH = 16;   // blur.comp: 16 x 8 threads, 4 x 2 pixels each
constexpr int kFastTileW = 64, kFastTileH = 8;    // fast.comp: 16 x 8 threads, 4 x 1 pixels each
// Every row of the packed GPU pyramid has kRowPad bytes before pixel 0 and at least kRowPad
// after pixel w-1, holding copies of the edge pixels (written by resize.comp, and by
// copyLevel0 for level 0). blur.comp reads them instead of handling the row ends.
constexpr int kRowPad = 4;
constexpr int kResizeRows = 2;   // output rows per thread in resize.comp
constexpr int kConstLevelBase = 260;
constexpr int kConstQuotaBase = 276;
constexpr int kConstCellBase = 292;
constexpr int kConstCell2Base = 308;
constexpr int kMaxRank = 7;           // MAX_RANK in common.glsl
constexpr uint32_t kListGroups = 64;  // workgroups (of 64 threads) for the grid-stride list kernels
constexpr size_t kSelHeaderBytes = 16 * 8 + 16 * 64 * 4 + 16 * 2048 * 4;  // thr, histCoarse, histFine
constexpr size_t kGridHeaderBytes = 16 * 8 + 16 * 64 * 4;                 // rthr, rankHist
constexpr size_t kRankSlots = 8;   // > MAX_RANK (common.glsl): each cell's top list (grid_top.comp)
static_assert(kRankSlots > (size_t)kMaxRank, "each cell's top list must hold MAX_RANK entries");

uint32_t groups(int n, int g) { return (uint32_t)((n + g - 1) / g); }

// Copies an 8-bit image into level 0 of a packed pyramid (row stride `dstStride`, pixel 0 at
// dst) and fills the row padding with the edge pixels.
void copyLevel0(uint8_t* dst, int dstStride, const uint8_t* src, int srcStride, int w, int h) {
    for (int y = 0; y < h; ++y) {
        uint8_t* d = dst + (size_t)y * dstStride;
        const uint8_t* s = src + (size_t)y * srcStride;
        memcpy(d, s, w);
        memset(d - kRowPad, s[0], kRowPad);
        memset(d + w, s[w - 1], dstStride - kRowPad - w);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Constant tables and CPU-side selection
// ---------------------------------------------------------------------------
void OrbVk::buildConsts(std::vector<int32_t>& out, const std::vector<LevelInfo>& levels,
                        const std::vector<int>& quotas) {
    out.assign((kConstCell2Base + 16) * 4, 0);

    // rBRIEF sampling pairs: OpenCV's learned pattern, so descriptors match cv::ORB.
    for (int i = 0; i < 256 * 4; ++i) out[i] = kOpenCvBitPattern31[i];

    // umax: half-width of each row of the radius-15 circular patch (same algorithm as OpenCV).
    const int HP = 15;
    int umax[HP + 2];
    int vmax = (int)std::floor(HP * std::sqrt(2.0) / 2 + 1);
    int vmin = (int)std::ceil(HP * std::sqrt(2.0) / 2);
    for (int v = 0; v <= vmax; ++v) umax[v] = (int)std::lround(std::sqrt((double)HP * HP - v * v));
    for (int v = HP, v0 = 0; v >= vmin; --v) {  // make the circle symmetric
        while (umax[v0] == umax[v0 + 1]) ++v0;
        umax[v] = v0;
        ++v0;
    }
    for (int v = 0; v < 16; ++v) out[256 * 4 + v] = umax[v];

    for (size_t l = 0; l < levels.size(); ++l) {
        out[(kConstLevelBase + l) * 4 + 0] = levels[l].w;
        out[(kConstLevelBase + l) * 4 + 1] = levels[l].h;
        out[(kConstLevelBase + l) * 4 + 2] = levels[l].goff;
        out[(kConstLevelBase + l) * 4 + 3] = levels[l].stride;
        // multiplier for the division by the cell size (cellOf() in common.glsl)
        out[(kConstCellBase + l) * 4 + 0] = levels[l].cell ? ((1 << 22) + levels[l].cell - 1) / levels[l].cell : 0;
        out[(kConstCellBase + l) * 4 + 1] = levels[l].gw;
        out[(kConstCellBase + l) * 4 + 2] = levels[l].gh;
        out[(kConstCellBase + l) * 4 + 3] = levels[l].cellOff;
        out[(kConstCell2Base + l) * 4 + 0] = levels[l].cell;
    }
    for (size_t l = 0; l < quotas.size() && l < 16; ++l) out[(kConstQuotaBase + l) * 4] = quotas[l];
}

std::vector<int> OrbVk::levelQuotas(const OrbParams& p, int nlevels) {
    std::vector<int> q(nlevels);
    double factor = 1.0 / p.scaleFactor;
    double n = p.nfeatures * (1 - factor) / (1 - std::pow(factor, nlevels));
    int sum = 0;
    for (int l = 0; l < nlevels - 1; ++l) {
        q[l] = (int)std::lround(n);
        sum += q[l];
        n *= factor;
    }
    q[nlevels - 1] = std::max(p.nfeatures - sum, 0);
    return q;
}

std::vector<int> OrbVk::selectIndices(const std::vector<OrbCandidate>& cands, const OrbParams& p,
                                      const std::vector<LevelInfo>& levels) {
    const int nl = (int)levels.size();
    std::vector<std::vector<int>> per(nl);
    for (int i = 0; i < (int)cands.size(); ++i) per[cands[i].level].push_back(i);
    // GPU output order is nondeterministic; sort by (response, y, x) for reproducible results.
    auto stronger = [&](int ia, int ib) {
        const OrbCandidate &a = cands[ia], &b = cands[ib];
        if (a.response != b.response) return a.response > b.response;
        if (a.py != b.py) return a.py < b.py;
        return a.px < b.px;
    };

    // distribute: rank within the grid cell (capped like the GPU), then (rank, response, y, x).
    std::vector<int> rank;
    if (p.distribute) {
        rank.assign(cands.size(), 0);
        for (int l = 0; l < nl; ++l) {
            const LevelInfo& L = levels[l];
            std::vector<std::vector<int>> cell((size_t)L.gw * L.gh);
            for (int i : per[l]) {
                int cx = std::min((cands[i].px - p.edgeThreshold) / L.cell, L.gw - 1);
                int cy = std::min((cands[i].py - p.edgeThreshold) / L.cell, L.gh - 1);
                cell[(size_t)cy * L.gw + cx].push_back(i);
            }
            for (auto& c : cell) {
                std::sort(c.begin(), c.end(), stronger);
                for (int k = 0; k < (int)c.size(); ++k) rank[c[k]] = std::min(k, kMaxRank);
            }
        }
    }

    auto quota = levelQuotas(p, nl);
    std::vector<int> out;
    for (int l = 0; l < nl; ++l) {
        auto& v = per[l];
        if (p.distribute)
            std::sort(v.begin(), v.end(), [&](int ia, int ib) {
                return rank[ia] != rank[ib] ? rank[ia] < rank[ib] : stronger(ia, ib);
            });
        else
            std::sort(v.begin(), v.end(), stronger);
        int keep = std::min<int>(quota[l], (int)v.size());
        out.insert(out.end(), v.begin(), v.begin() + keep);
    }
    return out;
}

void OrbVk::selectFeatures(const std::vector<OrbCandidate>& cands, const OrbParams& p,
                           const std::vector<LevelInfo>& levels, std::vector<OrbKeypoint>& kps,
                           std::vector<uint8_t>& desc) {
    kps.clear();
    desc.clear();
    for (int idx : selectIndices(cands, p, levels)) {
        const OrbCandidate& c = cands[idx];
        const int l = c.level;
        float s = levels[l].scale;
        OrbKeypoint k;
        k.x = c.px * s;
        k.y = c.py * s;
        k.size = kPatchSize * s;
        float deg = c.angleRad * 180.0f / (float)M_PI;
        k.angle = deg < 0 ? deg + 360.0f : deg;
        k.response = c.response;
        k.octave = l;
        kps.push_back(k);
        const uint8_t* d = reinterpret_cast<const uint8_t*>(c.desc);  // little-endian byte order
        desc.insert(desc.end(), d, d + 32);
    }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
OrbVk::OrbVk(int width, int height, const OrbParams& p, bool allowCpuDevice, std::shared_ptr<gpu::Context> ctx)
    : p_(p), w0_(width), h0_(height) {
    if (width >= 8192 || height >= 8192) throw std::runtime_error("image dimension must be < 8192");
    if (p_.edgeThreshold < 19) throw std::runtime_error("edgeThreshold must be >= 19 (rotated BRIEF patch radius)");
    if (p_.maxFeatures <= 0) p_.maxFeatures = p_.nfeatures;
    if (p_.nfeatures < 1 || p_.nfeatures > p_.maxFeatures) throw std::runtime_error("need 1 <= nfeatures <= maxFeatures");
    if (p_.numImages < 1 || p_.numImages > 8 || (p_.numImages > 1 && !p_.pyramidOnly))
        throw std::runtime_error("numImages must be 1..8, and > 1 requires pyramidOnly");

    // Pyramid geometry
    size_t off = 0, goff = 0;
    auto addLevel = [&](int w, int h, float s) {
        int stride = kRowPad + ((w + 3) & ~3) + kRowPad;
        levels_.push_back({w, h, (int)off, s, stride, (int)goff + kRowPad});
        off += (size_t)w * h;
        goff += (size_t)stride * h;
    };
    for (int l = 0; l < p_.nlevels && l < 16; ++l) {
        float s = std::pow(p_.scaleFactor, (float)l);
        int w = l == 0 ? width : (int)std::lround(width / s);
        int h = l == 0 ? height : (int)std::lround(height / s);
        bool tooSmall = p_.pyramidOnly ? (w < 8 || h < 8)
                                       : (w < 2 * p_.edgeThreshold + 1 || h < 2 * p_.edgeThreshold + 1);
        if (tooSmall) break;
        addLevel(w, h, s);
    }
    if (levels_.empty()) throw std::runtime_error("image too small for ORB");
    if (p_.extraLevelScale > 0.f) {
        if (!p_.pyramidOnly) throw std::runtime_error("extraLevelScale requires pyramidOnly");
        float s = p_.extraLevelScale;
        int w = (int)std::lround(width / s), h = (int)std::lround(height / s);
        if (s <= levels_.back().scale || w < 2 || h < 2)
            throw std::runtime_error("extraLevelScale must be larger than the last level's scale");
        addLevel(w, h, s);
    }
    totalPixels_ = off;
    totalBytes_ = goff;
    if (p_.maxCorners <= 0) p_.maxCorners = (int)std::max<size_t>(totalPixels_ / 4, 4096);
    if (!(p_.abortCornerFraction >= 0.f)) throw std::runtime_error("abortCornerFraction must be >= 0");
    abortCorners_ = p_.abortCornerFraction > 0.f ? std::max(1, (int)(p_.abortCornerFraction * totalPixels_)) : 0;

    // Grid (distribution, per-cell FAST thresholds): per level about quota * cellsPerFeature
    // square cells over the area inside the border (at least 8 pixels per cell).
    if (p_.minFastThreshold < 0 || p_.minFastThreshold > 254) throw std::runtime_error("minFastThreshold must be 0..254");
    if ((p_.distribute || p_.minFastThreshold > 0) && !p_.pyramidOnly) {
        if (!(p_.cellsPerFeature > 0.f)) throw std::runtime_error("cellsPerFeature must be > 0");
        auto q = levelQuotas(p_, (int)levels_.size());
        for (size_t l = 0; l < levels_.size(); ++l) {
            LevelInfo& L = levels_[l];
            int aw = L.w - 2 * p_.edgeThreshold, ah = L.h - 2 * p_.edgeThreshold;
            double cells = std::max(1.0, q[l] * (double)p_.cellsPerFeature);
            // 8 .. 511 pixels: the GPU divides by multiplying, exact up to 511 (cellOf())
            L.cell = std::clamp((int)std::lround(std::sqrt((double)aw * ah / cells)), 8, 511);
            L.gw = (aw + L.cell - 1) / L.cell;
            L.gh = (ah + L.cell - 1) / L.cell;
            L.cellOff = gridCells_;
            gridCells_ += L.gw * L.gh;
        }
    }

    // Device: the caller's (in a graph the box's gpu/Device, shared with the other GPU blocks, so they can
    // read the pyramid), else one of its own; one with VK_KHR_performance_query for the debug counters.
    const bool perfCounters = ORB_PERF_COUNTERS && !p_.perfCounters.empty();
    ctx_ = perfCounters ? gpu::Context::create(allowCpuDevice, true)
                        : ctx ? std::move(ctx) : gpu::Context::create(allowCpuDevice, false);
    inst_ = ctx_->instance();
    phys_ = ctx_->physical();
    dev_ = ctx_->device();
    qfam_ = ctx_->queueFamily();
    memTypeIdx_ = ctx_->memoryType();
    devName_ = ctx_->deviceName();
    tsPeriod_ = ctx_->timestampPeriod();
    tsSupported_ = ctx_->timestampsSupported();

    // Buffers
    pyr_     = makeBuf(totalBytes_ * p_.numImages);  // one packed pyramid per image
    blur_    = makeBuf(totalBytes_);
    corners_ = makeBuf((size_t)p_.maxCorners * 4);
    score_   = makeBuf(totalBytes_ * 4);
    kps_     = makeBuf((size_t)p_.maxCandidates * kGpuKpStride * 4);
    counter_ = makeBuf(10 * 4);   // Counter in common.glsl
    fcand_   = makeBuf((size_t)p_.maxCorners * 4 * (p_.minFastThreshold > 0 ? 2 : 1));  // + FAST pass 2
    desc_    = makeBuf((size_t)p_.maxCandidates * 32);
    std::vector<int> quotas;
    size_t maxSel = kSelSlack;
    if (!p_.pyramidOnly) {
        quotas = levelQuotas(p_, (int)levels_.size());
        maxSel += (size_t)p_.maxFeatures;   // the quotas always sum to nfeatures <= maxFeatures
    }
    selCap_ = (int)maxSel;
    sel_ = makeBuf(kSelHeaderBytes + (maxSel + (size_t)p_.maxCandidates) * 4);  // selected + tie list
    // The score map and histograms start at zero; afterwards the GPU restores that every frame.
    memset(pyr_.map, 0, totalBytes_ * p_.numImages);
    memset(score_.map, 0, totalBytes_ * 4);
    memset(sel_.map, 0, kSelHeaderBytes);
    // rthr, rankHist, then per cell: count, start, fill, strong-corner flag, list of cells
    // without one; then responses and positions grouped by cell.
    size_t gridBytes = kGridHeaderBytes + (gridCells_ ? ((5 + 2 * kRankSlots) * (size_t)gridCells_ +
                                                         2 * (size_t)p_.maxCandidates) * 4 : 4);
    grid_ = makeBuf(gridBytes);
    memset(grid_.map, 0, gridBytes);

    // Resize lookup tables: source index and 11-bit weight. The x table has one entry per byte
    // of a padded destination row (column clamped to the image), starting at column -4, so a
    // thread's 4 entries are contiguous. resize.comp reads the 4 outputs' source pixels from a
    // 12-byte window; levels where that does not fit use resize_any.comp.
    std::vector<uint32_t> tabs;
    xTab_.assign(levels_.size(), 0);
    yTab_.assign(levels_.size(), 0);
    resizeFits_.assign(levels_.size(), true);
    auto entry = [](int x, int dst, int src) {
        // source coordinate (2x + 1) * src / (2 * dst) - 0.5, kept as an exact fraction
        int64_t q = (int64_t)(2 * x + 1) * src - dst;
        int64_t x0 = q / (2 * dst), fx = ((q - x0 * 2 * dst) * 1024) / dst;
        return (uint32_t)x0 | ((uint32_t)fx << 16);
    };
    for (size_t l = 1; l < levels_.size(); ++l) {
        const LevelInfo &D = levels_[l], &S = levels_[l - 1];
        xTab_[l] = (int)tabs.size();
        for (int b = 0; b < D.stride; b += 4) {   // every word of the padded row, from word -1
            uint32_t e[4];
            for (int k = 0; k < 4; ++k) e[k] = entry(std::clamp(b - kRowPad + k, 0, D.w - 1), D.w, S.w);
            int xs = (int)(e[0] & 0xFFFF) & ~3, last = (int)(e[3] & 0xFFFF) + 1;   // window xs .. xs+11
            if (last - xs > 11) resizeFits_[l] = false;
            tabs.insert(tabs.end(), e, e + 4);
        }
        yTab_[l] = (int)tabs.size();
        for (int y = 0; y < D.h; ++y) tabs.push_back(entry(y, D.h, S.h));
        while (tabs.size() % 4) tabs.push_back(0);
    }
    if (tabs.empty()) tabs.push_back(0);
    tabs_ = makeBuf(tabs.size() * 4);
    memcpy(tabs_.map, tabs.data(), tabs.size() * 4);

    std::vector<int32_t> consts;
    buildConsts(consts, levels_, quotas);
    consts_ = makeBuf(consts.size() * 4);
    memcpy(consts_.map, consts.data(), consts.size() * 4);

    // One descriptor set with all storage buffers, shared by every pipeline.
    constexpr int NB = 12;
    VkDescriptorSetLayoutBinding b[NB];
    for (int i = 0; i < NB; ++i) {
        b[i] = {};
        b[i].binding = i;
        b[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        b[i].descriptorCount = 1;
        b[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo dlci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    dlci.bindingCount = NB;
    dlci.pBindings = b;
    VK_CHECK(vkCreateDescriptorSetLayout(dev_, &dlci, nullptr, &dsl_));

    VkPushConstantRange pcr{VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(PushConsts)};
    VkPipelineLayoutCreateInfo plci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    plci.setLayoutCount = 1;
    plci.pSetLayouts = &dsl_;
    plci.pushConstantRangeCount = 1;
    plci.pPushConstantRanges = &pcr;
    VK_CHECK(vkCreatePipelineLayout(dev_, &plci, nullptr, &pl_));

    VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, NB};
    VkDescriptorPoolCreateInfo dpci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    dpci.maxSets = 1;
    dpci.poolSizeCount = 1;
    dpci.pPoolSizes = &ps;
    VK_CHECK(vkCreateDescriptorPool(dev_, &dpci, nullptr, &dpool_));
    VkDescriptorSetAllocateInfo dsai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    dsai.descriptorPool = dpool_;
    dsai.descriptorSetCount = 1;
    dsai.pSetLayouts = &dsl_;
    VK_CHECK(vkAllocateDescriptorSets(dev_, &dsai, &ds_));

    // Binding order must match shaders/common.glsl.
    Buf* bufs[NB] = {&pyr_, &blur_, &corners_, &score_, &kps_, &counter_, &consts_, &desc_, &tabs_, &sel_, &fcand_, &grid_};
    VkDescriptorBufferInfo bi[NB];
    VkWriteDescriptorSet wr[NB];
    for (int i = 0; i < NB; ++i) {
        bi[i] = {bufs[i]->buf, 0, VK_WHOLE_SIZE};
        wr[i] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        wr[i].dstSet = ds_;
        wr[i].dstBinding = i;
        wr[i].descriptorCount = 1;
        wr[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        wr[i].pBufferInfo = &bi[i];
    }
    vkUpdateDescriptorSets(dev_, NB, wr, 0, nullptr);

    pReset_  = makePipeline("reset", spv_reset, sizeof spv_reset);
    pResize_ = makePipeline("resize", spv_resize, sizeof spv_resize);
    pResizeAny_ = makePipeline("resize_any", spv_resize_any, sizeof spv_resize_any);
    pBlur_   = makePipeline("blur", spv_blur, sizeof spv_blur);
    pFast_   = makePipeline("fast", spv_fast, sizeof spv_fast);
    pFastFull_ = makePipeline("fast_full", spv_fast_full, sizeof spv_fast_full);
    pHarris_ = makePipeline("harris", spv_harris, sizeof spv_harris);
    pNms_    = makePipeline("nms", spv_nms, sizeof spv_nms);
    pPost_   = makePipeline("post_nms", spv_post_nms, sizeof spv_post_nms);
    pThresh_ = makePipeline("thresh", spv_thresh, sizeof spv_thresh);
    pSelect_ = makePipeline("select", spv_select, sizeof spv_select);
    pTie_    = makePipeline("select_tie", spv_select_tie, sizeof spv_select_tie);
    pDesc_   = makePipeline("describe", spv_describe, sizeof spv_describe);
    if (gridCells_) {
        pGridScan_    = makePipeline("grid_scan", spv_grid_scan, sizeof spv_grid_scan);
        pGridScatter_ = makePipeline("grid_scatter", spv_grid_scatter, sizeof spv_grid_scatter);
        pGridTop_     = makePipeline("grid_top", spv_grid_top, sizeof spv_grid_top);
        pGridRank_    = makePipeline("grid_rank", spv_grid_rank, sizeof spv_grid_rank);
        pRankThresh_  = makePipeline("rank_thresh", spv_rank_thresh, sizeof spv_rank_thresh);
    }
    if (p_.minFastThreshold > 0) {
        pFastMark_    = makePipeline("fast_mark", spv_fast_mark, sizeof spv_fast_mark);
        pCellList_    = makePipeline("cell_list", spv_cell_list, sizeof spv_cell_list);
        pFastCells_   = makePipeline("fast_cells", spv_fast_cells, sizeof spv_fast_cells);
    }

    VkCommandPoolCreateInfo cpci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    cpci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;   // re-recorded when parameters change
    cpci.queueFamilyIndex = qfam_;
    VK_CHECK(vkCreateCommandPool(dev_, &cpci, nullptr, &cpool_));
    VkCommandBufferAllocateInfo cbai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cbai.commandPool = cpool_;
    cbai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cbai.commandBufferCount = 1;
    VK_CHECK(vkAllocateCommandBuffers(dev_, &cbai, &cmd_));
    VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    VK_CHECK(vkCreateFence(dev_, &fci, nullptr, &fence_));

    if (tsSupported_ && p_.profile) {
        VkQueryPoolCreateInfo qpci{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
        qpci.queryType = VK_QUERY_TYPE_TIMESTAMP;
        qpci.queryCount = kQueries;
        VK_CHECK(vkCreateQueryPool(dev_, &qpci, nullptr, &qpool_));
        qpci.queryCount = kFineQueries;
        VK_CHECK(vkCreateQueryPool(dev_, &qpci, nullptr, &qpoolFine_));
    }
#if ORB_PERF_COUNTERS
    if (perfCounters) setupPerfCounters();
    perfEach_ = (qpoolFine_ || perfPool_) && p_.perfEach;
#endif

    // Fixed resolution: record the command buffer once and resubmit it every frame.
    recordCommands();
}

#if ORB_PERF_COUNTERS
// OrbParams::perfCounters "name,name,...": a performance query pool with those counters (one pass), its reset command
// buffer and the profiling lock (held for the life of the device; the command buffer is recorded once).
void OrbVk::setupPerfCounters() {
    auto enumerate = (PFN_vkEnumeratePhysicalDeviceQueueFamilyPerformanceQueryCountersKHR)vkGetInstanceProcAddr(
        inst_, "vkEnumeratePhysicalDeviceQueueFamilyPerformanceQueryCountersKHR");
    auto passes = (PFN_vkGetPhysicalDeviceQueueFamilyPerformanceQueryPassesKHR)vkGetInstanceProcAddr(
        inst_, "vkGetPhysicalDeviceQueueFamilyPerformanceQueryPassesKHR");
    auto lock = (PFN_vkAcquireProfilingLockKHR)vkGetDeviceProcAddr(dev_, "vkAcquireProfilingLockKHR");
    if (!enumerate || !passes || !lock) throw std::runtime_error("perfCounters: VK_KHR_performance_query not available");
    uint32_t n = 0;
    VK_CHECK(enumerate(phys_, qfam_, &n, nullptr, nullptr));
    std::vector<VkPerformanceCounterKHR> ctr(n, {VK_STRUCTURE_TYPE_PERFORMANCE_COUNTER_KHR});
    std::vector<VkPerformanceCounterDescriptionKHR> desc(n, {VK_STRUCTURE_TYPE_PERFORMANCE_COUNTER_DESCRIPTION_KHR});
    VK_CHECK(enumerate(phys_, qfam_, &n, ctr.data(), desc.data()));
    const std::string& list = p_.perfCounters;
    for (size_t a = 0; a <= list.size();) {
        size_t b = list.find(',', a);
        if (b == std::string::npos) b = list.size();
        const std::string name = list.substr(a, b - a);
        a = b + 1;
        if (name.empty()) continue;
        uint32_t i = 0;
        while (i < n && name != desc[i].name) ++i;
        if (i == n) throw std::runtime_error("perfCounters: no counter named " + name);
        if (ctr[i].storage != VK_PERFORMANCE_COUNTER_STORAGE_UINT64_KHR)
            throw std::runtime_error("perfCounters: counter " + name + " is not a 64-bit integer counter");
        perfIdx_.push_back(i);
        perfNames_.push_back(name);
    }
    VkQueryPoolPerformanceCreateInfoKHR pci{VK_STRUCTURE_TYPE_QUERY_POOL_PERFORMANCE_CREATE_INFO_KHR};
    pci.queueFamilyIndex = qfam_;
    pci.counterIndexCount = (uint32_t)perfIdx_.size();
    pci.pCounterIndices = perfIdx_.data();
    uint32_t np = 0;
    passes(phys_, &pci, &np);
    if (np != 1) throw std::runtime_error("perfCounters: these counters need " + std::to_string(np) + " passes; give fewer");
    VkQueryPoolCreateInfo qpci{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
    qpci.pNext = &pci;
    qpci.queryType = VK_QUERY_TYPE_PERFORMANCE_QUERY_KHR;
    qpci.queryCount = kFineQueries;
    VK_CHECK(vkCreateQueryPool(dev_, &qpci, nullptr, &perfPool_));

    VkCommandBufferAllocateInfo cbai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cbai.commandPool = cpool_;
    cbai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cbai.commandBufferCount = 1;
    VK_CHECK(vkAllocateCommandBuffers(dev_, &cbai, &perfResetCmd_));
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    VK_CHECK(vkBeginCommandBuffer(perfResetCmd_, &bi));
    vkCmdResetQueryPool(perfResetCmd_, perfPool_, 0, kFineQueries);
    VK_CHECK(vkEndCommandBuffer(perfResetCmd_));

    VkAcquireProfilingLockInfoKHR li{VK_STRUCTURE_TYPE_ACQUIRE_PROFILING_LOCK_INFO_KHR};
    li.timeout = UINT64_MAX;
    VK_CHECK(lock(dev_, &li));
}
#endif  // ORB_PERF_COUNTERS

OrbVk::~OrbVk() {
    if (!dev_) return;
    // Every submission of this instance was waited for (runGpu, snapshotPyramid): nothing in flight.
    // The device may be shared (gpu/Device): no vkDeviceWaitIdle, the queue may be another user's now.
    if (copyFence_) vkDestroyFence(dev_, copyFence_, nullptr);
    if (qpool_) vkDestroyQueryPool(dev_, qpool_, nullptr);
    if (qpoolFine_) vkDestroyQueryPool(dev_, qpoolFine_, nullptr);
#if ORB_PERF_COUNTERS
    if (perfPool_) {
        auto unlock = (PFN_vkReleaseProfilingLockKHR)vkGetDeviceProcAddr(dev_, "vkReleaseProfilingLockKHR");
        if (unlock) unlock(dev_);
        vkDestroyQueryPool(dev_, perfPool_, nullptr);
    }
#endif
    vkDestroyFence(dev_, fence_, nullptr);
    vkDestroyCommandPool(dev_, cpool_, nullptr);
    for (VkPipeline p : {pReset_, pResize_, pResizeAny_, pBlur_, pFast_, pFastFull_, pHarris_, pNms_, pPost_, pThresh_,
                         pSelect_, pTie_, pDesc_, pGridScan_, pGridScatter_, pGridTop_, pGridRank_, pRankThresh_, pFastMark_, pCellList_, pFastCells_})
        vkDestroyPipeline(dev_, p, nullptr);   // null handles are ignored
    vkDestroyDescriptorPool(dev_, dpool_, nullptr);
    vkDestroyPipelineLayout(dev_, pl_, nullptr);
    vkDestroyDescriptorSetLayout(dev_, dsl_, nullptr);
    for (Buf* b : {&pyr_, &blur_, &corners_, &score_, &kps_, &counter_, &consts_, &desc_, &tabs_, &sel_, &fcand_, &grid_}) freeBuf(*b);
    // the device and instance belong to ctx_ (released after the members)
}

OrbVk::Buf OrbVk::makeBuf(VkDeviceSize size) {  // host-mapped storage buffer
    Buf b;
    VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bci.size = size;
    bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;   // pyr_: snapshotPyramid
    bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VK_CHECK(vkCreateBuffer(dev_, &bci, nullptr, &b.buf));
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(dev_, b.buf, &req);
    if (!(req.memoryTypeBits & (1u << memTypeIdx_))) throw std::runtime_error("memory type not usable for buffer");
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mai.allocationSize = req.size;
    mai.memoryTypeIndex = memTypeIdx_;
    VK_CHECK(vkAllocateMemory(dev_, &mai, nullptr, &b.mem));
    VK_CHECK(vkBindBufferMemory(dev_, b.buf, b.mem, 0));
    VK_CHECK(vkMapMemory(dev_, b.mem, 0, VK_WHOLE_SIZE, 0, &b.map));
    return b;
}

void OrbVk::freeBuf(Buf& b) {
    if (b.map) vkUnmapMemory(dev_, b.mem);
    if (b.buf) vkDestroyBuffer(dev_, b.buf, nullptr);
    if (b.mem) vkFreeMemory(dev_, b.mem, nullptr);
    b = {};
}

VkPipeline OrbVk::makePipeline(const char* name, const uint32_t* spv, size_t bytes) {
    // OrbParams::listPipelines: name each pipeline before it is compiled, so driver compiler
    // statistics (e.g. V3D_DEBUG=shaderdb on a Raspberry Pi) can be matched to shaders.
    if (p_.listPipelines) orb_extractor::log(orb_extractor::log_level::info, std::string("[orb_vk] pipeline ") + name);
    VkShaderModuleCreateInfo smci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    smci.codeSize = bytes;
    smci.pCode = spv;
    VkShaderModule sm;
    VK_CHECK(vkCreateShaderModule(dev_, &smci, nullptr, &sm));
    VkComputePipelineCreateInfo cpci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    cpci.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    cpci.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    cpci.stage.module = sm;
    cpci.stage.pName = "main";
    cpci.layout = pl_;
    VkPipeline p;
    VK_CHECK(vkCreateComputePipelines(dev_, VK_NULL_HANDLE, 1, &cpci, nullptr, &p));
    vkDestroyShaderModule(dev_, sm, nullptr);
    pipeNames_[p] = name;
    return p;
}

// ---------------------------------------------------------------------------
// Command recording
// ---------------------------------------------------------------------------
void OrbVk::recordCommands() {
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    VK_CHECK(vkBeginCommandBuffer(cmd_, &bi));

    // Make earlier compute writes visible to later compute shaders.
    auto barrier = [&]() {
        VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        mb.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &mb,
                             0, nullptr, 0, nullptr);
    };
    auto hostBarrier = [&]() {  // results readable by the CPU after the fence
        VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        mb.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &mb, 0,
                             nullptr, 0, nullptr);
    };
    VkPipeline bound = VK_NULL_HANDLE;   // perfEach labels
    auto bind = [&](VkPipeline p, const PushConsts& pc) {
        bound = p;
        vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_COMPUTE, p);
        vkCmdPushConstants(cmd_, pl_, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof pc, &pc);
    };
    auto stamp = [&](uint32_t q) {
        if (qpool_) vkCmdWriteTimestamp(cmd_, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, qpool_, q);
    };

    if (qpool_) {
        vkCmdResetQueryPool(cmd_, qpool_, 0, kQueries);
        vkCmdWriteTimestamp(cmd_, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, qpool_, 0);
    }
    fineLabels_.clear();
    if (qpoolFine_) {
        vkCmdResetQueryPool(cmd_, qpoolFine_, 0, kFineQueries);
        vkCmdWriteTimestamp(cmd_, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, qpoolFine_, 0);
    }
    // perfCounters: performance query i covers the kernel group labelled fineLabels_[i]
    if (perfPool_) vkCmdBeginQuery(cmd_, perfPool_, 0, 0);
    auto endPerf = [&]() {
        if (perfPool_) vkCmdEndQuery(cmd_, perfPool_, (uint32_t)fineLabels_.size());
    };
    // profile: timestamp after a barrier, attributing the time since the previous one to `label`
    // (perfEach: after every dispatch instead, see `dispatch`)
    auto mark = [&](const std::string& label, bool eachDispatch = false) {
        if (eachDispatch != perfEach_) return;
        if ((!qpoolFine_ && !perfPool_) || fineLabels_.size() + 1 >= kFineQueries) return;
        endPerf();
        fineLabels_.push_back(label);
        if (qpoolFine_)
            vkCmdWriteTimestamp(cmd_, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, qpoolFine_, (uint32_t)fineLabels_.size());
        if (perfPool_) vkCmdBeginQuery(cmd_, perfPool_, (uint32_t)fineLabels_.size(), 0);
    };
    auto dispatch = [&](uint32_t x, uint32_t y, uint32_t z, const std::string& tag) {
        vkCmdDispatch(cmd_, x, y, z);
        if (perfEach_) {
            auto it = pipeNames_.find(bound);
            mark((it != pipeNames_.end() ? it->second : std::string("?")) + tag + " [" + std::to_string(x * y * z) + " wg]",
                 true);
        }
    };
    vkCmdBindDescriptorSets(cmd_, VK_PIPELINE_BIND_POINT_COMPUTE, pl_, 0, 1, &ds_, 0, nullptr);

    const float hs = 1.0f / (4.0f * 7.0f * 255.0f);  // normalises Sobel sums like OpenCV
    PushConsts base{};
    base.fastThr = p_.fastThreshold;
    base.border = p_.edgeThreshold;
    base.maxKp = p_.maxCandidates;
    base.maxCorners = p_.maxCorners;
    base.harrisK = p_.harrisK;
    base.harrisScale4 = hs * hs * hs * hs;
    base.imgStride = (int)totalBytes_;

    auto levelPc = [&](int l) {
        PushConsts pc = base;
        const LevelInfo& L = levels_[l];
        pc.w = L.w; pc.h = L.h; pc.off = L.goff; pc.stride = L.stride; pc.level = l;
        if (l > 0) {
            const LevelInfo& S = levels_[l - 1];
            pc.srcW = S.w; pc.srcH = S.h; pc.srcOff = S.goff; pc.srcStride = S.stride;
            pc.xTab = xTab_[l]; pc.yTab = yTab_[l];
        }
        return pc;
    };
    auto dispatchResize = [&](int l, uint32_t images) {
        PushConsts pc = levelPc(l);
        // whole padded rows; resize.comp: kResizeRows rows per thread, resize_any.comp: 1
        bool fits = resizeFits_[l];
        bind(fits ? pResize_ : pResizeAny_, pc);
        dispatch(groups(pc.stride / 4, 16), groups(pc.h, 8 * (fits ? kResizeRows : 1)), images, " L" + std::to_string(l));
    };
    const int nl = (int)levels_.size();

    if (p_.pyramidOnly) {
        // Downscale level by level; all images are batched along gl_WorkGroupID.z.
        for (int l = 1; l < nl; ++l) {
            dispatchResize(l, (uint32_t)p_.numImages);
            barrier();
        }
        hostBarrier();
        for (uint32_t q = 1; q < kQueries; ++q) stamp(q);
        endPerf();
        VK_CHECK(vkEndCommandBuffer(cmd_));
        return;
    }

    auto dispatchList = [&](VkPipeline pipe, const PushConsts& pc) {  // grid-stride list kernels
        bind(pipe, pc);
        dispatch(kListGroups, 1, 1, "");
    };
    base.nlevels = nl;
    base.selCap = selCap_;
    base.gridCells = gridCells_;
    base.distribute = p_.distribute ? 1 : 0;
    // per-cell thresholds only while the low threshold is below the high one
    base.fastLo = p_.minFastThreshold > 0 && p_.minFastThreshold < p_.fastThreshold ? p_.minFastThreshold : 0;
    base.abortCorners = abortCorners_;

    // Reset counters and selection histograms (the score map is already zero, see post_nms).
    dispatchList(pReset_, base);
    barrier();
    mark("reset");

    // Stage 1: pyramid, blur and FAST. Step k builds level k+1 and, independently, blurs and
    // runs FAST on level k, so each level needs a single barrier. (Building the whole pyramid
    // first and then one dispatch over all levels was measured slower on a Pi 5.)
    auto dispatchLevel = [&](VkPipeline pipe, int l, int tw, int th) {
        PushConsts pc = levelPc(l);
        bind(pipe, pc);
        dispatch(groups(pc.w, tw), groups(pc.h, th), 1, " L" + std::to_string(l));
    };
    for (int k = 0; k < nl; ++k) {
        if (k + 1 < nl) dispatchResize(k + 1, 1);
        dispatchLevel(pBlur_, k, kBlurTileW, kBlurTileH);
        dispatchLevel(pFast_, k, kFastTileW, kFastTileH);
        barrier();
        mark("L" + std::to_string(k) + " resize+blur+fast pre-test");
    }
    // Full FAST test on the blocks that passed the pre-test, all levels at once.
    dispatchList(pFastFull_, base);
    barrier();
    mark("fast_full");
    if (base.fastLo > 0) {
        // Per-cell thresholds: mark the cells that got a corner, list the others, then FAST
        // pass 2 at the low threshold over the listed cells only (see vk/tools/gen_shaders.py).
        dispatchList(pFastMark_, base);
        barrier();
        mark("fast_mark");
        dispatchList(pCellList_, base);
        barrier();
        mark("cell_list");
        dispatchList(pFastCells_, base);
        barrier();
        mark("fast_cells (pass 2 pre-test)");
        PushConsts pc2 = base;
        pc2.fastPass2 = 1;
        dispatchList(pFastFull_, pc2);
        barrier();
        mark("fast_full pass 2");
    }
    stamp(1);

    // Stage 2: Harris response for every FAST corner.
    dispatchList(pHarris_, base);
    barrier();
    mark("harris");
    stamp(2);

    // Stage 3: NMS, then per-level selection of the strongest keypoints.
    dispatchList(pNms_, base);
    barrier();
    mark("nms");
    if (base.distribute) {         // rank within grid cells (counting sort by cell; nms counted)
        bind(pGridScan_, base);    // first list index of each cell
        dispatch(1, 1, 1, "");
        barrier();
        mark("grid_scan");
        dispatchList(pGridScatter_, base);
        barrier();
        mark("grid_scatter");
        dispatchList(pGridTop_, base);   // each cell's best MAX_RANK keypoints
        barrier();
        mark("grid_top");
        dispatchList(pGridRank_, base);
        barrier();
        mark("grid_rank");
        bind(pRankThresh_, base);  // boundary rank per level
        dispatch(1, 1, 1, "");
        barrier();
        mark("rank_thresh");
    }
    dispatchList(pPost_, base);    // response histograms + score map cleanup
    barrier();
    mark("post_nms");
    bind(pThresh_, base);          // boundary bin per level
    dispatch(1, 1, 1, "");
    barrier();
    mark("thresh");
    dispatchList(pSelect_, base);  // keeps keypoints above the boundary bin, collects ties
    barrier();
    mark("select");
    dispatchList(pTie_, base);     // ranks the boundary-bin keypoints exactly
    barrier();
    mark("select_tie");
    stamp(3);

    // Stage 4: orientation and descriptors for the selected keypoints only.
    dispatchList(pDesc_, base);
    hostBarrier();
    mark("describe");
    stamp(4);
    endPerf();
    VK_CHECK(vkEndCommandBuffer(cmd_));
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------
void OrbVk::runGpuStereo(const uint8_t* left, const uint8_t* right, int stride) {
    if (p_.numImages != 2) throw std::runtime_error("runGpuStereo requires numImages = 2");
    const uint8_t* imgs[2] = {left, right};
    runGpuMulti(imgs, 2, stride);
}

void OrbVk::runGpuMulti(const uint8_t* const* images, int n, int stride) {
    if (n != p_.numImages) throw std::runtime_error("runGpuMulti: n must equal numImages");
    const LevelInfo& L0 = levels_[0];
    for (int i = 1; i < n; ++i) {
        uint8_t* dst = static_cast<uint8_t*>(pyr_.map) + (size_t)i * totalBytes_ + L0.goff;
        copyLevel0(dst, L0.stride, images[i], stride, w0_, h0_);
    }
    runGpu(images[0], stride);
}

void OrbVk::setFastThreshold(int t) {
    if (t < 1 || t > 254) throw std::runtime_error("FAST threshold must be 1..254");
    if (t == p_.fastThreshold) return;
    p_.fastThreshold = t;
    dirty_ = true;
}

void OrbVk::setNumFeatures(int n) {
    if (p_.pyramidOnly) throw std::runtime_error("setNumFeatures is not available in pyramidOnly mode");
    if (n < 1 || n > maxFeatures()) throw std::runtime_error("setNumFeatures: n must be 1..maxFeatures");
    if (n == p_.nfeatures) return;
    p_.nfeatures = n;
    // The GPU is idle between runs, so the quota table can be rewritten in place.
    std::vector<int32_t> consts;
    buildConsts(consts, levels_, levelQuotas(p_, (int)levels_.size()));
    memcpy(consts_.map, consts.data(), consts.size() * 4);
}

const uint8_t* OrbVk::pyramidLevel(int l, int image) const {
    if (l < 0 || l >= (int)levels_.size() || image < 0 || image >= p_.numImages)
        throw std::runtime_error("pyramidLevel: level or image out of range");
    return static_cast<const uint8_t*>(pyr_.map) + (size_t)image * totalBytes_ + levels_[l].goff;
}

std::shared_ptr<const gpu::Pyramid> OrbVk::snapshotPyramid(int image) {
    if (image < 0 || image >= p_.numImages) throw std::runtime_error("snapshotPyramid: image out of range");
    if (!pool_) {
        pool_ = std::make_shared<gpu::PyramidPool>(ctx_, (VkDeviceSize)totalBytes_);
        VkCommandBufferAllocateInfo cbai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cbai.commandPool = cpool_;
        cbai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cbai.commandBufferCount = 1;
        VK_CHECK(vkAllocateCommandBuffers(dev_, &cbai, &copyCmd_));
        VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        VK_CHECK(vkCreateFence(dev_, &fci, nullptr, &copyFence_));
    }
    std::shared_ptr<gpu::Pyramid> out = pool_->acquire();
    out->levels.clear();
    for (const LevelInfo& L : levels_) out->levels.push_back({L.w, L.h, L.stride, L.goff, L.scale});

    // The destination changes from frame to frame: record the copy each time (microseconds).
    VK_CHECK(vkResetCommandBuffer(copyCmd_, 0));
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(copyCmd_, &bi));
    // the last run's shader writes (an earlier submission on the queue) before the transfer reads
    VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_HOST_WRITE_BIT;
    mb.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    vkCmdPipelineBarrier(copyCmd_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_HOST_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &mb, 0, nullptr, 0, nullptr);
    VkBufferCopy region{(VkDeviceSize)image * totalBytes_, 0, (VkDeviceSize)totalBytes_};
    vkCmdCopyBuffer(copyCmd_, pyr_.buf, out->buffer, 1, &region);
    // for the readers: shaders on the same queue (they add their own barrier too) and the host
    mb.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    mb.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_HOST_READ_BIT;
    vkCmdPipelineBarrier(copyCmd_, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &mb, 0, nullptr, 0,
                         nullptr);
    VK_CHECK(vkEndCommandBuffer(copyCmd_));
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &copyCmd_;
    VK_CHECK(ctx_->submit(si, copyFence_));
    VK_CHECK(vkWaitForFences(dev_, 1, &copyFence_, VK_TRUE, UINT64_MAX));
    VK_CHECK(vkResetFences(dev_, 1, &copyFence_));
    return out;
}

size_t OrbVk::pyramidBuffers() const { return pool_ ? pool_->allocated() : 0; }

void OrbVk::runGpu(const uint8_t* gray, int stride) {
    if (dirty_) {  // push constants are baked into the command buffer
        VK_CHECK(vkResetCommandBuffer(cmd_, 0));
        recordCommands();
        dirty_ = false;
    }
    // Unified memory: copy the 8-bit rows straight into the mapped buffer (rows padded to 4 bytes).
    const LevelInfo& L0 = levels_[0];
    copyLevel0(static_cast<uint8_t*>(pyr_.map) + L0.goff, L0.stride, gray, stride, w0_, h0_);

    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    const VkCommandBuffer cbs[2] = {perfResetCmd_, cmd_};
    si.commandBufferCount = perfPool_ ? 2 : 1;
    si.pCommandBuffers = perfPool_ ? cbs : &cmd_;
    VK_CHECK(ctx_->submit(si, fence_));
    VK_CHECK(vkWaitForFences(dev_, 1, &fence_, VK_TRUE, UINT64_MAX));
    VK_CHECK(vkResetFences(dev_, 1, &fence_));

    if (qpool_) {
        uint64_t ts[kQueries];
        if (vkGetQueryPoolResults(dev_, qpool_, 0, kQueries, sizeof ts, ts, sizeof(uint64_t),
                                  VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT) == VK_SUCCESS) {
            gpuMs_ = double(ts[kQueries - 1] - ts[0]) * tsPeriod_ * 1e-6;
            stageMs_.resize(kQueries - 1);
            for (uint32_t i = 1; i < kQueries; ++i) stageMs_[i - 1] = double(ts[i] - ts[i - 1]) * tsPeriod_ * 1e-6;
        }
    }
    if (qpoolFine_ && !fineLabels_.empty()) {
        const uint32_t n = (uint32_t)fineLabels_.size() + 1;
        uint64_t ts[kFineQueries];
        if (vkGetQueryPoolResults(dev_, qpoolFine_, 0, n, n * sizeof(uint64_t), ts, sizeof(uint64_t),
                                  VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT) == VK_SUCCESS) {
            kernelMs_.resize(n - 1);
            for (uint32_t i = 1; i < n; ++i)
                kernelMs_[i - 1] = {fineLabels_[i - 1], double(ts[i] - ts[i - 1]) * tsPeriod_ * 1e-6};
        }
    }
#if ORB_PERF_COUNTERS
    if (perfPool_ && !fineLabels_.empty()) {
        const uint32_t n = (uint32_t)fineLabels_.size(), c = (uint32_t)perfIdx_.size();
        std::vector<VkPerformanceCounterResultKHR> r((size_t)n * c);
        VK_CHECK(vkGetQueryPoolResults(dev_, perfPool_, 0, n, r.size() * sizeof r[0], r.data(), c * sizeof r[0],
                                       VK_QUERY_RESULT_WAIT_BIT));
        kernelCounters_.resize(n);
        for (uint32_t i = 0; i < n; ++i) {
            kernelCounters_[i].first = fineLabels_[i];
            kernelCounters_[i].second.resize(c);
            for (uint32_t j = 0; j < c; ++j) kernelCounters_[i].second[j] = double(r[(size_t)i * c + j].uint64);
        }
    }
#endif
}

void OrbVk::readCandidates(std::vector<OrbCandidate>& out, bool* overflow) const {
    const uint32_t* cnt = static_cast<const uint32_t*>(counter_.map);
    uint32_t n = std::min<uint32_t>(cnt[0], (uint32_t)p_.maxCandidates);
    if (overflow) *overflow = cnt[1] != 0 || cnt[3] != 0;
    const uint32_t* k = static_cast<const uint32_t*>(kps_.map);
    const uint32_t* d = static_cast<const uint32_t*>(desc_.map);
    out.resize(n);
    for (uint32_t i = 0; i < n; ++i) {
        const uint32_t* e = k + (size_t)i * kGpuKpStride;
        OrbCandidate& c = out[i];
        memcpy(&c.response, e + 0, 4);
        memcpy(&c.angleRad, e + 1, 4);
        c.level = (int)e[2];
        c.px = (int)e[3];
        c.py = (int)e[4];
        c.selected = e[5] != 0;
        if (c.selected) memcpy(c.desc, d + (size_t)i * 8, 32);
        else memset(c.desc, 0, 32);
    }
}

void OrbVk::readPyramid(std::vector<uint32_t>& pyr, std::vector<uint32_t>& blur, int image) const {
    // Unpack the padded 8-bit GPU layout into the compact layout used by the CPU reference.
    const uint8_t* a = static_cast<const uint8_t*>(pyr_.map) + (size_t)image * totalBytes_;
    const uint8_t* b = static_cast<const uint8_t*>(blur_.map);
    pyr.assign(totalPixels_, 0);
    blur.assign(totalPixels_, 0);
    for (auto& L : levels_)
        for (int y = 0; y < L.h; ++y)
            for (int x = 0; x < L.w; ++x) {
                size_t g = (size_t)L.goff + (size_t)y * L.stride + x, c = (size_t)L.off + (size_t)y * L.w + x;
                pyr[c] = a[g];
                blur[c] = b[g];
            }
}

void OrbVk::detectAndCompute(const uint8_t* gray, int stride, std::vector<OrbKeypoint>& keypoints,
                             std::vector<uint8_t>& descriptors) {
    if (p_.pyramidOnly) throw std::runtime_error("detectAndCompute is not available in pyramidOnly mode");
    runGpu(gray, stride);
    std::vector<OrbCandidate> c;
    bool ovf = false;
    readCandidates(c, &ovf);
    if (ovf && abortCorners_ == 0)
        orb_extractor::log(orb_extractor::log_level::warn,
                           "warning: GPU buffer overflow (raise maxCandidates/maxCorners or fastThreshold)");
    stats_.fastThreshold = p_.fastThreshold;
    stats_.corners = static_cast<const uint32_t*>(counter_.map)[2];
    // per-cell thresholds: corners added by the low threshold (FAST pass 2)
    const uint32_t* cnt = static_cast<const uint32_t*>(counter_.map);
    bool cellThr = p_.minFastThreshold > 0 && p_.minFastThreshold < p_.fastThreshold;
    stats_.weakCorners = cellThr ? stats_.corners - cnt[8] : 0;
    stats_.weakCells = cellThr ? cnt[9] : 0;
    stats_.candidates = (uint32_t)c.size();
    stats_.candidatesPerLevel.assign(levels_.size(), 0);
    for (auto& x : c) ++stats_.candidatesPerLevel[x.level];
    stats_.quotas = levelQuotas(p_, (int)levels_.size());
    stats_.overflow = ovf;
    {
        const uint32_t* cnt = static_cast<const uint32_t*>(counter_.map);
        stats_.aborted = abortCorners_ > 0 && (cnt[2] > (uint32_t)abortCorners_ || cnt[3] != 0);
    }
    // The GPU already picked the keypoints and described only those; keep just them.
    c.erase(std::remove_if(c.begin(), c.end(), [](const OrbCandidate& x) { return !x.selected; }), c.end());
    selectFeatures(c, p_, levels_, keypoints, descriptors);
}

#endif  // ORB_HAVE_VULKAN
