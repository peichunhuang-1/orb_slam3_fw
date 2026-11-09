// Resources shared by all compute shaders.
//
// Portability rules (Raspberry Pi 5 / Mesa v3dv and NXP i.MX95 / Mali-G310):
//   * Vulkan 1.1 core only, 32-bit int/float only (no int8/int16/int64, no subgroups,
//     no storage-image formats), workgroups of at most 128 invocations.
//   * Do NOT index private or constant arrays with run-time indices. On V3D such arrays
//     are spilled to scratch memory and became 8-13x slower (measured on a Pi 5).
//     Hot shaders (blur, fast, harris) are therefore generated fully unrolled by
//     tools/gen_shaders.py.
//
// Images are stored as packed 8-bit pixels (4 per uint). Pixel (0, 0) of every level is at a
// 4-byte aligned offset, rows are a multiple of 4 bytes ("stride"), and each row has 4 bytes
// before pixel 0 and at least 4 after pixel w-1 that repeat the edge pixels (see resize.comp).

layout(std430, set = 0, binding = 0) buffer Pyr     { uint pyr[];     };  // image pyramid (packed u8)
layout(std430, set = 0, binding = 1) buffer Blur    { uint blurImg[]; };  // Gaussian-blurred pyramid for BRIEF
layout(std430, set = 0, binding = 2) buffer Corners { uint corners[]; };  // FAST corners: x | y << 13 | level << 26
layout(std430, set = 0, binding = 3) buffer Score   { float score[];  };  // Harris score map, same layout as pyramid

struct KP {
    float response;  float angle;   // angle (radians) is valid only if selected
    int level; int px; int py;      // level and coordinates within that level
    int selected;                   // 1 = kept by the per-level selection (descriptor computed)
    int cell; int rank;             // distribute only: grid cell, rank within the cell (capped)
};
layout(std430, set = 0, binding = 4) buffer Kps     { KP kps[]; };          // keypoints after NMS
layout(std430, set = 0, binding = 5) buffer Counter {
    uint kpCount; uint kpOverflow; uint cornerCount; uint cornerOverflow;
    uint selCount; uint tieCount; uint fastCandCount;
    uint fastCand2Count;   // per-cell thresholds: pre-test blocks of pass 2
    uint strongCornerCount;   // per-cell thresholds: corners of pass 1 (set by fast_mark.comp)
    uint weakCellCount;       // per-cell thresholds: cells without a corner of pass 1 (cell_list.comp)
};
// consts[0..255]   : rBRIEF sampling pairs (x0, y0, x1, y1)
// consts[256..259] : umax[0..15], half-widths of the circular orientation patch
// consts[260 + l]  : level l as (width, height, byte offset, stride)
// consts[276 + l]  : level l as (keypoint quota, 0, 0, 0)
// consts[292 + l]  : level l grid as (ceil(2^22 / cell size), cells across, cells down,
//                    index of first cell); see cellOf()
// consts[308 + l]  : level l grid as (cell size, 0, 0, 0)
layout(std430, set = 0, binding = 6) readonly buffer Consts { ivec4 consts[]; };
layout(std430, set = 0, binding = 7) buffer Desc    { uint desc[]; };      // 8 uints (256 bits) per keypoint
// Resize lookup tables, one entry per destination column / row: src0 | weight << 16.
// Precomputed on the CPU so the GPU never does integer division.
layout(std430, set = 0, binding = 8) readonly buffer Tabs { uint tabs[]; };
// Per-level keypoint selection (see post_nms.comp, thresh.comp, select.comp):
//   thr[l]        : (boundary fine bin, number of keypoints in higher bins); bin -1 = keep all
//   histCoarse    : 16 levels x 64 bins over the response's float bits >> 25
//   histFine      : 16 levels x 2048 bins over the response's float bits >> 20
//   selIdx        : [0, selCap) indices into kps[] of the selected keypoints,
//                   [selCap, ...) keypoints that fall in a boundary bin (tie list, see select.comp)
// FAST blocks that passed the pre-test (fast.comp -> fast_full.comp):
//   x4 / 4 | y << 11 | level << 24 | pixel mask << 28; capacity maxCorners
//   (per-cell thresholds: pass 2's blocks follow in [maxCorners, 2 maxCorners))
layout(std430, set = 0, binding = 10) buffer FastCand { uint fastCand[]; };
layout(std430, set = 0, binding = 9) buffer Sel {
    ivec2 thr[16];
    uint  histCoarse[16 * 64];
    uint  histFine[16 * 2048];
    uint  selIdx[];
};
// Grid cells, used by the spatial distribution (OrbParams::distribute, see grid_*.comp and
// rank_thresh.comp) and the per-cell FAST thresholds (OrbParams::minFastThreshold, see
// tools/gen_shaders.py and fast_mark.comp). Every level is divided into cells.
// Distribution: keypoints are ranked within their cell, and each level keeps its quota in the
// order (rank, then response), i.e. the best of every cell first.
//   rthr[l]  : (boundary rank R, number of keypoints with rank < R); R = -1 = keep all
//   rankHist : 16 levels x 64 ranks
//   grid     : [0, C) keypoints per cell, [C, 2C) first index of each cell in the list,
//              [2C, 3C) fill counters, [3C, 4C) 1 = the cell has a corner of the high FAST
//              threshold (pass 1), [4C, 5C) list of the other cells (cell_list.comp),
//              then grouped by cell (M = pc.maxKp):
//              [5C, 5C + M) response bits, [5C + M, 5C + 2M) position y << 16 | x,
//              then each cell's best MAX_RANK keypoints (grid_top.comp), best first:
//              TOP_BASE + c * MAX_RANK + j response bits, TOP_BASE + C * MAX_RANK + c * MAX_RANK + j
//              position (C = pc.gridCells)
layout(std430, set = 0, binding = 11) buffer Grid {
    ivec2 rthr[16];
    uint  rankHist[16 * 64];
    uint  grid[];
};
uint TOP_BASE(uint C, uint M) { return 5u * C + 2u * M; }

layout(push_constant) uniform PC {
    int   w;       int h;       int off;     int stride;     // current level
    int   srcW;    int srcH;    int srcOff;  int srcStride;  // source level (resize only)
    int   xTab;    int yTab;    int level;   int fastThr;
    int   border;  int maxKp;   int maxCorners;
    float harrisK; float harrisScale4;
    int   imgStride;  // byte distance between images when several are batched (gl_WorkGroupID.z)
    int   nlevels;
    int   selCap;     // capacity of the selected-keypoint list (start of the tie list)
    int   gridCells;  // total grid cells over all levels (0 = no grid)
    int   distribute; // 1 = spatially distributed selection
    int   fastLo;     // > 0: per-cell FAST thresholds; cells without a corner of fastThr use fastLo
    int   fastPass2;  // 1 = FAST pass 2 of the per-cell thresholds (fast.comp, fast_full.comp)
    int   abortCorners; // > 0: abnormal frame above this many FAST corners, see aborted()
} pc;

const int HALF_PATCH = 15;
const int UMAX_BASE  = 256;
const int LEVEL_BASE = 260;
const int QUOTA_BASE = 276;
const int CELL_BASE  = 292;
const int CELL2_BASE = 308;
const int MAX_RANK   = 7;    // ranks within a cell are capped (rankHist has 64 bins). Each cell
                             // keeps about one keypoint per level quota (cellsPerFeature), so the
                             // boundary rank stays far below this: 63 and 7 selected the same
                             // keypoints on EuRoC V1_02 and TUM-VI room1 / outdoors4. Must stay
                             // below kRankSlots (orb_vk.cpp), the size of each cell's top list.

// Keypoint order within a cell and inside a boundary bin: response descending, then y, then x.
bool kpBefore(uint j, float r, int x, int y) {      // true if kps[j] ranks before (r, x, y)
    float rj = kps[j].response;
    if (rj != r) return rj > r;
    if (kps[j].py != y) return kps[j].py < y;
    return kps[j].px < x;
}

// Grid cell of pixel (x, y) of level l (inside the border; the last cells may be smaller).
// n / cell as (n * ceil(2^22 / cell)) >> 22, which equals the integer division for n < 8192 and
// cell <= 511 (n * error < 2^22; the product stays below 2^32). V3D has no integer divider.
int cellOf(int x, int y, int l) {
    ivec4 G = consts[CELL_BASE + l];
    int cx = min(int((uint(x - pc.border) * uint(G.x)) >> 22), G.y - 1);
    int cy = min(int((uint(y - pc.border) * uint(G.x)) >> 22), G.z - 1);
    return G.w + cy * G.y + cx;
}

// Abnormal frame (OrbParams::abortCornerFraction): FAST found far more corners than any normal
// scene (sensor noise, gravel), or a corner list overflowed. Every kernel after FAST checks this
// first and returns, so such a frame costs only the FAST stage and yields no keypoints.
bool aborted() {
    return pc.abortCorners > 0 && (cornerCount > uint(pc.abortCorners) || cornerOverflow != 0u);
}

// List kernels run a fixed number of workgroups and loop over their list (grid-stride).
// This avoids vkCmdDispatchIndirect, which v3dv turns into a CPU job that stalls the GPU.
uint gridStride() { return gl_NumWorkGroups.x * gl_WorkGroupSize.x; }

// Histogram bins: for positive floats the bit pattern is monotonic in the value.
uint fineBin(float r)   { return floatBitsToUint(r) >> 20; }   // 0 .. 2047
uint coarseBin(float r) { return floatBitsToUint(r) >> 25; }   // 0 .. 63

int umaxAt(int v) { return consts[UMAX_BASE + (v >> 2)][v & 3]; }

uint ldPyr(int byteIdx)  { return (pyr[byteIdx >> 2]     >> ((byteIdx & 3) << 3)) & 0xFFu; }
uint ldBlur(int byteIdx) { return (blurImg[byteIdx >> 2] >> ((byteIdx & 3) << 3)) & 0xFFu; }
