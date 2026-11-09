#pragma once
// ORB (Oriented FAST + Rotated BRIEF) feature extraction on Vulkan compute.
// One code base for Raspberry Pi 5 (VideoCore VII, Mesa v3dv) and NXP i.MX95 (Mali-G310).
#include <vulkan/vulkan.h>

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

struct OrbParams {
    int   nfeatures     = 1000;   // keypoints kept after per-level selection
    int   maxFeatures   = 0;      // capacity for setNumFeatures(), 0 = nfeatures
    int   nlevels       = 8;      // pyramid levels
    float scaleFactor   = 1.2f;   // scale between consecutive levels
    int   fastThreshold = 20;
    int   edgeThreshold = 31;     // border without features; also the patch size
    float harrisK       = 0.04f;
    int   maxCandidates = 32768;  // GPU capacity for keypoints after NMS
    int   maxCorners    = 0;      // GPU capacity for FAST corners, 0 = auto (pyramid pixels / 4)
    // Record GPU timestamps (lastGpuMs / lastStageMs). Off by default: on v3dv every
    // timestamp is a CPU job that briefly stalls the GPU.
    bool  profile       = false;

    // Spatial distribution (similar in effect to ORB-SLAM's quadtree). Each level's area inside
    // the border is divided into about quota * cellsPerFeature square cells; keypoints are
    // ranked within their cell, and the level keeps its quota in the order (rank, response):
    // the strongest keypoint of every cell first, then every cell's second, and so on.
    // Without it each level keeps its strongest keypoints, which cluster on high-contrast spots.
    // The grid is fixed at construction (from nfeatures); setNumFeatures() keeps it.
    bool  distribute      = false;
    float cellsPerFeature = 1.0f;

    // Per-cell FAST threshold, as ORB-SLAM's per-cell fallback (iniThFAST / minThFAST), decided
    // within the frame: > 0 enables it. A grid cell (same grid as distribute) that has a corner
    // of fastThreshold keeps only those; a cell without uses minFastThreshold. Low-texture
    // regions get corners without lowering the threshold of the whole image. FAST runs at
    // fastThreshold as usual, then a second time at minFastThreshold over the cells without a
    // corner only, so the cost follows the low-texture area. (If fastThreshold <=
    // minFastThreshold, every cell uses fastThreshold.)
    int   minFastThreshold = 0;

    // Abnormal frame: if FAST finds more corners than abortCornerFraction x the pyramid's pixels
    // (or a corner list overflows), the GPU skips everything after FAST and the frame yields no
    // keypoints (OrbFrameStats::aborted). Normal scenes have 1-3 % (EuRoC V1_02 at 720x540: at
    // most 4.3 %), sensor noise / dense fine texture 15-35 %, which also cost 3-4x the time.
    // fast_full.comp also stops early, so an aborted frame costs less than a normal one. 0 = off.
    float abortCornerFraction = 0.f;

    // Pyramid-only mode (e.g. for KLT): no blur, FAST or descriptors, and levels may be
    // smaller than the ORB patch.
    bool  pyramidOnly     = false;
    float extraLevelScale = 0.f;  // > 0: append one more level at 1/extraLevelScale (e.g. 8), pyramidOnly only
    int   numImages       = 1;    // pyramidOnly: build 1..8 pyramids in the same dispatches (stereo = 2)

    // Debug (were the environment variables ORBVK_PERF, ORBVK_PERF_EACH, ORB_LIST_PIPELINES; the tools
    // still read those, e.g. extract_profile):
    std::string perfCounters;     // ORB_PERF_COUNTERS builds: VK_KHR_performance_query counter names,
                                  // comma separated, one query per kernel group; empty = off
    bool  perfEach        = false;  // with perfCounters or profile: one query / timestamp per dispatch
    bool  listPipelines   = false;  // name each pipeline in the log before it is compiled (to match
                                    // driver compiler statistics, e.g. V3D_DEBUG=shaderdb, to shaders)
};

struct OrbKeypoint {
    float x, y;       // full-resolution image coordinates
    float size;       // patch size (31) * level scale
    float angle;      // degrees in [0, 360), same convention as OpenCV
    float response;   // Harris response
    int   octave;     // pyramid level
};

// Geometry of one pyramid level.
//   off         : offset in the compact layout (one entry per pixel, used by the CPU reference)
//   goff/stride : byte offset of pixel (0, 0) and row length (multiple of 4) in the packed GPU
//                 layout; each row has 4 bytes before pixel 0 and >= 4 after pixel w-1 that
//                 repeat the edge pixels
//   cell/gw/gh  : grid (OrbParams::distribute / minFastThreshold): cell size in pixels and cells
//                 across / down, covering [border, w - border) x [border, h - border)
//   cellOff     : index of the level's first cell among all levels
struct LevelInfo {
    int w, h, off; float scale; int stride, goff;
    int cell = 0, gw = 1, gh = 1, cellOff = 0;
};

// Counts from the last detectAndCompute(), e.g. for adapting the FAST threshold.
struct OrbFrameStats {
    int fastThreshold = 0;                 // threshold used for this frame
    uint32_t corners = 0;                  // FAST corners (before NMS)
    uint32_t weakCorners = 0;              // per-cell thresholds: corners of minFastThreshold (in cells without others)
    uint32_t weakCells = 0;                // per-cell thresholds: cells that used minFastThreshold
    uint32_t candidates = 0;               // keypoints after NMS
    std::vector<int> candidatesPerLevel;   // keypoints after NMS, per level
    std::vector<int> quotas;               // keypoint quota per level
    bool overflow = false;                 // a GPU list was full (results incomplete)
    bool aborted = false;                  // abnormal frame (abortCornerFraction): no keypoints
};

// Raw keypoint as produced by the GPU, before per-level selection.
struct OrbCandidate {
    int   level, px, py;   // level and coordinates within that level
    float response;
    float angleRad;        // valid only if selected
    uint32_t desc[8];      // valid only if selected
    bool  selected;        // kept by the per-level selection (descriptor computed)
};

class OrbVk {
public:
    // The resolution is fixed per instance: buffers and the command buffer are built once.
    // allowCpuDevice permits falling back to a software rasterizer such as llvmpipe.
    OrbVk(int width, int height, const OrbParams& p = {}, bool allowCpuDevice = false);
    ~OrbVk();
    OrbVk(const OrbVk&) = delete;
    OrbVk& operator=(const OrbVk&) = delete;

    // Full ORB on an 8-bit grayscale image. Descriptors are 32 bytes per keypoint with the
    // same bit layout as cv::ORB.
    void detectAndCompute(const uint8_t* gray, int stride, std::vector<OrbKeypoint>& keypoints,
                          std::vector<uint8_t>& descriptors);

    // Runs the GPU work only (results stay in GPU-visible memory).
    void runGpu(const uint8_t* gray, int stride);
    // pyramidOnly with numImages = 2: left and right pyramids in one submission.
    void runGpuStereo(const uint8_t* left, const uint8_t* right, int stride);
    // pyramidOnly with numImages = n: n pyramids in one submission.
    void runGpuMulti(const uint8_t* const* images, int n, int stride);

    // Runtime tuning; each takes effect from the next run (the command buffer is re-recorded
    // once, which costs microseconds). n must not exceed OrbParams::maxFeatures.
    void setFastThreshold(int t);
    void setNumFeatures(int n);
    int  maxFeatures() const { return selCap_ - kSelSlack; }
    const OrbFrameStats& lastStats() const { return stats_; }

    // Level l of pyramid `image` from the last run, in GPU-visible memory (row stride
    // levels()[l].stride). Valid until the next run.
    const uint8_t* pyramidLevel(int l, int image = 0) const;

    // Read back results of the last run (for verification or custom selection).
    void readCandidates(std::vector<OrbCandidate>& out, bool* overflow = nullptr) const;
    void readPyramid(std::vector<uint32_t>& pyr, std::vector<uint32_t>& blur, int image = 0) const;

    const std::vector<LevelInfo>& levels() const { return levels_; }
    const OrbParams& params() const { return p_; }
    std::string deviceName() const { return devName_; }
    // GPU time of the last run from timestamp queries; -1 unless OrbParams::profile is set.
    double lastGpuMs() const { return gpuMs_; }
    // Per-stage GPU time of the last run (profile only): pyramid+blur+FAST, Harris,
    // NMS+selection, describe.
    const std::vector<double>& lastStageMs() const { return stageMs_; }
    // GPU time of every kernel group of the last run (profile only), in execution order:
    // (label, ms) from a timestamp after each barrier.
    const std::vector<std::pair<std::string, double>>& lastKernelMs() const { return kernelMs_; }
    // Hardware counters (debug, CMake option ORB_PERF_COUNTERS): with OrbParams::perfCounters = "name,name,..."
    // (VK_KHR_performance_query counter names, at most one pass), every kernel group's counter
    // values of the last run, in the order of lastKernelMs; OrbParams::perfEach: per dispatch instead (with
    // OrbParams::profile, also the timestamps). Empty otherwise, and always without ORB_PERF_COUNTERS.
    const std::vector<std::string>& perfCounterNames() const { return perfNames_; }
    const std::vector<std::pair<std::string, std::vector<double>>>& lastKernelCounters() const { return kernelCounters_; }

    // Keypoint quota per level (geometric distribution, as in OpenCV ORB).
    static std::vector<int> levelQuotas(const OrbParams& p, int nlevels);
    // Indices of the strongest candidates per level according to the quotas, in output order
    // (per level: response descending, then y, then x; with p.distribute: rank within the
    // grid cell first, see OrbParams). The GPU selection matches this exactly.
    static std::vector<int> selectIndices(const std::vector<OrbCandidate>& cands, const OrbParams& p,
                                          const std::vector<LevelInfo>& levels);
    // Builds the output keypoints and descriptors from the selected candidates.
    static void selectFeatures(const std::vector<OrbCandidate>& cands, const OrbParams& p,
                               const std::vector<LevelInfo>& levels, std::vector<OrbKeypoint>& kps,
                               std::vector<uint8_t>& desc);
    // Constant table shared by GPU and CPU reference: BRIEF pairs, umax, level geometry, quotas.
    static void buildConsts(std::vector<int32_t>& out, const std::vector<LevelInfo>& levels,
                            const std::vector<int>& quotas = {});

private:
    static constexpr int kSelSlack = 64;   // extra room in the selected-keypoint list
    struct Buf {
        VkBuffer buf = VK_NULL_HANDLE;
        VkDeviceMemory mem = VK_NULL_HANDLE;
        void* map = nullptr;
    };
    Buf  makeBuf(VkDeviceSize size);
    void freeBuf(Buf& b);
    VkPipeline makePipeline(const char* name, const uint32_t* spv, size_t bytes);
    void recordCommands();

    OrbParams p_;
    int w0_, h0_;
    std::vector<LevelInfo> levels_;
    size_t totalPixels_ = 0;   // pixels in the compact layout
    size_t totalBytes_  = 0;   // bytes of one packed GPU pyramid

    VkInstance       inst_  = VK_NULL_HANDLE;
    VkPhysicalDevice phys_  = VK_NULL_HANDLE;
    VkDevice         dev_   = VK_NULL_HANDLE;
    VkQueue          queue_ = VK_NULL_HANDLE;
    uint32_t         qfam_  = 0;
    uint32_t         memTypeIdx_ = 0;
    std::string      devName_;
    float            tsPeriod_ = 0.f;
    bool             tsSupported_ = false;

    Buf pyr_, blur_, corners_, score_, kps_, counter_, consts_, desc_, tabs_, sel_, fcand_;

    VkDescriptorSetLayout dsl_   = VK_NULL_HANDLE;
    VkPipelineLayout      pl_    = VK_NULL_HANDLE;
    VkDescriptorPool      dpool_ = VK_NULL_HANDLE;
    VkDescriptorSet       ds_    = VK_NULL_HANDLE;
    VkPipeline pReset_ = VK_NULL_HANDLE, pResize_ = VK_NULL_HANDLE, pResizeAny_ = VK_NULL_HANDLE, pBlur_ = VK_NULL_HANDLE,
               pFast_ = VK_NULL_HANDLE, pFastFull_ = VK_NULL_HANDLE, pHarris_ = VK_NULL_HANDLE, pNms_ = VK_NULL_HANDLE,
               pPost_ = VK_NULL_HANDLE, pThresh_ = VK_NULL_HANDLE, pSelect_ = VK_NULL_HANDLE, pTie_ = VK_NULL_HANDLE,
               pDesc_ = VK_NULL_HANDLE, pGridScan_ = VK_NULL_HANDLE,
               pGridScatter_ = VK_NULL_HANDLE, pGridTop_ = VK_NULL_HANDLE, pGridRank_ = VK_NULL_HANDLE, pRankThresh_ = VK_NULL_HANDLE,
               pFastMark_ = VK_NULL_HANDLE, pCellList_ = VK_NULL_HANDLE, pFastCells_ = VK_NULL_HANDLE;
    Buf grid_;
    int gridCells_ = 0;
    int abortCorners_ = 0;           // abortCornerFraction in corners (0 = off)              // total grid cells over all levels (distribute / per-cell FAST)
    VkCommandPool   cpool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_   = VK_NULL_HANDLE;
    VkFence         fence_ = VK_NULL_HANDLE;
    VkQueryPool     qpool_ = VK_NULL_HANDLE;
    VkQueryPool     qpoolFine_ = VK_NULL_HANDLE;   // profile: one timestamp per kernel group
    std::vector<std::string> fineLabels_;           // label of each fine timestamp after the first
    double gpuMs_ = -1.0;
    std::vector<double> stageMs_;
    std::vector<std::pair<std::string, double>> kernelMs_;
    // perfCounters: one performance query per kernel group, reset by its own command buffer (a performance
    // query pool cannot be reset in the command buffer that begins its queries)
    VkQueryPool     perfPool_ = VK_NULL_HANDLE;
    VkCommandBuffer perfResetCmd_ = VK_NULL_HANDLE;
    std::vector<uint32_t> perfIdx_;
    std::vector<std::string> perfNames_;
    std::vector<std::string> perfLabels_;
    std::vector<std::pair<std::string, std::vector<double>>> kernelCounters_;
    bool perfEach_ = false;                         // OrbParams::perfEach: one query / timestamp per dispatch
    std::map<VkPipeline, std::string> pipeNames_;   // labels for perfEach
    void setupPerfCounters();
    std::vector<int> xTab_, yTab_;   // per-level offsets into the resize lookup tables
    std::vector<bool> resizeFits_;   // per level: resize.comp (else resize_any.comp)
    int selCap_ = 0;                 // capacity of the selected-keypoint list
    bool dirty_ = false;             // parameters changed: re-record before the next run
    OrbFrameStats stats_;
};
// The CPU implementation with the same results is orb_cpu::Extractor (algorithms/orb_extractor/orb_cpu.hpp).
