# Algorithm notes (GPU ORB core, from orb_vk)

Paths below are those of the original orb_vk tree: src/ is now core/ and tests/, test/ is tests/data/. The ORB-SLAM3 drop-in header (orb_slam3/ORBextractor.h) is not part of orb_extractor.

GPU implementation of ORB (Oriented FAST + Rotated BRIEF) for embedded boards, written once
for the **Raspberry Pi 5** (VideoCore VII, Mesa v3dv) and the **NXP i.MX95** (Arm Mali-G310).
It uses only Vulkan 1.1 core compute with 32-bit integers and floats, so it needs no
vendor-specific extensions.

It also has a **pyramid-only mode**, for KLT optical flow, that can build several
pyramids (for example a stereo pair) in one submission.

## Results (Raspberry Pi 5, 512x512, GPU time)

| Workload | Time |
|---|---|
| Full ORB, 1000 keypoints | 15.7 ms |
| - pyramid + blur + FAST | 8.6 ms |
| - Harris | 2.2 ms |
| - NMS + per-level selection | 2.0 ms |
| - orientation + descriptors (selected keypoints only) | 2.9 ms |
| Pyramid only (10 levels) | 1.4 ms |
| Stereo pair, full ORB (left, then right) | 31.0 ms |
| Stereo pyramids, batched | 2.6 ms |

Frame times are very stable: p99 is within 0.5 ms of the median.

Every result is checked bit for bit against a CPU reference (`--verify`). Descriptors use
OpenCV's learned sampling pattern, so they can be matched against descriptors from `cv::ORB`.

## Build

Raspberry Pi OS (Bookworm or later):

```bash
sudo apt install cmake g++ libvulkan-dev glslang-tools mesa-vulkan-drivers vulkan-tools
sudo apt install libopencv-dev   # optional: ORB-SLAM3 front end (orbvk_slam, slam_demo)
vulkaninfo --summary        # should list "V3D 7.1"
cmake -S . -B build && cmake --build build -j4
```

NXP i.MX95 (Yocto BSP): the image needs the Vulkan loader and the Mali Vulkan driver.
`glslangValidator` runs only on the build host, so cross-compiling with the SDK works:

```bash
source /opt/fsl-imx-*/environment-setup-armv8a-poky-linux
cmake -S . -B build-imx -DGLSLANG=/usr/bin/glslangValidator && cmake --build build-imx
```

## Tools

```bash
./build/orb_demo test/coffee.pgm --verify            # compare GPU with the CPU reference
./build/orb_demo test/astronaut.pgm --iters 300       # timing: min / median / mean / p99 / max
./build/orb_demo in.pgm --out kp.txt --draw kp.ppm    # write keypoints / draw them
./build/orb_demo in.pgm --rotate-test                 # match against a 90-degree rotated copy
./build/pyr_bench left.pgm [right.pgm]                # pyramid-only mode: verify and time
./build/stereo_bench left.pgm right.pgm               # full ORB on a stereo pair (left, then right)
./build/slam_demo test/astronaut.pgm                  # ORB-SLAM3 front end: tests + adaptive threshold
```

`orb_demo` options: `-n` features, `-t` FAST threshold, `-l` levels, `-e` edge threshold
(border without keypoints, default 31 as in OpenCV; ORB-SLAM3 uses 19), `--distribute` /
`--cells k` spatially distributed selection (k grid cells per keypoint, default 1), `--min-t N`
per-cell FAST threshold (`-t` per cell, N in cells without a corner).

Input images are 8-bit binary PGM (`convert in.jpg -colorspace gray in.pgm`). The tools
refuse to run on a software device (llvmpipe) unless `--allow-cpu` is given.

## API

```cpp
#include "orb_vk.hpp"

OrbVk orb(640, 480);                     // fixed resolution: buffers and commands built once
                                         // (set OrbParams::profile for per-stage GPU timings)
std::vector<OrbKeypoint> kps;            // same fields as cv::KeyPoint
std::vector<uint8_t> desc;               // 32 bytes per keypoint, cv::ORB bit layout
orb.detectAndCompute(gray, stride, kps, desc);

OrbParams p;                             // pyramid only, e.g. for KLT
p.pyramidOnly = true;
p.nlevels = 9;                           // scales 1, 1/1.2, ..., 1/4.30
p.extraLevelScale = 8.f;                 // plus a 1/8 level
p.numImages = 2;                         // stereo pair in one submission
OrbVk pyr(640, 480, p);
pyr.runGpuStereo(left, right, stride);
```

The level scales are powers of 1.2 (level 4 is 1/2.07 and level 8 is 1/4.30), not powers
of 2. A KLT tracker must convert coordinates between levels with the real scale (see
`levels()[l].scale`); `cv::calcOpticalFlowPyrLK` assumes exactly 2x per level.

## Using it in ORB-SLAM3

`orbvk_slam` (built when OpenCV is found) provides `orbvk::ORBextractorVk`, which has the
interface of `ORB_SLAM3::ORBextractor`: the same constructor, `operator()` with the lapping
area, the scale getters and `mvImagePyramid` (views of the GPU pyramid, no copy).
`orb_slam3/ORBextractor.h` is a drop-in replacement for ORB-SLAM3's header:

1. Copy `orb_vk/orb_slam3/ORBextractor.h` over `ORB_SLAM3/include/ORBextractor.h`.
2. In `ORB_SLAM3/CMakeLists.txt`, remove `src/ORBextractor.cc`, then add
   `add_subdirectory(path/to/orb_vk orb_vk)` and `orbvk_slam` to `target_link_libraries(${PROJECT_NAME} ...)`.
3. Optional, in your `main()` before `ORB_SLAM3::System`, to read the `ORBextractor.vk.*` keys (link
   `orb_extractor_settings`, include `extractor_settings.hpp`):
   ```cpp
   orbvk::ORBextractorVk::defaultConfig() = orbvk::extractorConfigFromSettings(argv[2]);
   ```

Settings (the first five are ORB-SLAM3's; `vk.*` are optional, defaults shown in comments):

```yaml
ORBextractor.nFeatures: 450
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20            # threshold of every cell (with vk.cellThreshold: 0, of the first frame)
ORBextractor.minThFAST: 7             # threshold of cells without a corner (vk.cellThreshold: 0: lowest of the adaptation)
ORBextractor.vk.cellThreshold: 1      # per-cell threshold as ORB-SLAM3 (see "FAST threshold"); the
                                      # adaptation and retry below apply only with 0
ORBextractor.vk.maxThFAST: 40         # highest threshold of the adaptation (0 = iniThFAST)
ORBextractor.vk.adaptiveThreshold: 1  # 1
ORBextractor.vk.raiseRatio: 3.0       # raise while candidates > 3 x nFeatures
ORBextractor.vk.retryBelow: 0.5       # < 50% of nFeatures: run the frame again at minThFAST
ORBextractor.vk.maxFeatures: 0        # SetNumFeatures capacity without rebuild (0 = 2 x nFeatures)
ORBextractor.vk.edgeThreshold: 19     # 19 = ORB-SLAM3
ORBextractor.vk.distribute: 1         # spread keypoints over a grid (see "Spatial distribution")
ORBextractor.vk.cellsPerFeature: 1.0  # grid cells per keypoint of a level's quota
ORBextractor.vk.abortCornerFraction: 0     # abnormal frame (see "Abnormal frames"); 0 = off (default)
```

**FAST threshold.** ORB-SLAM3 runs FAST per 35-pixel cell with `iniThFAST` and falls back to
`minThFAST` in cells without corners. By default (`vk.cellThreshold: 1`) the wrapper does the
same within the frame, on the distribution grid (`OrbParams::minFastThreshold`, see
"Per-cell FAST threshold"); the threshold is then fixed and the adaptation below is off.

With `vk.cellThreshold: 0` the GPU uses one threshold per frame, adapted
from the previous frame: it drops by 1/5 when a level misses its quota, rises while there are
more than `raiseRatio x nFeatures` NMS candidates (by 1/8 above twice that ratio, else by 1),
and stays within `[minThFAST, maxThFAST]`. If a frame yields less than `retryBelow x
nFeatures` keypoints, it is run again at `minThFAST` immediately. With `maxThFAST` above
`iniThFAST`, textured scenes get faster: 512x512, 450 features, threshold 20 -> 40 converges
in about 10 frames and lowers the GPU time from 15.5 to 11.2 ms (`slam_demo`).

**Camera input.** `operator()(FrameSource& source, keypoints, descriptors, vLappingArea, frame,
timeoutMs)` takes the next frame from a camera object itself (`src/frame_source.hpp`: any source
of 8-bit gray frames with an id and a timestamp; no GStreamer or OpenCV dependency) and returns
the frame used (id, capture time, image for the tracking), or -1 if none came. The GStreamer
camera in `camera_gstream_test` implements it.

Run-time tuning: `SetFastThreshold(t)`, `SetNumFeatures(n)` (no rebuild up to `maxFeatures`),
`LastFrame()` (threshold, keypoints, candidates, corners, retried, cells and corners at
`minThFAST`, GPU ms).

Notes:
- The wrapper spreads keypoints over the image by default (`vk.distribute`, see below); with
  `vk.distribute: 0` each level keeps its strongest keypoints, which cluster on a few spots.
- One GPU instance per extractor. For stereo, ORB-SLAM3 runs the left and right extractor in
  two threads; each has its own Vulkan device and the GPU runs them one after the other.
- `mvImagePyramid` points into GPU-visible memory and is overwritten by the next call, like
  ORB-SLAM3's own pyramid buffers.

## Spatial distribution

ORB-SLAM3 spreads keypoints with a quadtree per level (one keypoint per node). orb_vk does the
same job with a fixed grid, entirely on the GPU (`OrbParams::distribute`, `--distribute` in
`orb_demo`, on by default in `ORBextractorVk`):

1. Each level's area inside the border is divided into about `quota x cellsPerFeature` square
   cells (at least 8 pixels), fixed at construction.
2. Keypoints are ranked within their cell (0 = strongest, capped at 63).
3. Each level keeps its quota in the order (rank, response): the strongest keypoint of every
   cell first, then every cell's second, and so on, so the quota is always met when there are
   enough candidates.

Unlike the quadtree the cells do not get smaller where the texture is dense, and the ranking
uses the Harris response (ORB-SLAM3: the FAST score). `--verify` checks the GPU selection
against the CPU (`OrbVk::selectIndices`) in this mode too.

Cost on a Pi 5 (512x512, 1000 features): NMS + selection 2.07 -> 3.07 ms, GPU total 15.8 ->
16.8 ms. The grid adds four list kernels (prefix sum of the per-cell counts that `nms.comp`
collects, scatter by cell, rank, boundary rank per level). Ranking reads the responses and
positions from contiguous per-cell lists (0.85 ms; following indices into the keypoint list was
1.3 ms). One workgroup per cell with the cell in shared memory was 2x slower: V3D shared memory
goes through the TMU like global memory, and every cell costs two barriers.

## Per-cell FAST threshold

`OrbParams::minFastThreshold` (`--min-t N` in `orb_demo`): a grid cell (the distribution grid,
also built when `distribute` is off) that has a corner of `fastThreshold` keeps only those; a
cell without uses `minFastThreshold`. This is ORB-SLAM3's per-cell fallback, decided within
the frame, so low-texture regions get corners without lowering the threshold of the whole
image (whose extra corners would all go through Harris and NMS).

1. FAST pass 1 at `fastThreshold`, unchanged.
2. `fast_mark.comp` marks the cells that got a corner, `cell_list.comp` lists the others.
3. `fast_cells.comp` runs the pre-test at `minFastThreshold` over the listed cells only (one
   workgroup per cell), and `fast_full.comp` runs again on those blocks (pass 2).

`--verify` checks the corners and keypoints against the CPU reference in this mode too.

Cost on a Pi 5 (512x512, 1000 features, distribute, GPU total): threshold 20: 18.3 ms; per
cell 20 / 7: 21.6 ms; whole image at 7: 30.3 ms. Pass 2 costs what the low-texture area costs:
the cells to rescan (on these test images large flat backgrounds) are scanned a second time.
In a nearly textureless frame every cell is rescanned, which roughly doubles the FAST stage. A
higher `fastThreshold` sends more cells to pass 2, so raising it is not faster in this mode.

Designs measured slower: testing every corner of the low threshold against both thresholds
in one pass and filtering before Harris (+7 ms: the second comparison set halved the QPU
threads, and every low-threshold corner was handled); pass 2 as a per-pixel dispatch over the
whole pyramid that skips marked cells (+2.7 ms even when every thread exited at once, since V3D
cost follows the number of threads). The cell of a pixel is computed with a multiply and shift
instead of an integer division (V3D has no divider), exact for cells of at most 511 pixels.

## Abnormal frames

Images like sensor noise or dense fine texture give 10x the FAST corners of a normal scene
(15-35 % of the pyramid's pixels instead of 1-3 %) and, without a limit, 3-4x the time
(720x540 stereo pair: 156-190 ms instead of ~45 ms). With `OrbParams::abortCornerFraction`
(`ORBextractor.vk.abortCornerFraction`, off by default since 2026-10-21, `--abort F` in
`orb_demo` / `extract_profile`) such a frame is aborted on the GPU: `fast_full.comp` stops as soon as the corner
count passes the limit, every later kernel returns at once (`aborted()` in `common.glsl`),
and the frame yields no keypoints (`OrbFrameStats::aborted`, `FrameInfo::aborted`).

Why it is off by default: with CLAHE, real outdoor scenes (bare trees against the sky, TUM-VI
outdoors4) reach 10 % and sat right at the limit for tens of seconds, so frame sets alternated
between the slowest case and no keypoints at all (up to 5.8 s in a row). Without the limit the
time stays bounded by the corner buffer (`OrbParams::maxCorners`, pyramid pixels / 4): on that
woods segment a 512x512 stereo frame set took at most 81 ms (84 ms with the limit, because one
image of the pair often still ran in full), and synthetic worst cases (pure or blurred noise,
buffer full) 95-138 ms, still with keypoints.

| 720x540, 500 features, per-cell 20/7 + distribute | GPU per image |
|---|---|
| normal (synthetic, coffee) | 23-24 ms (+0.3 ms for the early-stop check) |
| pixel noise / 4-px blobs, without abort | 79 / 95 ms |
| pixel noise / 4-px blobs, aborted | 17.5 / 14.9 ms |

EuRoC V1_02_medium (1710 stereo pairs, 720x540): FAST corners per frame 0.3-4.3 % of the pyramid
(p99 3.3 %), no frame aborted. `--verify` checks an aborted frame's images and that it has no
keypoints.

## Pipeline

| Stage | Shader | Work |
|---|---|---|
| 0 | `reset.comp` | zero counters and selection histograms |
| 1 (per level) | `resize.comp` | bilinear 1/1.2 downscale, fixed-point weights from a lookup table, 4x2 pixels per thread from 3-word source windows; also fills the row padding with the edge pixels (`resize_any.comp`: byte loads, for scale ratios above about 2.3) |
| | `blur.comp` | 7x7 separable Gaussian, both passes in one shader, 4x2 pixels per thread; image borders come from the row padding |
| | `fast.comp` | FAST-9 pre-test, 4 pixels per thread (packed compass test); appends passing blocks to a list |
| 1 (end) | `fast_full.comp` | full FAST-9 test of the listed blocks, all levels in one dispatch; appends to a corner list |
| | `fast_mark.comp`, `cell_list.comp`, `fast_cells.comp`, `fast_full.comp` | per-cell threshold only: FAST pass 2 at the low threshold in cells without a corner |
| 2 | `harris.comp` | Harris response for each corner (word loads, column sums) |
| 3 | `nms.comp` | 3x3 non-maximum suppression over the corner list (distribute: also counts keypoints per grid cell) |
| | `grid_scan.comp`, `grid_scatter.comp`, `grid_rank.comp`, `rank_thresh.comp` | distribute only: group keypoints by cell, rank them within the cell, boundary rank per level |
| | `post_nms.comp` | per-level response histograms; resets the score map at this frame's corners |
| | `thresh.comp` | per level, the histogram bin where the keypoint quota is reached |
| | `select.comp` | keeps keypoints above that bin, collects the ones inside it |
| | `select_tie.comp` | ranks the boundary-bin keypoints exactly (response, then y, then x) |
| 4 | `describe.comp` | orientation and 256-bit rotated BRIEF, only for the selected keypoints |

The GPU selects exactly the keypoints that the CPU selection (`OrbVk::selectIndices`, same
per-level quotas as OpenCV) would keep, so descriptors are computed only for those (about a
quarter of the NMS survivors). `--verify` checks the selection against the CPU.
Both SoCs have unified memory, so all buffers are mapped directly and no staging copies are made.

Every pyramid row has 4 bytes before pixel 0 and at least 4 after the last pixel that repeat
the edge pixels (`LevelInfo::goff` points at pixel 0). With them the blur needs no border code.

## Register pressure on the Pi 5

On the Pi 5 the V3D compiler runs 4 threads per QPU while a shader needs about 32 live
registers or fewer, 2 threads up to about 64, and spills to memory beyond that
(`V3D_DEBUG=shaderdb`). The hot shaders were chosen by measuring alternatives on a Pi 5
(512x512, ms):

| Shader | Alternatives measured | Used |
|---|---|---|
| blur | 4x4 block 5.84, 4x2 6.13, 4x1 7.52 | 4x2 with the horizontal pass on two 16-bit lanes, 5.64 |
| FAST | 1 pixel per thread 6.54 (4 threads, not faster), 4 pixels per thread sharing word loads 6.20 | 4 pixels per thread, packed compass pre-test with 5 loads instead of 21, 5.58 |
| Harris | 9x9 bytes loaded up front 5.52, rows loaded late 5.48 | word loads + column sums, 2.19 |

The blur originally handled the left and right image border with a separate code path in
the same kernel. Although only the first and last threads of a row took it, the V3D compiler
kept both paths: 1486 instructions with 16 spills, against 485 and none for the interior path
alone. Repeating the edge pixels in the row padding removed the border path: pyramid + blur
+ FAST went from 13.5 to 9.9 ms (512x512) with bit-identical results. Tried and not used:
a separate border pass (still 0.6-1 ms of extra dispatches), branch-free fix-ups of the edge
words (+160 instructions for every thread), and per-level specialization constants for blur
and FAST (about 0.1 ms faster, but 16 pipelines: cold start 0.46 -> 1.5 s). Optimizing the
SPIR-V (`glslangValidator -Os`, `spirv-opt -O`) does not change the V3D machine code.

On V3D the cost of the image kernels follows the number of threads more than their work
(roughly 10-18 us per 1000 threads). Resize went from 4 to 4x2 pixels per thread (and three
word loads per source row instead of 16 byte loads): pyramid-only mode 2.2 -> 1.4 ms, stage 1
-1.2 ms at 720x540; 4x4 and 4x8 were no faster. FAST used to run the full test inside the
per-level kernel: a SIMD group of 16 threads (64 pixels) took that path as soon as one pixel
passed the compass pre-test, which in textured areas is almost always. The pre-test now lists
the passing blocks and `fast_full.comp` tests only those (stage 1 -1.4 to -2.3 ms at
720x540, threshold 20-30). It must process the list in contiguous parts per workgroup: a
grid-stride loop mixed the corner order across levels and made describe 20% slower.
Not faster for describe: 16 threads per keypoint (2.2 instead of 1.5 ms, it is bound by its
1260 byte loads, not by parallelism) and an unrolled orientation with word loads (the
compiler hoists all 250 loads: 204 spills, 4.5 ms).

To see the compiler statistics of every shader on a Pi:

```bash
MESA_SHADER_CACHE_DISABLE=true V3D_DEBUG=shaderdb ORB_LIST_PIPELINES=1 \
  ./build/orb_demo in.pgm --iters 1 --no-profile 2>&1 | grep -A1 "pipeline"
```

Scheduling stage 1 differently (whole pyramid first, then one dispatch over all levels, or
blur and FAST fused into one kernel) was measured slower on a Pi 5 and was removed.

## Stereo

For full ORB, call `detectAndCompute` for the left and then the right image (one instance
can serve both). On a Pi 5 a 512x512 pair takes 31.0 ms of GPU time. Processing both images
as one list of levels in a single submission was tried (before the blur change) and measured
slightly slower: the GPU is compute bound, so there is no idle time to recover.
For KLT, `pyramidOnly` with `numImages = 2` does batch the pair (2.6 ms instead of 2.8 ms).
`test/stereo_left.pgm` / `test/stereo_right.pgm` are a synthetic pair (12-pixel disparity).

## Frame structure: one submission, no GPU stalls

- One command buffer per frame, recorded once and resubmitted.
- No `vkCmdDispatchIndirect`: list kernels run a fixed number of workgroups and loop over
  their list. On v3dv an indirect dispatch is a CPU job that waits for the GPU to go idle.
- No `vkCmdFillBuffer`: counters are cleared by `reset.comp`, and the score map is restored to
  zero by clearing only the entries written this frame (v3dv runs fills as separate TFU jobs).
- GPU timestamps only with `OrbParams::profile`; on v3dv each timestamp is also a CPU job.

On a Pi you can watch the compute queue ("csd") with the `gpu_stats` file of the v3d driver;
nvtop only counts the render queue and shows 0% for compute work.

## Portability rule: no run-time indexed arrays in shaders

On the Pi 5, a private or constant array indexed with a run-time index is spilled to
scratch memory. The first version used such arrays and was 5x slower overall. Measured on a
Pi 5 at 512x512:

| Shader | With arrays | Unrolled |
|---|---|---|
| blur | 46.5 ms | 5.8 ms |
| FAST | 79.9 ms | 6.1 ms |
| Harris | 9.4 ms | 4.4 ms |

`blur.comp`, `fast.comp` and `harris.comp` are therefore generated fully unrolled by
`tools/gen_shaders.py`. The generated files are committed. After editing the script, run
`python3 tools/gen_shaders.py` to regenerate them. The software Vulkan device (llvmpipe)
does not show this slowdown, so measure on the target board.

## Differences from OpenCV ORB

- NMS uses the Harris score directly (OpenCV runs NMS on the FAST score, then ranks by Harris).
- Blur borders use clamp-to-edge instead of reflect-101. This only affects pixels within
  the 31-pixel border, where no keypoints are detected.
- Per-level selection keeps the strongest responses (as OpenCV). The optional grid
  distribution plays the role of ORB-SLAM's quadtree (see "Spatial distribution").

## Possible next steps

- A GPU KLT tracker for the pyramid-only mode, following the same no-array rule.
