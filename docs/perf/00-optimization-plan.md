# 優化計畫：NXP i.MX95 目標板（2026-11-24 提出，2026-11-25 更新狀態）

來源：使用者在 NXP 板上試過的改動（原 `claude_patch/`：`changes.patch`、`CHANGES.md`、`HANDOFF.md`），以及 NXP imx-hc-slam 對原版 ORB-SLAM3 的 patch（`01-nxp-orbextractor-core.diff`、`02+03-combined.diff`、`03-perf-parallel-descriptors.patch`）。這些原始檔已刪除；它們的內容整理在本文件與附錄。

相關文件：`01-test-report.md`（階段 0 與 2-A 的測試）、`02-math-plan.md`（數學化簡 M1–M10 與結果）、`03-parallel-plan.md`（並行化 P1–P6）。

## 目前狀態

| 項目 | 狀態 |
|---|---|
| 階段 0（`changes.patch`） | ✅ master `244bae1`。和 patch 的差異：補上 `local_map.hpp` 的宣告；位姿最佳化不平行（量起來反而慢）；`slam_merge_job` 也改用 `OrbStereoExtractor` |
| 2-A 純視覺位姿最佳化 | ✅ master `7992a0e`。**沒有平行化**，只改 double 與每邊一次投影（平行版 4.76 ms／幀，比依序的 4.02 ms 慢） |
| 數學化簡 M1–M6 | ✅ master `2f052d9`…`addd013`（不在本計畫內，見 `02-math-plan.md`）；M7–M10 不做 |
| 1-B、1-E | ⏸ 不需 GPU，等決定；1-E 要用板子上的量測 |
| 1-A、1-C、1-D、1-F、3-A～3-D | 📋 GPU 相關，只規劃 |
| 板子上的驗證 | ⏳ 見 `01-test-report.md`「還要在板子上做的」 |

以下是當初提出的計畫原文（數字為提計畫時的估計，實測結果見 `01-test-report.md`）。

## 判斷用的設計原則

主要依據 [DESIGN.md#原則](algorithms/orb_tracking/docs/DESIGN.md) 和 api_guide：
1. 邏輯和數學與 ORB-SLAM3 一致。你說過浮點捨入不必逐位元相同。
2. 不引入第三方函式庫。
3. 平行化用 `fw::parallel_for`，不自己開 `std::thread`。
4. 長時間的運算放在鎖外。

## 對計算結果的影響度分級（低 → 高）

| 級 | 意義 |
|---|---|
| **0** | 逐位元相同 |
| **1** | 只有排程或時序不同：lossless 下逐位元相同，real-time 下丟哪一幀可能不同 |
| **2** | 只有浮點捨入不同（float→double），inlier 數相同，位姿差約 1e-7 |
| **3** | 演算法行為改變，例如特徵點分布、門檻 |

---

## NXP imx-hc-slam 的手法：逐項判斷

| NXP 手法 | 這裡的現況 | 判斷 |
|---|---|---|
| FAST 改用 OpenCL，整個 ROI 一次算（`imx_ocl_FAST`） | Vulkan 已經做了 FAST，而且保留 per-cell 門檻，和 CPU 逐位元相同 | **不採用**。NXP 拿掉了逐格門檻，影響度 3；而且要依賴 NXP 的函式庫，違反原則 2 |
| 描述子用 NEON 或 OpenCL | 描述子已經在 GPU 上算 | **不需要**。頂多用在 CPU fallback，效益很小 |
| 各層用 `std::thread` 平行 | CPU backend 已經用 `parallel_for` 依層平行 | **不採用**。違反原則 3，而且已經有等效的做法 |
| 異質分工：level 0 在 GPU，level 1–7 在 NEON，同時跑 | 全部 8 層都在 GPU 上，兩眼和 matcher 擠同一條 queue | **概念可以借用**，見階段 3-B |
| 立體模式兩眼依序跑（MULTI_THREADS 時避免巢狀平行） | 你的 patch 讓兩眼平行，CPU backend 會改成依序 | 已涵蓋 |
| `fastRoi.clone()`（不連續緩衝區的修正） | Vulkan 自己管理緩衝區 | 不適用 |
| 語意分割 | 不在範圍內 | 不採用 |

**小結：** NXP 主要的加速手段（GPU 上的 FAST 和描述子）這裡已經用更嚴謹的方式做了。真正值得借用的只有「CPU 和 GPU 分工，讓 GPU queue 不塞車」這個概念。

---

## 計畫（依影響度由低到高排序）

### 階段 0：收進 `changes.patch`，並補做驗證（影響度 2）

| 項目 | 影響度 | 速度 | CPU | 記憶體 |
|---|---|---|---|---|
| 0-1 `OrbStereoExtractor`、`workers: 6` | 1 | matcher 7 → 18.6 Hz（實測） | ↑（兩眼同時跑） | 輸入佇列深度 3：每眼最多多留 2 張影像（約 0.36 MB × 2） |
| 0-2 GPU 資源 `write` → `read`，`submit` 加鎖 | 0 | 18.6 → 20.1 Hz（實測） | 持平 | 持平 |
| 0-3 KB8 改 double、IMU 位姿最佳化和 `searchLocalPoints` 平行化 | **2** | `track_local_map` 33 → 20 ms（實測） | ↑（`THREADS(4)`），總 CPU 時間反而下降 | 持平（`point_scratch` 以 id 索引，大約幾十 KB） |

還沒做的驗證（必做）：
- 每個序列跑一次 `lossless: true` 量 ATE：V1_02、room1、corridor1、outdoors4，和目前的 master 比。
- 跑 `tests/orb_extractor/*`、`tests/orb_matcher/*`。
- `slam_merge_job.graph.yaml` 也改用 `OrbStereoExtractor`，否則還會有左右各自丟幀的問題。

### 階段 1：影響度 0，逐位元相同

| 項目 | 內容 | 速度 | CPU | 記憶體 |
|---|---|---|---|---|
| **1-A 資料開始流動前先暖機 GPU** | 把 `Prepare()` 和 matcher 的 shader 初始化移到第一幀之前；fw 若沒有 init hook，就用 driver 的啟動延遲 | 消除開頭約 3 秒的暖機（兩眼 2.9 s、matcher 680 ms），**開頭的 12 次丟幀 → 約 0** | 不變 | 不變 |
| **1-B 延後釋放舊的 atlas 版本** | `sync_atlas` 的 `t.view = *atlas`：舊版本留到這一幀結束後才釋放（換手或放進回收佇列） | 拿掉 p95 18 ms 的尖峰 | 總量不變，只是移出關鍵路徑 | 尖峰時多留一個版本，大約多一份被修改區塊的 COW 副本 |
| **1-C 兩眼合成一次 GPU submit** | 在 `OrbStereoExtractor` 裡把左右眼的 command buffer 一起送出 | GPU 端：省一次 submit 和 fence 往返，queue 上兩眼不再和 matcher 交錯。估計每組省 0.5–1.5 ms，要在 Mali 上量 | 稍微下降（少一次等待） | 不變 |
| **1-D matcher 改用獨立的 compute queue** | 如果 Mali-G310 有超過一個 compute queue，matcher 的次像素 shader 就不必排在 extractor 後面 | matcher 10.4 → 約 3.5 ms（回到合併之前的水準），**p95 延遲 97 → 約 60–70 ms**（估計） | 不變 | 不變 |
| **1-E 執行緒預算** | 設 `GraphSpec::cores: 6`，重新分配 `THREADS`（extractor 2 + tracking 4 + local mapping / BA），查清多出的 6 條執行緒是不是 Mali 驅動開的 | 間接改善：減少超額訂閱造成的抖動 | **↓**（現在超出預算） | 每條執行緒的 stack 約 8 MB 虛擬記憶體，實際占用很少 |
| **1-F pyramid 用完就放掉** | 確認 `gpu_pyramid` 只有 matcher 用：matcher 做完次像素就 reset，不要隨 stereo frame 往下游傳；snapshot pool 的大小配合佇列深度 | 不變 | 不變 | 每組約 2 × 1.2 MB（752×480，8 層）。佇列裡同時留著的組數越多，省越多；pool 上限也可以跟著調小 |

1-D 要先查 `vkGetPhysicalDeviceQueueFamilyProperties`。如果只有一個 queue，這項就取消，改做 3-B。

### 階段 2：影響度 2，只有浮點捨入不同

| 項目 | 內容 | 速度 | CPU | 記憶體 |
|---|---|---|---|---|
| **2-A 純視覺位姿最佳化（`shared/pose_optimizer.cpp`）** | 和 0-3 同樣的做法：`project_jacobian_d` 共用 θ，固定切 8 塊用 `parallel_for` 平行，依塊的順序加總 | 參照 IMU 版的 23 → 13.7 ms：`optimize(pose)` 15 → 約 8 ms；`track_with_motion_model` p95 23 → 約 14 ms；`updateOdom` p95 26 → 約 15 ms | 牆鐘時間降、平行度升 | 不變 |
| 效果 | IMU 初始化前那 5 次丟幀 → 預期 0；`updateOdom` 尖峰消失 | | | |

驗證：`orb_tracking_pose_optimizer_test`（inlier 數相同），加上 lossless ATE。

### 階段 3：要先量測才決定（影響度 0–1）

| 項目 | 內容 | 影響度 | 預期 |
|---|---|---|---|
| **3-A Mali 上的 GPU 剖析** | 現有的 GPU_PROFILING 是 Pi 5 v3dv 的數據。要在 Mali-G310 上重量每個 kernel 的時間和 dispatch 開銷（timestamp query） | 0 | 先找出瓶頸，決定要不要做 3-B、3-C |
| **3-B NXP 式異質分工** | 小層（level 4–7，約占像素的 15%）改在 CPU 上用 `orb_cpu` 的 NEON 自動向量化，和 GPU 的 level 0–3 同時跑 | **1**：CPU 和 GPU 的關鍵點逐位元相同，只有角度最後幾位可能差，偶爾翻 1–2 個描述子位元 | GPU 工作 ↓ 約 15–25%，dispatch 數約砍半；但 CPU ↑。只有在 3-A 顯示 GPU 飽和、CPU 有餘裕時才做 |
| **3-C list kernel 加寬到 128** | Pi 5 的 microbenchmark 快 2 倍，但 Mali 的 warp 是 16 寬，不一定有效 | 0 | 不確定，看 3-A |
| **3-D 次像素改在 CPU 上算** | Mali 是 UMA，可以直接讀 GPU pyramid，但 mapping 可能是 uncached（matcher 的註解寫「slow mapping」） | 0 | GPU queue 少 8.7 ms；CPU 每組 ↑。可以當作 1-D 失敗時的替代方案 |

### 不建議（影響度 3）

調降 `nfeatures`、`nlevels`，或關掉 `per_cell_threshold`、`distribute`，都能明顯加速，但會改變特徵分布。MH_05 在 500 個特徵時已經會失去追蹤，所以都不列入。

---

## 預期的總效果（目標板，V1_02 real-time）

| 指標 | 現在（master） | 階段 0 之後 | 階段 0+1+2 之後（估計） |
|---|---|---|---|
| matcher 輸出 | 約 7 Hz | 20.1 Hz | 20 Hz（不再有開頭的損失） |
| matcher 延遲 p95 | — | 97 ms | 約 60–70 ms（1-C、1-D） |
| tracking 每幀時間 | 46.6 ms | 30–44 ms | 約 25–35 ms，沒有超過 50 ms 的尖峰 |
| tracking 丟幀 | — | 17 | **約 0–2** |
| CPU | 低（GPU 在等鎖） | ↑ | 和階段 0 差不多，1-E 之後不再超額訂閱 |
| 記憶體 | 基準 | +約 1 MB（佇列） | 比階段 0 少幾 MB（1-F），atlas 尖峰多一個版本（1-B） |

## 驗證方式

- **精度：** 每個設定在 V1_02、room1、corridor1、outdoors4 各跑一次 lossless。lossless 是確定性的，跑一次就夠。長序列看終點漂移。
- **頻率和丟幀：** 用 `rate: 1.0`、`lossless: false` 跑，配合 `tools/fw_stats.py`。
- **CPU 和記憶體：** 用 `pidstat -t -r`，或看 `/proc/<pid>/status` 的 VmHWM，比較每個階段前後。
- 每個階段各自一個 commit，每個 commit 附上 ATE 和 Hz 數字。

## 建議的順序

**0 → 1-A → 2-A → 1-B → 3-A → 再依剖析結果決定 1-C、1-D 或 3-B → 1-E、1-F**

前三步就能解決目前全部 17 次丟幀：開頭 12 次靠 1-A，IMU 初始化前 5 次靠 2-A。

---

## 附錄 A：板子上的基準量測（原 HANDOFF / CHANGES，2026-11-24）

環境：目標板 6 核、Mali-G310；EuRoC V1_02（20 Hz，1710 組），`rate: 1.0`，`graphs/slam_job.graph.yaml`。量法：`fw launch` 結束時 orb_matcher 印 Hz 與延遲；另一個終端 `python3 tools/fw_stats.py --interval 10`；逐節點計時是暫時的 `ORB_NODE_TIMER`（沒有進版控）。

### 前端（階段 0 的 0-1、0-2）

| | 原本 | 合成 stereo block | + GPU 改 read、兩眼平行 |
|---|---|---|---|
| matcher 輸出 | 約 7 Hz | 18.6 Hz | **20.1 Hz** |
| 配對完成 | — | 1483 | **1649**（96%，其餘丟在開頭約 3 秒 GPU 暖機） |
| 每組配對數 | — | 198.7 | 192.1 |
| 延遲中位數 / p95 | — | 51.5 / 69.4 ms | 46.3 / 97.3 ms |
| matcher 每組時間 | — | 3.45 ms | 10.4 ms（次像素 8.7 ms，排在 extractor 的 GPU 工作後面） |

原本 7 Hz 的原因：兩個 per-eye extractor 佇列深度 1 各丟各的幀（20 ×（12/20）² ≈ 7 Hz）；三個 block 都用 `write` 獨占 `gpu/Device`，連 CPU 工作也在鎖內；框只有 2 條 worker。

### tracking（階段 0 的 0-3）

| | 改之前 | 改之後 |
|---|---|---|
| `track_local_map` | 33.0 ms | 20.0 ms |
| ├ IMU 位姿最佳化 | 22.9 | 13.7 |
| ├ `searchLocalPoints` | 6.0 | 3.3 |
| └ `updateLocalMap` | 4.3 | 2.2 |
| tracking 每幀平均（每 10 秒區間） | 40–51 ms（整體 46.6） | 30–44 ms |
| tracking 丟幀 | 16–18 | 17（開頭 12，30–40 秒那段 5） |

CPU 時間和實際經過時間相同：是 tracking 本身算不完，不是被搶 CPU。

### 剩下的丟幀與尖峰（計畫 1-A、1-B、2-A 的依據）

- 開頭約 12 幀：第一幀 `make_frame` 500 ms；GPU 暖機約 2.9 秒（兩眼）、matcher 680 ms，之後一次湧進一批幀。
- IMU 初始化前約 5 幀：純視覺路徑 `track_with_motion_model` p95 23 ms 加 `optimize(pose)` 15 ms（2-A 已處理，待板子確認）。
- `publish_frame` 的 `updateOdom` p95 26 ms（local BA 移動地圖點後多做兩次位姿最佳化；2-A 已處理）。
- `sync_atlas` 的 `t.view = *atlas` p95 18 ms：釋放舊的 atlas 版本（1-B）。
- 警告 `box 'main' exceeds its thread budget`（多 6 條）：`parallel_for` 的池約 4 條，可能還有 Mali 驅動開的。不影響結果（1-E）。

## 附錄 B：GPU 共用的規則（階段 0 之後）

使用 GPU 的 block 以 `read` 共用 `gpu/Device`。前提是每個使用者都有自己的 command pool、command buffer、descriptor pool、fence，只有 queue 是共用的，送工作一律經過 `gpu::Context::submit`（內部加鎖）。`gpu::Device` 第一次建立裝置時加鎖。新增 GPU 程式碼要遵守這個前提（`types/gpu/context.hpp` 檔頭也有寫）。

## 附錄 C：NXP imx-hc-slam 的 patch 內容（原始檔已刪除，摘要）

- `01-nxp-orbextractor-core.diff`（NXP 對原版 ORB-SLAM3 的核心改動）：`Settings` 加 `COMPUTE_TYPE_{CPU, NEON, OPENCL, DSP, NEON_OPENCL}`；FAST 改呼叫 OpenCL 的 `imx_ocl_FAST`，對整個 ROI 一次算；描述子 `imx_ocl_ComputeOrbDescriptor`（OpenCL）或 NEON；各層的 `ComputeKeyPointsOctTree_level` 用 `std::thread` 平行（`ORBextractorMultiThreads.cc`）；語意分割過濾（class 15＝人的特徵丟掉）；CMake 連 `-lOpenCL`、EGL／GLES，範例程式改名為 `imx_*`。
- `02+03-combined.diff`（使用者在板上做的修正）：
  - 02：傳給 `imx_ocl_FAST` 的 ROI 先 `.clone()`（rowRange／colRange 的子矩陣不連續）。
  - 03：各層描述子 `ComputeDescriptors_level` 用 `std::thread` 平行（每層只寫自己的 `allKeypoints[level]`／`allDescriptors[level]`）。`COMPUTE_TYPE_OPENCL` 保持依序（各層共用同一個 `cl::Kernel`，setArg／enqueue 不是 thread-safe）；`COMPUTE_TYPE_NEON_OPENCL` 只有 level 0 用 GPU，所以 GPU（level 0）和 NEON（level 1..n）重疊。這是計畫 3-B「異質分工」的來源。
- `03-perf-parallel-descriptors.patch`：03 單獨的版本（02+03 已包含）。
- 逐項判斷見上面「NXP imx-hc-slam 的手法：逐項判斷」。
