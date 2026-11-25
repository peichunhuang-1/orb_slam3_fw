# 測試報告：階段 0（`changes.patch`）＋ 2-A（2026-11-25）

## 一句話結論

`changes.patch` 已套用（另外補了一個漏掉的宣告）。2-A（純視覺位姿最佳化改用 double、每條邊只投影一次）也做好了。在 17 個序列上，軌跡精度沒有系統性的變化：除了一個序列剛好多關了一次 loop，各序列的 ATE 有好有壞，幾何平均是 master 的 0.96–0.97 倍。tracking 每幀時間 6.3 → 4.05 ms（−36%），整個行程的 CPU 時間 −9 ~ −12%。

**已 commit 進 master：階段 0 `244bae1`、2-A `7992a0e`（2026-11-25）。**

## 測試環境與限制

- 這台是 devcontainer：8 核 aarch64，**沒有 GPU**（只有 llvmpipe，block 不接受）。所以：
  - extractor 走的是 CPU backend；**階段 0 的 GPU 改動（`gpu` 改 `read`、`submit` 加鎖、stereo block 的 GPU 路徑）在這裡沒有執行到**，只能在板子上驗證（你在板子上量過 20.1 Hz）。
  - `orb_matcher_subpixel_gpu_test` 在這裡跑不了，master 也一樣。
  - 時間是這台的。板子是 6 核 Cortex-A55，絕對值會不同，相對的方向應該一致。
- 圖都是 `workers: 6`，和 NXP 上一樣。
- 比較對象：master（HEAD `914a701`，在 scratchpad 的 worktree 另外編譯）。

## 改了什麼

### 階段 0：使用者在板上做的 `changes.patch`（內容見 `00-optimization-plan.md` 附錄 A）

照原樣套用：`OrbStereoExtractor`、GPU 資源改 `read`、`gpu::Context::submit` 加鎖、KB8 的 `project_jacobian_d`、`searchLocalPoints` 平行化、`updateLocalMap` 換資料結構、IMU 位姿最佳化改 double、`workers: 6`、`OrbExtractor` 的 `THREADS(1)`。

和 patch 不一樣的三處：

| | 內容 | 原因 |
|---|---|---|
| 1 | **補上 `local_map.hpp` 的宣告**（`searchLocalPoints` 多一個 `const ParallelFor& par = {}` 參數、include `parallel.hpp`） | patch 裡漏了這個檔案（CHANGES.md 有寫到，但 diff 沒有），直接套用會編譯失敗 |
| 2 | **IMU 位姿最佳化不放到執行緒上**：`track_local_map` 不再把 `par` 傳給 `optimizePoseInertial*`；`pose_inertial_optimizer` 的平行程式保留，只是沒人傳 `par` | 實測平行版比依序版慢（見下面「平行化的量測」）。加速來自 double 和共用投影，不是平行 |
| 3 | `graphs/slam_merge_job.graph.yaml` 也改用 `OrbStereoExtractor`、`workers: 6` | 和 `slam_job` 一致，避免它也有左右各自丟幀的問題 |

`searchLocalPoints` 的平行化（每幀 2 個區段，工作量夠大）照 patch 保留。

### 2-A：純視覺位姿最佳化（`shared/pose_optimizer.cpp`）

- 投影和 Jacobian 改用 `project_jacobian_d`（double、共用 θ、不呼叫 `atan2`）。
- 每次迭代：誤差、Jacobian、H、b、robust χ² 一次算完（原本是先算誤差、再算 χ²、再為了 Jacobian 重新投影一次）；試探步只算誤差和 χ²。
- 也接受 `ParallelFor`（固定切 8 塊、依塊的順序加總，結果和執行緒數無關），但 tracking 的葉節點都不傳，因為量過會變慢。
- 影響範圍：`track_with_motion_model`、`track_reference_keyframe`、`relocalize`、`track_local_map`（IMU 初始化前）、`publish_frame` 的 `updateOdom`。
- 新測試：`pose_optimizer_test` 加上「4 執行緒的結果和依序的逐位元相同」。

### 檔案

17 個檔案，+461 / −177，另外新增 `algorithms/orb_tracking/shared/parallel.hpp`（來自 patch）。

## 單元測試

`ctest`：26 個裡 25 個通過；`orb_matcher_subpixel_gpu_test` 因為沒有 GPU 失敗（master 也一樣）。

2-A 和 ORB-SLAM3 參考值比對（`orb_tracking_pose_optimizer_test`，8 個情境）：

| 情境 | 邊數 | inlier（ORB-SLAM3） | outlier 標記不同 | 旋轉差 | 位移差 |
|---|---|---|---|---|---|
| nominal | 277 | 247（247） | 0 | 3.5e-8 rad | 6.0e-8 m |
| dense | 712 | 560（560） | 0 | 4.3e-7 | 8.5e-7 |
| sparse | 43 | 38（38） | 0 | 6.1e-8 | 4.2e-7 |
| heavy_outliers | 268 | 151（151） | 0 | 1.1e-8 | 3.0e-8 |
| far | 274 | 235（235） | 0 | 8.6e-8 | 1.7e-6 |

（另外 3 個情境同樣全部相同。）4 執行緒和依序的結果逐位元相同。

## 軌跡精度（ATE）

### lossless，17 個序列

`lossless: true`、`rate: 0`，每個設定跑一次（lossless 是確定性的，重跑結果完全相同）。三個版本：**master**、**s0**（只有階段 0）、**opt**（階段 0 + 2-A）。單位 m。

| 序列 | odom master | s0 | opt | map master | s0 | opt |
|---|---|---|---|---|---|---|
| MH_01 | 0.174 | 0.168 | 0.159 | 0.050 | 0.075 | 0.045 |
| MH_02 | 0.140 | 0.131 | 0.143 | 0.035 | 0.053 | 0.044 |
| MH_03 | 0.349 | 0.328 | 0.295 | 0.066 | 0.069 | 0.066 |
| MH_04 | 0.378 | 0.363 | 0.378 | 0.118 | 0.115 | 0.114 |
| MH_05 | 0.853 | 0.868 | 0.834 | 0.457 | 0.269 | 0.323 |
| V1_01 | 0.064 | 0.066 | 0.072 | 0.041 | 0.040 | 0.041 |
| V1_02 | 0.105 | 0.107 | 0.075 | 0.024 | 0.024 | 0.024 |
| V1_03 | 0.076 | 0.094 | 0.083 | 0.027 | 0.025 | 0.025 |
| V2_01 | 0.053 | 0.052 | 0.048 | 0.035 | 0.036 | 0.030 |
| V2_02 | 0.119 | 0.105 | 0.140 | 0.029 | 0.037 | 0.030 |
| V2_03 | 0.169 | 0.150 | 0.138 | 0.136 | 0.063 | 0.095 |
| room1 | 0.035 | 0.033 | 0.036 | 0.016 | 0.016 | 0.027 |
| room2 | 0.034 | 0.038 | 0.034 | 0.014 | 0.014 | 0.014 |
| room3 | 0.076 | 0.060 | 0.065 | 0.011 | 0.011 | 0.011 |
| room4 | 0.046 | 0.047 | 0.023 | 0.015 | 0.013 | **0.068** |
| room5 | 0.041 | 0.041 | 0.047 | 0.017 | 0.017 | 0.015 |
| room6 | 0.033 | 0.041 | 0.028 | 0.008 | 0.008 | 0.008 |

odom：整條軌跡（`tools/ate.py`）。map：`--frame map --from 20`。

**和 master 的比值（幾何平均，不含 room4）：**

| | odom | map | 變差的序列數 |
|---|---|---|---|
| s0 / master | 0.991（各序列的分散 ×/÷1.13） | 0.977（×/÷1.34） | odom 6/16、map 6/16 |
| opt / master | **0.961**（×/÷1.15） | **0.974**（×/÷1.23） | odom 8/16、map 6/16 |

**room4**：opt 那次關了一次 loop（kf 144 和 kf 91），master 和 s0 都沒有；關 loop 之後 map ATE 變差。這是已知的問題（之前的評估裡 room4 關 loop 之後也到過 0.215 m），捨入的差異只是決定了這次有沒有偵測到。所以從統計裡拿掉，單獨列出。

**怎麼解讀**：SLAM 對很小的數值差異很敏感，捨入層級的改動會讓軌跡往兩個方向走，就像一次隨機擾動。光是階段 0（IMU 位姿最佳化改 double）就讓 map ATE 在各序列間有 ×/÷1.34 的分散，例如 MH_02 0.035 → 0.053、V2_03 0.136 → 0.063。2-A 的分散沒有比它大，平均值也沒有往壞的方向偏。**依「沒差很多就過」的標準，2-A 通過。**

### TUM-VI 長序列（lossless，終點漂移）

| | master | opt |
|---|---|---|
| corridor1 終點漂移 odom / map | 2.80 / 0.175 m | 2.72 / 0.159 m |
| outdoors4 終點漂移 odom / map | 59.8 / 22.1 m | 61.1 / 25.7 m |

outdoors4 本來就很分散（之前的紀錄：lossless 17.6 m，real-time 一次 29.8 m，之前的 artifact 8–17 m），一次結果不能代表變化。

### real-time，各 3 次（`rate: 1.0`，`lossless: false`）

| | odom master | odom opt | map master | map opt |
|---|---|---|---|---|
| V1_02 | 0.096 / 0.097 / 0.097 | **0.073 / 0.065 / 0.063** | 0.026 / 0.026 / 0.028 | **0.021 / 0.023 / 0.024** |
| room1 | 0.029 / 0.040 / 0.034 | 0.042 / 0.038 / 0.037 | 0.0147 / 0.0148 / 0.0153 | 0.0190 / 0.0186 / 0.0186 |

這台機器 CPU 夠用，real-time 下也幾乎是確定性的，所以 3 次之間的差異很小（約 ±0.3 mm），**不能拿來估計對擾動的敏感度**，那要看上面 17 個序列的分散。room1 在 real-time 下和 lossless 一樣稍微變差，V1_02 則變好。

## 效能

### tracking

| | master | opt |
|---|---|---|
| tracking 每幀平均（V1_02，real-time） | 6.22–6.40 ms | **4.05–4.08 ms**（−36%） |
| tracking 每幀平均（room1，real-time） | 5.57–5.61 ms | **3.72–3.84 ms**（−33%） |
| V1_02 tracking 處理的幀數（共 1710） | 1691–1693 | 1695 |
| V1_02 extractor 丟幀（每眼） | 3–6 | 0 |

### 各 block（V1_02，real-time，平均每次）

| | master | opt |
|---|---|---|
| extractor | 每眼 5.0–5.2 ms（兩個 block 各 `THREADS(4)`） | 兩眼 13.2 ms（一個 block，每眼一條執行緒） |
| matcher | 0.80 ms | 0.77 ms |
| matcher 延遲中位數 / p95 | 20.2 / 23.5 ms | **13.3 / 20.8 ms** |
| local mapping（每個 keyframe） | 11.9–12.2 ms | 11.1–11.3 ms |
| `ba_solver`（local BA） | 20.9–22.9 ms | 20.4–20.5 ms |

**CPU backend 的 extractor**：兩眼放在同一個 block 之後，每眼只有一條執行緒（`OrbExtractor` 是 `THREADS(1)`，stereo block 裡層的 `parallel_for` 也是依序），所以 handler 時間從每眼約 5 ms 變成兩眼合計 13 ms。不過兩眼同時跑，matcher 的延遲反而變短。板子上 extractor 走 GPU，不受影響；但如果板子上 GPU 失敗、`auto` 退回 CPU，抽特徵的延遲會是現在的 2.6 倍（仍在 50 ms 以內）。

### CPU 與記憶體（整個行程）

| | master | opt |
|---|---|---|
| CPU user 時間，V1_02 real-time | 85.4–87.5 s | **78.4–79.1 s**（−9%） |
| CPU user 時間，room1 real-time | 130.0–130.3 s | **113.6–115.0 s**（−12%） |
| CPU user 時間，lossless（V1_02、V2_02、MH_03、room1、corridor1、outdoors4 合計） | 1377 s | **1225 s**（−11%） |
| 最大 RSS，V1_02 | 402–405 MB | 454–456 MB（+50 MB） |
| 最大 RSS，V1_02，`MALLOC_ARENA_MAX=2` | — | **384 MB** |

**記憶體的 +50 MB 不是資料變多**。worker 從 2 條變 6 條，glibc 會給每條配置記憶體的執行緒各自一個 malloc arena。設 `MALLOC_ARENA_MAX=2` 之後，最大 RSS 是 384 MB，比 master 還低，ATE 逐位元相同（0.0753）。板子上可以在啟動的環境變數設 `MALLOC_ARENA_MAX`（2–4），省下這部分。

### 平行化的量測（決定不把最佳化器放到執行緒上的依據）

| | 依序 | 4 執行緒 |
|---|---|---|
| `optimizePose`，277 條邊（microbenchmark） | 0.89 ms（master 1.80 ms） | 1.72 ms |
| `optimizePose`，712 條邊 | 2.19 ms（master 5.37 ms） | 2.90 ms |
| IMU 位姿最佳化，nominal | 0.95 ms（master 1.24 ms） | 1.83 ms |
| graph 裡的 tracking 每幀，V1_02 | **4.02 ms** | 4.76 ms |

一次最佳化大約有 80 個平行區段，每個只有 10–30 µs 的工作量，叫醒執行緒的成本比省下的時間還多。加速全部來自 double 加上每條邊只投影一次：純視覺 2.0–2.4 倍，IMU 1.3 倍。

### 其他

- `box 'main' needs 10 threads`（6 + extractor 1 + `track_local_map` 3）：這台 8 核，所以只是警告；板子 6 核會更緊。同一棵樹的葉節點，`THREADS` 取最大值而不是相加（另外加了 `THREADS(4)` 的葉節點後，數字沒有變）。
- `tools/fw_stats.py`：box 結束時，最後一則 stats 偶爾不含任何 block，summary 就只印標題（6 次裡遇到 1 次）。修法是只在 `stats["handlers"]` 不是空的時候更新 `last`。這次是改在 scratchpad 的複本，倉庫裡的工具沒動。

## 還要在板子上做的

1. 板子上跑一次 `slam_job`：確認 GPU 路徑（`read` 共用、`submit` 加鎖）在兩眼同時 submit 時正常，matcher 仍是 20 Hz。
2. tracking 每幀時間與丟幀數：HANDOFF 的 17 幀裡，IMU 初始化前的 5 幀預期會因 2-A 消失；開頭 12 幀是 GPU 暖機（1-A，還沒做）。
3. `MALLOC_ARENA_MAX=2` 時的 RSS 與 CPU。
4. 1-E（`cores: 6`、`THREADS` 分配），要用板子上的量測決定。

## 還沒做

- 1-B（延後釋放舊的 atlas 版本）、1-E：等你決定。
- 1-A、1-C、1-D、1-F、3-x：GPU 相關，依你的指示只規劃。
- commit：已完成（見開頭）。

## 重現

腳本當時放在 Claude session 的 scratchpad，沒有進版控，現在可能已經不在了（`/tmp/claude-1000/-workspaces-orb-slam3-fw/f425f824-e9ab-4677-9834-552514be2942/scratchpad/`，重開機可能會清掉）：`eval.sh`（跑一個序列並記錄 ATE、CPU、RSS、`fw_stats`）、`summary.py`、`compare.py`，以及 `logs/` 底下每次執行的紀錄。
