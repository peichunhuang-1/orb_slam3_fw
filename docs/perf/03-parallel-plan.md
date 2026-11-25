# 可並行的地方：計畫（2026-11-25）

只是計畫，程式還沒改。GPU（extractor 的 Vulkan 與 matcher 的次像素 shader）不在範圍內。

## 先要知道的兩件事

### 1. 太小的平行區段反而變慢（這次量到的）

2-A 原本照 `changes.patch` 的做法，把位姿最佳化每次迭代的邊切 8 塊，放到 `parallel_for` 上。實測結果：

| | 依序 | 4 執行緒 |
|---|---|---|
| `optimizePose`，277 條邊（microbenchmark） | 0.89 ms | 1.72 ms |
| `optimizePose`，712 條邊 | 2.19 ms | 2.90 ms |
| IMU 位姿最佳化，約 200 條邊 | 0.95 ms | 1.83 ms |
| 整個 graph，V1_02：tracking 每幀平均 | **4.02 ms** | 4.76 ms |

- 一次最佳化大約有 80 個平行區段（4 輪 × 10 次迭代 × 每次 1–2 個），每個區段只有 10–30 µs 的工作。叫醒執行緒、等它們做完的成本和工作量差不多，甚至更多。
- `fw::parallel_for` 是用 condition variable 的執行緒池（`/opt/fw/include/fw/call.hpp`），每個區段都要叫醒、等待一次。
- **規則：一個平行區段的工作量，至少要是執行緒數 × 約 100 µs，才可能划算。** 下面每一項都標了估計的區段大小。

所以這次 tracking 的兩個位姿最佳化維持依序（`searchLocalPoints` 每幀只有 2 個區段、工作量足夠，所以保留平行）。

### 2. 6 核上的執行緒預算已經超過

`slam_job` 的 box 現在要 10 條執行緒：`workers: 6` + `OrbStereoExtractor` 多 1 條 + `track_local_map` 的 `THREADS(4)` 多 3 條（同一棵樹的葉節點取最大值，不是相加，這次量過）。NXP 只有 6 核。

- 再加平行化，等於讓更多執行緒去搶 6 個核。tracking 是關鍵路徑（每幀 50 ms 的期限），local mapping、BA、loop closing 只要跟得上就好。
- **任何新的平行化之前，先做 1-E（設 `cores: 6`，重新分配 `THREADS`）**，並在板子上量 tracking 的每幀時間有沒有變差。

## 現在的並行

fw 已經是「block 之間的管線並行」：driver、extractor、matcher、IMU 預積分、tracking、local mapping、ba_solver、gba_solver、loop closing 各自是 block，box 的 6 個 worker 讓它們同時跑。local mapping 的 BA 已經在鎖外（`ba_solver` block），tracking 讀 atlas 不必等（`policy: versioned`）。

block 內部的平行，目前只有：

| 位置 | 方式 |
|---|---|
| `OrbStereoExtractor` | 兩眼同時跑（`THREADS(2)`）；CPU backend 時改成依層平行 |
| `track_local_map` 的 `searchLocalPoints` | 視錐檢查、候選搜尋（`THREADS(4)`） |

## 結論

| # | 項目 | 級 | 區段大小（估計） | 預期效果 | 前提 |
|---|---|---|---|---|---|
| P1 | **local BA：nls 的殘差計算、Schur、回代** | 2（加總順序） | 每個區段 1–10 ms | `ba_solver` 時間約 ÷2（4 執行緒） | 1-E；先量 BA 是不是瓶頸 |
| P2 | `create_new_map_points`：各鄰居的配對搜尋同時做 | 0 | 每個鄰居 0.5–3 ms | local mapping 每個 keyframe 少幾 ms | 第二階段照原順序寫入 |
| P3 | `fuse`：投影與候選搜尋平行，寫入照原順序 | 0 | 每個目標 keyframe 0.3–1 ms | `search_in_neighbors` 約 ÷2 | 同 `searchLocalPoints` 的兩段式 |
| P4 | `search_in_neighbors` 結尾：描述子與法向的計算平行，寫入依序 | 0 | 數百個點，約 0.5 ms | 小 | 和 M3（沒變就不寫）一起做 |
| P5 | BoW `transform`：每個描述子走訪詞彙樹平行 | 0 | 約 1000 個描述子 × 60 次 Hamming，約 0.5 ms | relocalize、新 keyframe 少約 0.3 ms | 只在 relocalize、`track_reference_keyframe`、keyframe 用 |
| P6 | matcher：每個左眼 keypoint 的候選搜尋平行 | 0 | 0.6 ms（這台）、3.5 ms（板子） | 板子上 matcher 約 ÷2 | matcher 目前不是瓶頸，排最後 |
| ✗ | tracking 的位姿最佳化（兩個） | — | 10–30 µs | 量過，變慢 | 不做 |
| ✗ | keyframe culling | — | — | — | 剔除一個 keyframe 會改變下一個的判斷，要推測執行再重算，太複雜 |
| ✗ | 三角化的 4×4 SVD、IMU 預積分 | — | µs 級 | — | 太小 |

**建議順序：1-E → P3 → P2 → P1 → 其餘**。P2、P3 逐位元相同，而且是 local mapping 每個 keyframe 都會做的事。P1 的單次效益最大，但會改變加總順序，而且依下面的量測，BA 目前不是瓶頸。

**整體來說：在 6 核的板子上，再加平行化的效益有限，省 CPU 的數學化簡（另一份計畫）更值得先做。** 理由見下一節。

## 現在的時間花在哪裡（量測）

這台 container（8 核，extractor 走 CPU backend），V1_02，real-time，`fw_stats` 的 handler 時間，改動後的版本：

| block | 每次 | 頻率 | 每秒的 CPU 時間 |
|---|---|---|---|
| extractor（這裡是 CPU backend；板子上主要在 GPU） | 13.2 ms（兩眼） | 20 Hz | 265 ms |
| tracking | 4.05 ms | 20 Hz | 81 ms |
| `ba_solver`（local BA） | 20.4 ms | 2.5 Hz | 51 ms |
| local mapping（每個 keyframe 的處理 + 套用 BA 結果） | 11.1 + 約 4 ms | 2.5 Hz | 38 ms |
| matcher | 0.76 ms | 20 Hz | 15 ms |
| loop closing | 0.9 ms | 2.5 Hz | 2 ms |

room1 的比例相近（BA 25.7 ms、local mapping 13.2 ms、tracking 3.7 ms）。

- keyframe 大約每 400 ms 來一個，local mapping 加上 BA 一個 keyframe 約 35 ms。就算板子慢 3 倍（約 105 ms），也遠遠跟得上。所以 P1 只會縮短「地圖更新的延遲」，不會提高頻率，也不會減少丟幀。
- tracking 是唯一有期限的（每幀 50 ms）。它在板子上 30–44 ms（HANDOFF），主要的工作已經平行化，或者已經量過不適合平行。
- 6 核上已經有 10 條執行緒。新的平行化會和 tracking 搶核心，可能讓關鍵路徑反而變慢。

**板子上要先補的量測**：各 block 的 handler 時間（`python3 tools/fw_stats.py`，見下面的注意事項）、`ba::Result::solve_ms`，以及 tracking 丟幀數。如果板子上 local mapping 會累積 keyframe（`abort_ba` 次數多），P1 的優先序才往前移。

**`tools/fw_stats.py` 的注意事項**：box 結束時，最後一則 stats 偶爾不含任何 block，summary 就只印出標題。這次 6 次 real-time 裡遇到 1 次。修法是只在 `stats["handlers"]` 不是空的時候才更新 `last`（一行）。這次是改在 scratchpad 的複本，倉庫裡的工具沒有動。

---

## P1 local BA：nls 的三個迴圈

[libs/nls/problem.cpp](../../libs/nls/problem.cpp)。local BA 有數十個 keyframe、數千個點、上萬條殘差，一次迭代就是數 ms 到數十 ms，是這個專案裡區段最大的地方。

| 迴圈 | 現在 | 平行化的方法 | 結果 |
|---|---|---|---|
| `computeActiveErrors` | 依序算每個殘差的誤差 | 每個殘差只寫自己的 `ebuf`，直接切塊平行 | 逐位元相同 |
| `activeRobustCost` | 依序加總 | 固定切 8 塊，依塊的順序加總（同 `shared/parallel.hpp`） | 加總順序不同（級 2），但和執行緒數無關 |
| `buildSystem` | 每個殘差算 Jacobian，再累加到 H 的區塊 | 依「被消去的點」分組：同一個點的殘差，寫的是同一組 (相機, 點)、(點, 點) 區塊，不同點之間不衝突；只有 (相機, 相機) 區塊會衝突，每塊各自一份，最後依序加總（local BA 裡相機數 < 30，一份約 30² × 36 個 double，約 260 KB） | 級 2 |
| Schur complement（`solveLinear` 的每個點） | 依序 | 每個點的 `Hinv`、`Q = H_ce·Hinv` 互不相關；對 S 的累加同上，每塊一份再依序加 | 級 2 |
| 回代（每個點的 dx） | 依序 | 每個點只寫自己的 dx，直接平行 | 逐位元相同 |
| 稀疏 LDLT（相機的部分） | Eigen `SimplicialLDLT` | 不動（相機數 × 6 維 < 200，很小） | — |

- 加 `nls::Options::parallel`（一個 `std::function`，和 `shared/parallel.hpp` 同樣的形式），`nls` 本身仍然不依賴 fw；`ba_solver` block 宣告 `THREADS(n)`，把 `fw::parallel_for` 傳進去。
- 只有殘差多（例如超過 2000 條）的時候才平行，小問題（pose graph、`optimize_relative_pose`）維持依序。
- 記憶體：每塊一份 (相機, 相機) 的 H 和 S，8 塊約 2 MB。
- **先量**：`ba::Result::solve_ms` 已經有了。要看 BA 在板子上多久、local mapping 是不是因為等 BA 而累積 keyframe（`abort_ba` 的次數）。如果 local mapping 跟得上，這項的效益只在「地圖更新快一點」，不影響 tracking 的頻率。

## P2 `create_new_map_points`：各鄰居同時搜尋

[triangulation.cpp](../../algorithms/orb_local_mapping/create_new_map_points/triangulation.cpp) `createNewMapPoints` 對最多 10 個鄰居依序做：`searchForTriangulation`（BoW 配對 + 三角化檢查），然後建立新點（寫入 atlas）。

- 前一個鄰居建立的點，會設定現在的 keyframe 的 `points[idx1]`，後面的鄰居就不會再用這個 `idx1`。
- 但 `searchForTriangulation` 裡，每個 `idx1` 挑哪個 `idx2`，只看 `idx1` 自己（`matched2` 從來沒被設定，見程式註解），和其他 `idx1` 無關。
- 所以可以：
  1. **第一段（平行）**：所有鄰居同時做 `searchForTriangulation`，以及 `createNewMapPoints` 裡寫入之前的所有檢查（視差、DLT、深度、重投影、尺度），用的是開始時的 atlas。
  2. **第二段（依序，照鄰居順序）**：逐個配對再檢查一次 `cur.points[idx1]` 和 `kf2.points[idx2]` 是不是還空著，空著才建立。
- 和依序的結果逐位元相同。另外兩個細節也要照原樣處理：`Ow1` 從上一個配對帶到下一個鄰居的基線檢查（ORB-SLAM3 的行為，見程式註解），以及「queue 裡有新 keyframe 就提早結束」。這兩個都只影響第二段，第一段多算的結果丟掉就好。
- 可以和 M2（反投影的快取）一起做：快取在第一段開始前算好，之後只讀。

## P3 `fuse`：兩段式

[fuse.cpp](../../algorithms/orb_local_mapping/search_in_neighbors/fuse.cpp)。`searchInNeighbors` 對 10–20 個目標 keyframe 各呼叫兩次 `fuse`，每次數百個點。

- **第一段（平行，每個點各自）**：投影、範圍檢查、`inArea`、在候選裡找 Hamming 距離最小的特徵。只讀點的位置、描述子、keyframe 的位姿與特徵，這些在 `fuse` 裡都不會被改。
- **第二段（依序）**：照點的順序，再檢查點是不是 bad、是不是已在這個 keyframe 裡，然後 `replace` 或 `addObservation`。
- 和 `changes.patch` 的 `searchByProjection` 同樣的做法，結果逐位元相同。
- 另一個做法是「不同目標 keyframe 同時做」，但目標之間會因為 `replace` 互相影響，不建議。

## P4 `search_in_neighbors` 的結尾

對現在的 keyframe 的每個點做 `computeDistinctiveDescriptors` 與 `updateNormalAndDepth`。

- 計算部分（距離矩陣、中位數、法向）只讀 atlas，每個點可以平行算出新值。
- 寫入（`editPoint`）依序做，而且依 M3，值沒變就不寫。
- 逐位元相同。

## P5 BoW `transform`

[vocabulary.cpp](../../types/map/vocabulary.cpp)：每個描述子從根往下走 6 層，每層 10 個 Hamming。

- 走訪（`descend`）每個描述子各自獨立，可以平行，把每個描述子的 (word, weight, node) 存到陣列。
- 再依描述子的順序寫入 `bow`（`std::map`）和 `feat`，順序與加總都和原本相同，逐位元相同。
- 只有 relocalize、`track_reference_keyframe`、建立 keyframe 時才用，所以效益小。

## P6 matcher

[stereo_matcher.cpp](../../algorithms/orb_matcher/stereo_matcher.cpp) 的第 1 步：每個左眼 keypoint 在 epipolar index 裡找最好、次好的右眼 keypoint，各自獨立。第 2 步（右眼被多個左眼搶時的仲裁）依序。

- 逐位元相同。
- 板子上每組 3.5 ms，在前端的時間預算裡算小，所以排最後。

---

## 驗證計畫

- 每一項各自一個 commit。逐位元相同的項目（P2–P6），lossless 跑一次 ATE 必須完全相同。P1 照級 2 的做法（6 個序列 lossless，加上 real-time 多次）。
- 效能一定要在**板子上**量：這台 container 是 8 核，NXP 是 6 核，執行緒的競爭不同。要看的是：
  - tracking 每幀時間與丟幀數（關鍵路徑，不能變差）；
  - local mapping 每個 keyframe 的時間；`abort_ba` 的次數；
  - 總 CPU 時間（user + sys），平行化通常會讓它稍微增加。
