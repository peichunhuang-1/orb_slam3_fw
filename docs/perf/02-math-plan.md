# 數學上等價的化簡：計畫（2026-11-25）

只是計畫，程式還沒改。範圍是整個專案的 block 與它們用到的 `types/`、`libs/`。GPU（`vk/`、shader）不在範圍內。

## 影響度分級（與優化計畫相同，低 → 高）

| 級 | 意義 | 怎麼驗證 |
|---|---|---|
| **0** | 逐位元相同：同樣的輸入做同樣的浮點運算，只是不重算、不重複複製 | 單元測試逐位元比對；lossless 跑一次，ATE 與軌跡必須完全一樣 |
| **2** | 只有捨入不同（float → double、運算順序） | 單元測試比 ORB-SLAM3 參考（inlier 數相同、位姿 1e-6 量級）；lossless 跑全部序列，加上 real-time 多跑幾次看分散 |
| **3** | 演算法改變 | 不列入，只在最後列出不做的原因 |

所有項目都遵守設計原則：邏輯和數學與 ORB-SLAM3 一致，不引入第三方函式庫，程式放在涵蓋所有使用者的最小資料夾。

## 先講結論

| # | 項目 | 級 | 預期效果 | 工作量 |
|---|---|---|---|---|
| M1 | nls：每次迭代開頭不重算誤差 | 0 | 所有 BA 的殘差計算少 1/3 左右 | 小 |
| M2 | 三角化：每個 keyframe 的反投影只算一次 | 0 | `create_new_map_points` 估計快 30–60% | 小 |
| M3 | atlas：值沒變就不 `edit`（避免 COW 複製） | 0 | local mapping 時間與記憶體尖峰都下降，要先量測 | 中 |
| M4 | `computeDistinctiveDescriptors`：中位數用 `nth_element` | 0 | 這個函式 O(n² log n) → O(n²) | 小 |
| M5 | `updateConnections`：`std::map` 計數改排序過的 vector | 0 | 小 | 小 |
| M6 | `fuse`：th=3 與 th=1 兩次呼叫共用投影結果 | 0 | `search_in_neighbors` 的投影少一半 | 小 |
| M7 | inertial 殘差：誤差與 Jacobian 共用子運算式 | 0 | inertial BA 的殘差計算約快 10–20% | 小 |
| M8 | IMU 預積分的共變異數：利用 A、B 的固定零元素 | 0（有限值時） | 每個 IMU 樣本約快 2×，但總量本來就很小 | 小 |
| M9 | **BA 的重投影殘差改用 `project_jacobian_d`** | 2 | local BA 殘差計算估計快約 2×（量過同樣的改法：位姿最佳化 1.8 → 0.9 ms） | 小 |
| M10 | 其餘 float 投影（fuse、搜尋、loop closing）改用不呼叫 `atan2` 的投影 | 2（門檻邊緣可能翻一兩個配對） | 每個點省 2 次 `atan2f` 和 1 組 `sin/cos` | 中 |

**建議順序：M1 → M2 → M9 → M3 → 其餘**。M1、M2、M9 改動小、效益大。M1 和 M2 逐位元相同，只要做逐位元驗證。

---

## 級 0：逐位元相同

### M1 nls：每次迭代開頭的誤差重算是多餘的

[libs/nls/problem.cpp](../../libs/nls/problem.cpp) `Problem::step()`：

```
computeActiveErrors();          // (1) 所有殘差的誤差
current = activeRobustCost();
buildSystem();                  // (2) 所有殘差的誤差 + Jacobian（又算一次誤差）
do { solveLinear; update; computeActiveErrors();  // (3) 試探步的誤差
     ...接受就 current = trial，否則 pop() 再試 } while (rho < 0 && tries < max_trials);
```

- 一般情況下，只有接受一步，或試到上限（此時 `done = true`），`do` 迴圈才會結束。所以下一次迭代一開始的參數，就是上一步被接受的試探點，`ebuf` 裡的誤差和 `current` 正是 (1) 要算的東西。第一次迭代時，`start()` 也已經算過一次。
- **例外**：試探點的 cost 是 NaN 時，`rho` 也是 NaN，`rho < 0` 不成立，迴圈結束了，但這一步沒有被接受（參數已經 `pop()` 回去）。這時 `ebuf` 是試探點的誤差，不是現在參數的。
- 做法：在 `Run` 加一個旗標「`ebuf` 對應目前的參數」。`start()` 和「接受一步」時設為 true，`pop()` 時設為 false。`step()` 開頭只有旗標為 false 時才做 (1)，否則直接用 `R.s.final_cost`。`buildSystem` 本來就會重算誤差（它需要 Jacobian），值也一樣。
- 逐位元相同：同樣的參數、同樣的函式、同樣的加總順序；例外情況照舊重算。
- 影響範圍：local BA、inertial BA、IMU 初始化、global BA、pose graph、merge、`optimize_relative_pose`，都用 `nls::Problem`。
- 效果：每次迭代的殘差計算從「(1) + (2) + 每次試探 (3)」少掉 (1)。通常一次迭代只試 1 次，所以大約少 1/3 的純誤差計算（(2) 含 Jacobian，比較重）。
- 驗證：`nls_test`、`orb_local_mapping_*`、`orb_loop_closing_*` 的 fixture 逐位元不變；lossless ATE 完全相同。

### M2 三角化：反投影的 Newton 迴圈在最內層重複做

[triangulation.cpp](../../algorithms/orb_local_mapping/create_new_map_points/triangulation.cpp)：

- `searchForTriangulation` 對每個 `idx1`，在每個候選 `idx2` 上呼叫 `triangulateMatches`，裡面每次都做 `cam1.unproject_orb(kp1)` 和 `cam2.unproject_orb(kp2)`（float 的 Newton，最多 10 步）。同一個 `kp1` 會因為每個候選重算一次，同一個 `kp2` 也會因為每個 `kp1` 重算。
- `createNewMapPoints` 對每組配對又算一次 `unproject_orb(kp1)`、`unproject_orb(kp2)`。
- 一個 keyframe 對 10 個鄰居，現在的 keyframe 的反投影會被算 10 次以上。
- 做法：每次 `createNewMapPoints` 開頭，算出現在的 keyframe 每個特徵的 bearing（有需要時才算，`std::vector<Eigen::Vector3f>` 加一個「算過了」的旗標）；每個鄰居也一樣。`triangulateMatches` 改成接 bearing。放在 `create_new_map_points/` 裡（只有它用）。
- 逐位元相同：同一個函式、同樣的輸入。
- 記憶體：每個 keyframe 12 B × 特徵數（1000 個約 12 KB），用完就丟，不留在 keyframe 上。

### M3 atlas：值沒變就不要 `edit`

atlas 是 `policy: versioned`，每個點、每個 keyframe 都是一個 tile。一個版本裡第一次 `editPoint` / `editKeyFrame`，就會**複製整個 tile**（包括 observations 這類 FlatMap）。

| 位置 | 現在 | 改成 |
|---|---|---|
| `computeDistinctiveDescriptors`（[atlas_ops.cpp](../../types/map/atlas_ops.cpp)） | 不管有沒有變都 `editPoint` 寫描述子 | 和現有的描述子相同就不寫 |
| `updateNormalAndDepth` | 每次都 `editPoint` | `max_distance`、`min_distance`、`normal` 都沒變就不寫 |
| `updateConnections` → `addConnection` | 對每個共視 keyframe 都寫權重 | 權重沒變就不寫（不必複製那個 keyframe） |

- `search_in_neighbors` 結尾，對現在的 keyframe 的每個點（數百個）都呼叫前兩個函式；很多點的描述子和法向其實沒變。
- 逐位元相同：寫進去的值本來就和原值一樣。
- 效果：local mapping 時間，以及 atlas 版本裡被複製的 tile 數（記憶體尖峰、tracking 釋放舊版本的時間，也就是 HANDOFF 裡 `sync_atlas` p95 18 ms 的來源之一）。**要先量**：加一個計數器，看一個 keyframe 會觸發幾次「值沒變的 edit」。

### M4 `computeDistinctiveDescriptors`：中位數不必整列排序

- 現在對距離矩陣的每一列做 `std::sort`，只是要拿中位數：O(n² log n)。
- 改成 `std::nth_element` 取同一個位置：O(n²)。中位數的值相同；挑選規則是「嚴格小於才換」，所以選中的 index 也相同。
- n 是一個點的觀測數 × 2（左右眼），長時間觀測的點會到幾十個。

### M5 `updateConnections`：計數用排序過的 vector

- 現在 `std::map<KeyFrameId, int> counter`，每個點的每個觀測都 `++counter[id]`。
- 改成收集 id 到 vector，排序後數相鄰相同的數量。走訪順序（id 由小到大）和計數都和 `std::map` 相同，所以 `connected`、`ordered_connected` 與「平手時誰先」都不變。`changes.patch` 對 `updateLocalKeyFrames` 做的就是同樣的事。

### M6 `fuse`：兩次呼叫共用投影

- [fuse.cpp](../../algorithms/orb_local_mapping/search_in_neighbors/fuse.cpp) `searchInNeighbors` 對每個目標呼叫 `fuse(th=3)` 再 `fuse(th=1)`，兩次都把同一組點用同一個 keyframe 位姿投影一次（`Tcw * Pw`、`project_orb`、距離、法向角度、`predictScale`）。
- 第一次呼叫可能讓某些點變成 bad，或在 keyframe 上加觀測，但**投影本身只依賴位姿和點的位置**，在這兩次呼叫之間都不變。
- 做法：第一次呼叫時記下每個點的投影結果，第二次直接用；bad、`inKeyFrame` 等檢查照舊每次重做。
- 逐位元相同。

### M7 inertial 殘差：誤差和 Jacobian 共用子運算式

[inertial_residuals.cpp](../../algorithms/ba_solver/inertial_residuals.cpp) `InertialResidual::evaluate`：

- `Rbw1 * (v2 - v1 - g·dt)` 和 `Rbw1 * (P2.t - P1.t - v1·dt - g·dt²/2)`，在誤差和 Jacobian 裡各算一次。誤差寫 `g*dt*dt/2`，Jacobian 寫 `0.5*g*dt*dt`；乘或除以 2 在浮點是精確的，所以兩者逐位元相同，可以共用。
- `deltaRotation(b1)`、`deltaVelocity(b1)`、`deltaPosition(b1)` 各自算一次 bias 修正；改成一次呼叫回傳三者（在 `types/imu/` 加一個函式），內部運算順序不變。
- `P2.R.transpose() * P1.R` 等矩陣乘積同樣可以共用。

### M8 IMU 預積分：共變異數傳播利用固定的零

[preintegrated.cpp](../../types/imu/preintegrated.cpp) `integrate`：`C = A·C·Aᵀ + B·Σ·Bᵀ` 用的是稠密的 9×9。A 有一半是固定的 0 或單位矩陣，B 是 9×6 而且一半是 0。

- 按區塊寫出，只算非零的部分。加 0、乘 1 在有限值時是精確的，所以逐位元相同（如果出現 NaN 或 Inf，原本就已經是錯的）。
- 效果：每個 IMU 樣本省約一半的乘加；但每幀只有約 10 個樣本，所以總量很小。**排在最後**。

---

## 級 2：只有捨入不同

### M9 BA 的重投影殘差改用 `project_jacobian_d`

[reprojection.hpp](../../algorithms/ba_solver/reprojection.hpp) `Kb8Reprojection::evaluate`，是 local BA、global BA、merge 裡數量最多的殘差：

- 現在：`project_orb(Xr)`（float 的 θ、ψ，兩次 `atan2f` + `cos` + `sin`），再加 `project_jacobian_orb(Xr)`（同樣的 θ 再算一次），還有 `q * Xw`（四元數旋轉）加上 `q.toRotationMatrix()`（給點的 Jacobian）。
- 改成：先 `R = q.toRotationMatrix()`，`Xl = R·Xw + t`；再 `project_jacobian_d(Xr, J)`，一次得到像素和 Jacobian。
- 這就是 2-A 對 tracking 位姿最佳化做的改法。量過的效果是 1.80 → 0.89 ms/次（2.0×），單元測試的 inlier 數與 ORB-SLAM3 相同。
- 也適用於 [inertial_residuals.cpp](../../algorithms/ba_solver/inertial_residuals.cpp) 裡 inertial BA 的視覺邊（2 處），以及 [optimize_relative_pose.cpp](../../algorithms/orb_loop_closing/optimize_relative_pose/optimize_relative_pose.cpp)（3 處，loop closing 偶爾用）。
- 驗證：`orb_local_mapping_local_ba_test` 等 fixture（位姿差在 1e-6 量級），以及 lossless 全部序列加 real-time 多次。和 2-A 一樣，「沒差很多就過」。

### M10 其餘 float 投影：不呼叫 `atan2`

`fuse`、`search_by_projection`（tracking 與 relocalize）、`local_map` 的 `searchByProjection`、`keyframe_matching`、`correction` 都只要像素、不要 Jacobian，用的是 `project_orb`（兩次 `atan2f`，再 `cos(ψ)`、`sin(ψ)`）。

- 改成 `cos ψ = x/ρ`、`sin ψ = y/ρ`，省掉 `atan2f(y, x)`、`cos`、`sin`。θ 仍用 `atan2f(ρ, z)`。
- 這些結果是拿去和門檻比（`inImage`、搜尋半徑、5.99 的 χ²），捨入差異可能讓剛好在門檻上的配對翻過來。所以屬於「級 2，但偶爾會多或少一兩個配對」。
- 效果：每次投影省約 40% 的時間。tracking 每幀有數千次（`searchLocalPoints`），local mapping 每個 keyframe 有數萬次（`fuse`）。

---

## 不做（級 3）

| 想法 | 為什麼不做 |
|---|---|
| 三角化的 4×4 `JacobiSVD` 換成封閉解（midpoint 等） | 結果不同，和 ORB-SLAM3 的 DLT 不一致 |
| 預積分不再每步 `normalizeRotation`（SVD） | ORB-SLAM3 每步都正交化；省掉會讓長時間的 dR 漂移 |
| 減少 LM 迭代次數、放寬停止條件 | 改變最佳化的結果 |
| 減少 `fuse` 的第二次呼叫（th=1） | 是 ORB-SLAM3 的行為（對應它的右眼 Fuse） |

## 驗證計畫

1. **每一項各自一個 commit**。級 0 的項目要確認 lossless 跑一次的 ATE 與軌跡完全相同（V1_02、room1 兩個就夠，因為是逐位元相同）。
2. 級 2 的項目照 2-A 的做法：lossless 跑 6 個序列，real-time 在 V1_02、room1 各跑 3 次，和改之前比 ATE 的分布。
3. 每一項都要量改之前、改之後的時間。local mapping、BA 沒有逐節點計時，需要先加暫時的計時（和 HANDOFF 的 `ORB_NODE_TIMER` 一樣，不進 commit）。

---

## 實作結果（2026-11-25）

M1–M6 已合併進 master（fast-forward，2026-11-25）。**決定：M9、M10 不做**；M9 的 commit 留在 `perf/m9-ba-double` 分支，沒有合併。

| # | commit | 結果 |
|---|---|---|
| M1 | `2f052d9` | 逐位元相同 |
| M2 | `73af65e` | 逐位元相同 |
| M3、M4、M5 | `2ced06c`（同一個檔案 `atlas_ops.cpp`） | 逐位元相同。M3 的 `addConnection` 本來就有跳過，只補了描述子和法向／距離 |
| M6 | `addd013` | 逐位元相同。投影快取是惰性的：第二次呼叫遇到第一次沒算到的點，照樣當場算 |
| M7、M8 | 沒做 | 這個 container 和板子都是 aarch64，GCC 預設把乘加縮成 FMA，所以重排運算式不保證逐位元相同。inertial 邊每次 BA 只有約 10–20 個（重投影邊有數千個），省下的時間可以忽略 |
| M9 | `7f8b4be`（在 `perf/m9-ba-double`） | 級 2，**不做**（BA 只快約 10%，ATE 平均稍差），見下面 |
| M10 | 沒做 | 級 2，**不做**（會翻轉門檻邊緣的配對，理由同 M9） |

逐位元驗證：V1_02 與 room1 lossless，錄下來的 `tracking/pose` 與 `map_to_odom` 訊息的 sha256 和改之前完全相同；單元測試 25/26 通過（GPU 那項在這裡本來就失敗）。

### 時間（real-time，fw_stats 每次呼叫平均，opt 3 次、其餘各 2 次）

| | tracking | local mapping（atlas_edits） | BA（ba_solver on_problem） |
|---|---|---|---|
| V1_02：opt → M1–M6 → +M9 | 4.06 → 4.18 → 4.11 ms | 11.23 → 10.89 → 10.64 ms | 20.44 → **17.88** → **16.26** ms |
| room1：opt → M1–M6 → +M9 | 3.76 → 3.76 → 3.75 ms | 13.34 → 12.61 → 12.81 ms | 25.86 → **22.63** → **19.67** ms |

- M1–M6：BA 約 −12%（主要是 M1），local mapping 約 −3 到 −5%。tracking 不受影響（差異在 real-time 的波動內）。
- M9：BA 再 −9 到 −13%。比計畫估的 2 倍小很多：殘差計算只佔 BA 的一部分，Schur 消去和線性求解佔大頭。
- lossless 的總 CPU 時間幾乎沒變（M1–M6 約 −1%，M9 −0.1%），因為 local mapping 和 BA 本來就只佔整體 CPU 的一小部分，大頭是 extractor 和 tracking。

### M9 的 ATE（17 序列 lossless，對 M1–M6）

- 全部：odom 幾何平均 1.065、map 0.950。
- 排除 room4：odom 1.029（9/16 變差）、map 1.049（7/16 變差，分散 ×/÷1.38）。room4 這次反過來：M1–M6 有觸發 loop（map 0.068 m），M9 沒有（0.013 m）。
- V2_03 的 map 從 0.094 變成 0.256 m（2.7 倍），兩邊都有偵測到 loop，看起來又是回環修正的問題。
- 和 2-A（0.96／0.97）比，M9 平均稍差，但仍在捨入擾動的分散範圍內。

**建議**：M9 換到的只是 BA 約 10%，而 BA 本來就不是瓶頸（每個 keyframe 約 20 ms，keyframe 間隔約 400 ms），ATE 平均還稍微變差。我傾向**先不合併 M9**，等板子上的量測確認 BA 有沒有影響 tracking，再決定。M10 也同理。
