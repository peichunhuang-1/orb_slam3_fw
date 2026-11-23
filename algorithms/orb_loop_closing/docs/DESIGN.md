# orb_loop_closing 設計：ORB-SLAM3 的 loop closing 與地圖合併

這份文件定架構（圖上怎麼連、誰寫 `atlas`、local mapping 的樹怎麼加、資料放在哪裡），並分階段記錄實作與驗證。沿用 [local mapping 的設計](../../orb_local_mapping/docs/DESIGN.md)與 [tracking 的設計](../../orb_tracking/docs/DESIGN.md#原則)的原則（以 id 引用、型別只放資料、程式碼依使用者放置、長時間運算在鎖外），不再重複。

**目前的狀態**：階段 1 完成（只偵測，不修正）。

| 階段 | 內容 | 狀態 |
|---|---|---|
| 0 | 骨架：`loop_closing` block、`loop_result`（`wake`）、`send_to_loop_closing` 把 keyframe 加進 BoW 索引；`wake` 與 job 結束的試驗；`frame_gate` 只丟影像的試驗 | 完成（見下） |
| 1 | place recognition：候選、keyframe 間的匹配、`RelativePoseRansac`、`optimizeRelativePose`、三次確認（loop 與 merge 都偵測，只偵測不修正） | 完成（見下） |
| 2 | `CorrectLoop`：Stop／Release、`SearchAndFuse`、essential graph（6DoF、4DoF） | 未開始 |
| 3 | global BA：`gba_solver`、分段迭代、寫回時沿 spanning tree 傳遞 | 未開始 |
| 4 | 地圖合併：`MergeLocal`（stereo）、`MergeLocal2`（stereo-inertial）、`MergeInertialBA`；`slam_merge_job` | 未開始 |

**階段 0 的實跑**（Pi 5，V1_02_medium，lockstep；以暫時的追蹤輸出觀察，之後已移除）：

- `slam_job`：job 自己結束（2 分 12 秒，`every block has ended (EOS)`），`wake` 的 `loop_result` 不妨礙結束。350 個 keyframe 都收到 `loop_result` 並套用，沒有 epoch 對不上而丟掉的，也沒有因 keyframe 已是 bad 而跳過的。結束時沒有 keyframe 留在 `not_erase`。BoW 索引從 keyframe 25 的 18,773 筆長到 350 的 67,378 筆，bad 的 keyframe 由 `setKeyFrameBad` 移出（原版 `SetBadFlag` 的 `mpKeyFrameDB->erase`），所以 relocalization 從此查得到候選。這次 tracking 沒有跟丟，relocalization 沒有實際觸發。
- `slam_merge_job`（`frame_gate` 丟掉 30–32 s 的影像，左右各 40 張）：`imu_preintegration` 報 0 gaps、0 frames dropped，沒有 `stop_box`。缺口之後 atlas 從 1 張地圖變成 2 張（`current` 0 → 1），與預期的路徑相同（`frame_discontinuity` → `handle_discontinuity` → `createMapInAtlas`）。job 同樣自己結束。
- 還沒測到的：reset（`epoch` 的遞增與丟棄）在這兩次都沒有發生。

**階段 1**：

- 程式：`keyframe_candidates/`（`DetectNBestCandidates`）、`keyframe_matching/`（`searchByBoW(kf, kf)`、`searchByProjection` 兩個版本）、`relative_pose_ransac/`、`optimize_relative_pose/`、`place_recognition/`（`NewDetectCommonRegions` 與它呼叫的函式）。`loop_closing` 再做 `Run()` 在修正之前的部分：merge 時 IMU 未初始化就放棄、算出 `Tmw` 與 `T_old_new`；inertial 的 loop 檢查 roll、pitch、yaw 並在 BA2 後只留 yaw；最後重設該組狀態。結果放在 `loop_result`（`kind`、`matched_kf`、`Tcw`…），local mapping 目前只計數。
- 照原版保留的怪處：
  - `searchByProjection` 帶點來源 keyframe 的那個版本，用 KB8 的內參以針孔公式投影（沒有畸變）。
  - `refinePoseFromLastKeyFrame` 確認的位姿是傳進來的那個；最佳化的結果只用來決定過不過。
  - `FindMatchesByProjection` 把共視的 keyframe 全部接上，不只是新加的。
  - `DetectCommonRegionsFromBoW` 對目前 keyframe 的共視逐一檢查時，會用那個 keyframe 附近的點蓋掉 `points`。
  - `optimizeRelativePose` 用 g2o 的數值微分（δ = 1e-9）。KB8 投影內部用 float，所以這個微分大多是捨入誤差，原版就是用這種有雜訊的 Jacobian 在最佳化。換成解析 Jacobian 會收斂得更好，fixture 上 inlier 多 20 %，偵測會比原版鬆，所以沒有換。
- 和原版不同的地方：
  - 不在 kf2 左影像裡的點，原版用 `mnTrackScaleLevel`（tracking 留下的暫存值）當它的層級；這裡改用它到 kf2 的距離預測層級。這個層級只影響那條邊的權重。
  - RANSAC 的亂數用 `std::mt19937`，原版用 `rand()`。
- fixture（`tests/fixtures/orb_slam3/loop/`，兩段經過同一處，第二段有漂移；`loop` 一張地圖、`merge` 兩張）：
  - 候選、`searchByBoW`、兩個 `searchByProjection` 與原版完全相同。
  - RANSAC 與 `optimizeRelativePose` 只能在容差內比：RANSAC 取第一個 inlier 夠多的假設，原版自己的結果就離真值 0.3–1.1°、2–8 cm；最佳化從同一起點出發，inlier 差 5–15 %，位姿差 0.05–0.3°。
  - `DetectCommonRegionsFromBoW` 重複 30 次：偵測到的次數，這裡 27 與 24，原版 25 與 26。
- V1_02 實跑（Pi 5）：每個 keyframe 偵測平均 2.8 ms，p95 8 ms，最大 19 ms，佇列深度 16 足夠。沒有偵測到 loop。原版在 V1_02 stereo-inertial 跑 5 次是 0–1 次 loop（`reference_orb_source/vslam/tools/compare_runs.py` 的記錄）。大部分候選停在共視檢查或 RANSAC（BoW 配對 20–30 個，多半是錯的）。
- `slam_merge_job` 實跑：第二張地圖做完 BA2 之後，回到舊區域的 keyframe 都找得到舊地圖的 merge 候選（254 個），BoW 配對約 60–130 個。但都沒有通過：最接近的幾個停在「最佳化後的投影」（47–75，門檻 80），所以沒有偵測到 merge。原版在這條資料上會不會 merge 還沒比過（要讓 reference 吃同一份特徵）。階段 4 之前要決定：比對原版，或把缺口移到回訪舊區域比較多的位置。
- 因為還不修正，偵測到的 loop 之後可能再被偵測到；原版修正後地圖已經對齊，不會再偵測。這要到階段 2 才會一致。

**驗收**：V1_02 上整條軌跡的 ATE 與原版相近、loop 的次數相近。中間狀態不要求一致（原版的三條執行緒時序本身不固定）。函式層以 `fixture_gen` 的答案比對，是為了除錯，不是驗收條件。

## 範圍

| | 做 | 不做 |
|---|---|---|
| 感測器 | stereo、stereo-inertial（KB8） | 單目 |
| 偵測 | `NewDetectCommonRegions` 的全部：loop 與 merge 的候選、`DetectCommonRegionsFromBoW`、`DetectAndReffineSim3FromLastKF`、三次確認、找不到兩次就放棄 | 舊的 `DetectLoop`／`ComputeSim3`（原版已不用） |
| 修正 | `CorrectLoop`、`MergeLocal`、`MergeLocal2`、global BA（視覺 `GlobalBundleAdjustemnt`、inertial `FullInertialBA`） | |
| 開關 | 原版的 `mbActiveLC`（`config.loop_closing`） | |

**以 SE3 取代 Sim3**：stereo 與 stereo-inertial 時原版 `mbFixScale = true`，所有 Sim3 的尺度都固定為 1，所以全部改用 `geometry::SE3f`／`SE3d`，結果與原版相同：

| 原版 | 這裡（名稱） | 內容 |
|---|---|---|
| `Sim3Solver(…, bFixScale = true)` | `RelativePoseRansac`（`relative_pose_ransac/`） | Horn 的閉式解，s = 1，RANSAC 照原版 |
| `Optimizer::OptimizeSim3(…, bFixScale = true)` | `optimizeRelativePose`（`optimize_relative_pose/`） | 一個 SE3 頂點（nls 的 `SE3Left`），兩邊的重投影誤差 |
| `ORBmatcher::SearchBySim3` | 不需要 | 只有舊的 `ComputeSim3` 用，`NewDetectCommonRegions` 不用 |
| `ORBmatcher::SearchByProjection(pKF, Scw, …)`（兩個 overload） | `searchByProjection(kf, Tcw, …)` | |
| `ORBmatcher::Fuse(pKF, Scw, …)` | `fuseWithPose` | |
| `DetectAndReffineSim3FromLastKF` | `refinePoseFromLastKeyFrame` | |
| `OptimizeEssentialGraph(…, bFixScale = true)` | `ba::Kind::pose_graph` | 6DoF 的 SE3 位姿圖 |
| `OptimizeEssentialGraph4DoF` | `ba::Kind::pose_graph_4dof` | 4DoF（x、y、z、yaw），與原版相同 |
| `g2o::Sim3` 的 `KeyFrameAndPose` | `KeyFramePoses` = `FlatMap<KeyFrameId, geometry::SE3d>` | |
| `mg2oLoopScw`、`mg2oLoopSlw`、`mg2oMergeSmw`… | `Tcw`、`Tlw`、`Tmw`… | |
| merge 時 scale 在 0.9–1.1 的檢查 | 拿掉 | 固定尺度時恆為 1，原版這個檢查不會成立 |

**命名**：這裡的型別、函式、檔案、資料夾、欄位名稱都不含 `sim3`，避免讓人以為有尺度。原版的名稱只出現在對照（「原版」那一欄、程式開頭的註解）。

代價是不能支援單目；要支援時再把 SE3 換回 Sim3（型別與一個 manifold）。

## 誰寫 `atlas`：只有 local mapping

`loop_closing` **只讀** `atlas`（versioned，不等任何人）。它對地圖的修改都送給 local mapping，由 local mapping 在自己的樹裡套用，與 tracking 送 `atlas_edits` 是同一個模式。

| 原版 | 這裡 |
|---|---|
| loop closing 執行緒拿 `mMutexMapUpdate` 直接改地圖 | `loop_result` 送到 local mapping，由它的葉節點改 |
| `RequestStop` → 等 `isStopped` → 修正 → `Release` | 有修正等著時 `stop_requested`：不再開始新的 keyframe 與 local BA，目前的 keyframe 處理完就進修正的分支；修正完 `Release` |
| `EmptyQueue`（LC 執行緒呼叫 LM 的 `ProcessNewKeyFrame`） | 修正分支的第一步，同一個樹 |
| GBA 執行緒結束時 `RequestStop`，寫回，`Release` | GBA 的結果到了先記著，local mapping 下一次閒置時寫回 |
| LM 執行中的 local BA 寫回可能蓋掉修正 | stop 時 local BA 不會開始；已送出的照常等它回來寫回（與原版一樣，`isStopped` 在 `Run()` 一輪結束才成立），之後才修正 |

這樣不需要跨 block 的 stop 握手，也只有一個寫入者，寫入鎖不會互等。

## 圖

```
 ┌────────────────────────────────────── box main ──────────────────────────────────────────┐
 │  …（與 slam_job 相同：driver → extractor × 2 → matcher → imu_preintegration → tracking）   │
 │                                                                                          │
 │   ┌──────────── local_mapping (behavior) ──────────────────────────────────────┐         │
 │   │ atlas_edits (trigger)  mapper_cmd (trigger)                                │         │
 │   │ ba_result (wake) ◄───────── ba_solver ◄── ba_problem                       │         │
 │   │ gba_result (wake) ◄──────── gba_solver ◄── gba_problem / gba_continue      │         │
 │   │ loop_result (wake) ◄───────────────────────────────┐                       │         │
 │   │ ──► loop_keyframe ────────────────┐                │                       │         │
 │   └───────────────────────────────────┼────────────────┼───────────────────────┘         │
 │                            trigger    ▼                │                                 │
 │                     ┌──────── loop_closing (algorithm class) ────────┐                   │
 │                     │ keyframe (trigger)  params, vocabulary (latest)│                   │
 │                     │ atlas (read)                                   │                   │
 │                     │ ──► loop_result                                │                   │
 │                     └────────────────────────────────────────────────┘                   │
 │                                                                                          │
 │   atlas (r: tracking, loop_closing；w: local_mapping)   local_mapper (w: local_mapping)    │
 └──────────────────────────────────────────────────────────────────────────────────────────┘
```

- **loop_closing** 是一般的 algorithm class，不是樹：`LoopClosing::Run()` 一輪只是「取一個 keyframe、偵測、送出結果」，沒有要等的東西。偵測的狀態（`mnLoopNumCoincidences`、`mpLoopMatchedKF`、`mg2oLoopSlw`…）是它的成員，沒有別人用，不必放資源。
- **gba_solver** 與 `ba_solver` 是同一個 algorithm 的兩個實例。GBA 要算幾秒，用同一個實例會卡住 local BA。essential graph 與 merge 的最佳化用 `ba_solver`：那時 local mapping 停著，`ba_solver` 是空的。
- **輸入的種類**：

| 埠 | 模式 | 理由 |
|---|---|---|
| `loop_closing.keyframe` | `trigger`，`QUEUE(16, drop_oldest)` | 原版的 `mlpLoopKeyFrameQueue`。佇列深度 16（階段 1 量過：偵測平均 2.8 ms） |
| `local_mapping.loop_result` | `wake` | 打斷 `local_mapping → loop_closing → local_mapping` 的環，與 `ba_result` 相同 |
| `local_mapping.gba_result` | `wake` | 同上 |

**環**：

| 環 | 為什麼成立 |
|---|---|
| `local_mapping.loop_keyframe → loop_closing → loop_result → local_mapping` | `loop_result` 是 `wake` |
| `local_mapping.gba_problem → gba_solver → gba_result` | `gba_result` 是 `wake` |

**`wake` 與 job 的結束（階段 0 要實測）**：`wake` 不計入資料結束的判定。bag 播完、`atlas_edits` 結束後，樹進入 draining，「丟棄 wake 輸入上還沒處理的資料」（fw 文件 `docs/concepts/behavior-tree.md` 的結束流程）。所以最後一兩個 keyframe 的 `loop_result` 與進行中的 GBA 可能被丟掉。即時的位姿已經由 tap 錄下，影響的只有結束時的地圖；若實測發現 job 不會結束或會卡住，再改設計。

## 訊息

**`loop_closing/LoopKeyFrame`**（取代 `map/KeyFrameRef` 作為 `loop_keyframe` 的型別）：

```
map, keyframe        mpCurrentKF
epoch                local_mapper.reset_epoch：每次套用 reset_all／reset_active_map 加一
```

`clearAtlas` 之後 keyframe 的 id 從 0 重新開始，loop closing 留著的 id 會指到別的 keyframe。`epoch` 改變時 loop closing 清掉自己的狀態（原版的 `ResetIfRequested`；`reset_active_map` 時只在它留著的 keyframe 屬於那張地圖時才清）。local mapping 丟掉 `epoch` 對不上的 `loop_result`。

**`loop_closing/LoopResult`**（每處理完一個 keyframe 送一筆）：

```
seq, epoch
keyframe                         這次處理的 keyframe
erase_flags                      SetNotErase／SetErase 依原版呼叫的順序排成一列（同一個 keyframe 可能兩種都有，
                                 分成兩個清單會弄錯先後）
kind: none | loop | merge
loop:  matched_kf、Tcw（mg2oLoopScw，目前 keyframe 修正後的位姿）、
       matched_points（每個特徵：mvpLoopMatchedMPs）、loop_points（mvpLoopMapPoints）
merge: matched_kf、Tcw（mg2oMergeScw）、Tmw（mg2oMergeSmw）、
       matched_points、merge_points、current 的 last keyframe（mpMergeLastCurrentKF）
```

**`ba/Problem` 的新種類**：

| kind | 原版 | 用在 |
|---|---|---|
| `pose_graph` | `OptimizeEssentialGraph(pMap, pLoopKF, pCurKF, …, bFixScale)` | `CorrectLoop`（非 inertial，或 IMU 未初始化） |
| `pose_graph_4dof` | `OptimizeEssentialGraph4DoF` | `CorrectLoop`（inertial 且 IMU 已初始化） |
| `pose_graph_merge` | `OptimizeEssentialGraph(pCurKF, vpFixedKFs, vpFixedCorrectedKFs, vpNonFixedKFs, vpNonCorrectedMPs)` | `MergeLocal` 的第二段 |
| `global` | `BundleAdjustment`（`GlobalBundleAdjustemnt(pMap, 10, …)`） | 視覺的 GBA |
| `full_inertial` | 已有；GBA 用 `FullInertialBA(pMap, 7, false, nLoopKF)` | inertial 的 GBA |
| `welding` | `LocalBundleAdjustment(pMainKF, vpAdjustKF, vpFixedKF, …)`（三個參數的 overload） | `MergeLocal` 的第一段 |
| `merge_inertial` | `MergeInertialBA` | `MergeLocal2` |
| `inertial_bias` | `InertialOptimization(pMap, bg, ba)`（只估 bias） | `MergeLocal2`，地圖還沒有 BA2 時 |

位姿圖的問題另需 keyframe 對之間的相對位姿（`Sji`）與邊的種類（spanning tree、loop、covisibility ≥ 100、新的 loop 連線、inertial 的 prev），放在 `ba::Problem` 新的 `edges` 欄位。

## global BA：分段迭代

fw 的同一個 block 的 handler 不會同時執行，也沒有中斷一次呼叫的機制（[api_guide.md](../../../api_guide.md) 第 4 節：「同一個 block 實例的所有 handler 絕對不會同時執行」），所以 stop 的輸入在 solver 算完之前收不到。改成每次呼叫只跑一次迭代，由 local mapping 決定要不要繼續：

```
local_mapping                                   gba_solver（問題留在成員裡）
  啟動 GBA：  ──gba_problem{seq, kind, …}──►      建 nls 問題，跑 1 次迭代 ──gba_result{seq, partial}──►
  partial 到了，沒有新的 loop： ──gba_continue{seq}──►   再跑 1 次 ──partial──►
  ……
  最後一次（10 次或收斂）：                                       ──gba_result{seq, final, 狀態}──►
  有新的 loop／merge：不送 continue，換掉 seq（原版的 mbStopGBA、mnFullBAIdx）
```

- 中止的粒度是一次迭代，與原版 g2o 每次迭代之間看 `mbStopGBA` 相同（視覺 10 次、inertial 7 次）。
- `gba_solver` 收到 seq 不同的 `gba_problem` 就丟掉手上的問題；`continue` 的 seq 對不上就忽略。
- `partial` 只帶序號與迭代次數，不帶狀態，不寫回。
- **nls 要能接著算**：λ、上一次的 cost、「連續 3 次下降不到千分之一」的計數、`analyzePattern` 的結果都要保留在 `nls::Problem` 裡跨呼叫（新的 `solveSome(n)` 之類的介面）。結果必須與一次跑完相同，`nls_test` 加一個比對。
- `ba_solver` 也用同一個 class，只是它收到的問題都一次跑完（不送 partial）。

**寫回**（原版 `RunGlobalBundleAdjustment` 的後半）：`final` 的結果先存在 `local_mapper.gba`，在 local mapping 下一次停下時寫回：最佳化過的 keyframe 直接設；之後才加入的 keyframe 從 `keyframe_origins` 沿 spanning tree 傳遞修正（與 `initialize_imu` 第 3 階段相同，抽成共用函式）；點依 `mnBAGlobalForKF` 或參考 keyframe 修正；`InformNewBigChange`、`IncreaseChangeIndex`。

**啟動條件**照原版：`!isImuInitialized() || (KeyFramesInMap() < 200 && CountMaps() == 1)`。

**耗時**（Pi 5，要量）：寫回的成本已在 [atlas 設計](../../../types/map/docs/DESIGN.md#第-0-步效能實測)量過：每個點都改一次，1 萬點 7.8 ms、5 萬點 28.7 ms，相對於 GBA 本身的秒級計算可以忽略，而且 tracking 不等。真正貴的是解 BA，nls 目前是單執行緒。階段 3 先量，再決定：

1. nls 的殘差、Jacobian、每個點的 Schur 區塊用 `fw::parallel_for`，`gba_solver` 設 `THREADS(2–3)`（Pi 5 有 4 核，扣掉 extractor 與 tracking）。local BA 也會跟著變快。
2. 寫回時跳過位置幾乎沒變的點（不 `edit` 就不複製 tile）。
3. 複製問題時共用 `shared_ptr<const>` 的 `Preintegrated`，不深拷貝。

## local mapping 的樹

```
local_mapping
└ ReactiveSequence
  ├ SubTree receive_inputs
  │   + ForceSuccess( SkipUnlessUpdated(loop_result) → apply_loop_result )
  │   + ForceSuccess( SkipUnlessUpdated(gba_result)  → step_global_ba )
  ├ ForceSuccess( Repeat(-1)( Sequence[ has_new_keyframe, SubTree process_keyframe ] ) )
  │                                   has_new_keyframe 多一個條件：!stop_requested
  ├ ForceSuccess
  │ └ Sequence
  │   ├ stop_requested                有 loop／merge 要修正，或 GBA 的結果要寫回
  │   ├ empty_queue                   EmptyQueue：佇列中的 keyframe 只做 ProcessNewKeyFrame（GBA 寫回時不做，與原版相同）
  │   └ Fallback
  │     ├ Sequence[ is_pending{loop},  CorrectLoop ]          stateful：同步部分 → pose_graph(_4dof) → 寫回 → 啟動 GBA
  │     ├ Sequence[ is_pending{merge}, MergeMaps ]            stateful：MergeLocal 或 MergeLocal2 的各段
  │     └ Sequence[ is_pending{gba},   apply_global_ba ]
  │   （結束時 Release：stop_requested = false、stopped = false）
  └ mapper_idle                       ResetIfRequested、SetAcceptKeyFrames(true)；stopped 時送出 isStopped
```

- **`apply_loop_result`**（同步，`receive_inputs` 裡）：`epoch` 對不上就丟；`erase_flags` 照順序套用，keyframe 已經是 bad 的跳過；`kind` 是 loop 或 merge 時，matched keyframe 還是 good 才記成 pending 並設 `stop_requested`、`abort_ba`（`RequestStop`），否則整筆丟掉並計數。
- **`step_global_ba`**（同步）：partial 時送 `gba_continue`；final 時存結果、設 pending gba 與 `stop_requested`。若期間來了新的 loop，`CorrectLoop` 已換掉 seq，舊的結果在這裡被丟掉。
- **RUNNING 只出現在 `CorrectLoop`、`MergeMaps`**（等 `ba_solver`）。這期間 `has_new_keyframe` 為 false（`stop_requested`），tracking 看到 `stopped`／`stop_requested` 就不建 keyframe（`need_new_keyframe` 已經這樣判斷），與原版 LM 停著時相同。
- **GBA 本身不是 RUNNING**：它在 `gba_solver` 上跑好幾秒，期間 local mapping 照常處理 keyframe（原版也是），只有寫回時停一下。
- 這一段放在新的子樹檔 `graphs/trees/local_mapping/correct_map.tree.yaml`。葉節點的程式碼放在 `algorithms/orb_loop_closing/`（邏輯是 loop closing 的，只是由 local mapping 的樹執行）。

## `send_to_loop_closing` 的改變

原版 `NewDetectCommonRegions` 一取出 keyframe 就 `SetNotErase()`，最後（不論結果）`mpKeyFrameDB->add()`。這兩件事都是寫入，改在 local mapping 送出 keyframe 的同一個 tick 做：

| 原版（LC 執行緒） | 這裡（`send_to_loop_closing`，atlas 改為 write） |
|---|---|
| `mpCurrentKF->SetNotErase()` | 同，在送出之前 |
| 偵測，然後 `mpKeyFrameDB->add(mpCurrentKF)` | 送出之前就加進 BoW 索引；`DetectNBestCandidates` 排除查詢的 keyframe 自己。候選與原版相同（原版查詢時它還不在索引裡） |
| 沒偵測到時 `SetErase()` | `loop_result.erase_flags` |

好處：`SetNotErase` 不會晚一個 tick（loop closing 處理之前它不會被 `keyframe_culling` 刪掉），relocalization 也立刻查得到這個 keyframe。

**時序造成的差異**（接受，與原版的競態同性質）：`DetectCommonRegionsFromBoW` 對候選 keyframe 的 `SetNotErase` 要等 `loop_result` 到了才套用，這之間候選可能被刪。被刪的候選照「忽略」處理：`apply_loop_result` 跳過；loop closing 在下一個快照上檢查留著的 `mpLoopMatchedKF`、`mpMergeMatchedKF`、`mpLoopLastCurrentKF`，不是 good 就重設那一組偵測狀態。計數器記下次數，用來確認很少發生。

## 資料放在哪裡

| 放在 | 內容 |
|---|---|
| **埠** | `loop_keyframe`／`keyframe`、`loop_result`、`gba_problem`、`gba_continue`、`gba_result`、`params`、`vocabulary` |
| **`atlas`**（write：local mapping；read：tracking、loop closing） | 新增：`KeyFrame::loop_edges`、`merge_edges`（已有欄位）；merge 時 keyframe 與點換地圖、`current` 換地圖、`bad_maps` |
| **`local_mapper`** | 新增：`reset_epoch`；pending 的 loop／merge（`LoopResult` 的內容）；`stop_requested`／`stopped`（已有欄位，開始使用）；GBA 的 seq、迭代、final 的結果；GBA 的查詢暫存（`KeyFrameScratch`、`PointScratch` 已有） |
| **`loop_closing` 的成員** | `LoopClosing` 的偵測狀態；`DetectNBestCandidates` 的查詢暫存（`mnPlaceRecognitionQuery`、`mnPlaceRecognitionWords`、`mPlaceRecognitionScore`），與 tracker 的 relocalization 暫存同理 |

原版 `MapPoint::mnCorrectedByKF`、`mnCorrectedReference`、`KeyFrame::mnMergeCorrectedForKF`、`mnMergeForKF`、`mnLoopQuery` 等都只在一次修正或一次查詢裡有意義，改成該次呼叫裡的 set，不保存。

## atlas 的新操作（`types/map/atlas_ops`）

| 操作 | 原版 |
|---|---|
| `setNotErase`、`setErase` | `KeyFrame::SetNotErase`、`SetErase`（`to_be_erased` 時 `SetBadFlag`） |
| `addLoopEdge`、`addMergeEdge` | `KeyFrame::AddLoopEdge`、`AddMergeEdge`（同時 `SetNotErase`） |
| `changeMap`、`setMapBad`、`removeBadMaps` | `Atlas::ChangeMap`、`SetMapBad`、`RemoveBadMaps` |
| `moveKeyFrameToMap`、`movePointToMap` | `KeyFrame::UpdateMap`、`MapPoint::UpdateMap`（含 `Map::AddKeyFrame`／`EraseKeyFrame` 與點數） |
| `fuseWithPose` | `ORBmatcher::Fuse(pKF, Scw, vpPoints, th, vpReplacePoint)` 的 SE3 版 |

`applyScaledRotation`、`updateConnections`、`changeParent`、`replace` 已有。

## tracking 端

不需要新的介面：

- 修正與 GBA 寫回都 `IncreaseChangeIndex`，tracking 的 `begin_frame` 已依 `change_index` 設 `map_updated`，`predict_state_imu` 改從 last keyframe 預測（原版的 `mbMapUpdated`）。
- `MergeLocal2` 的 `mpTracker->UpdateFrameIMU` 走既有的 `MapperStatus::imu_update_seq`。
- merge 換掉 `atlas.current`；tracking 每幀從 `sync_atlas` 的快照讀 `current`。stereo（`MergeLocal`）時目前的 keyframe 視窗移到合併後的地圖座標，tracking 的上一幀依參考 keyframe 重算位姿（`UpdateLastFrame`）。這一點要在階段 4 實跑確認。

## 測試地圖合併：`frame_gate`

只有 tracking 開了新地圖之後才會合併，V1_02 照常跑通常不會。

- **不能整段刪**：影像與 IMU 一起缺 2 秒，`imu_preintegration` 的 IMU gap 檢查（相鄰樣本間隔超過中位數 3 倍）會讓它進入 error，`on_error: stop_box`。
- **只丟影像**：IMU 照常送，gap 與 stalled 都不會觸發，缺口後第一幀的預積分包含這 2 秒。tracking 的 `frame_discontinuity` 看到「inertial 地圖裡影格時間跳了超過 1 s」，`handle_discontinuity` 在 BA2 做完時 `createMapInAtlas`（否則重設 active map）。這正是原版產生第二張地圖的路徑。

做法：測試用的 block `frame_gate`（`algorithms/frame_gate/`），接在 driver 與兩個 extractor 之間，丟掉設定時間區段內的左右影像，IMU 不經過它。用在新的 `graphs/slam_merge_job.graph.yaml`，`slam_job` 不變。缺口放在約 30–32 s：BA2 在 IMU 初始化後約 15 s，新地圖的 IMU 也要再初始化（stereo-inertial 的合併要求目前的地圖 IMU 已初始化），回到舊區域時才會合併。

階段 0 先實跑確認：沒有 `stop_box`，有第二張地圖。

## 程式碼的配置與行數（估計）

```
types/
  loop_closing/loop_keyframe.hpp     loop_closing/LoopKeyFrame                                ~20
  loop_closing/loop_result.hpp       loop_closing/LoopResult                                  ~60
  ba/problem.hpp                     + 新的 kind、位姿圖的邊、gba 的 continue／partial          +80
  local_mapping/local_mapper.hpp     + reset_epoch、pending loop／merge／gba                    +50
  map/atlas_ops                      + 上表的操作                                              +250
algorithms/
  orb_loop_closing/
    loop_closing/                    block：Run() 的一輪、偵測狀態、reset                      ~250
    place_recognition/               NewDetectCommonRegions、FromBoW、refinePoseFromLastKeyFrame、FindMatchesByProjection  ~450
    keyframe_candidates/             DetectNBestCandidates                                    ~200
    keyframe_matching/               searchByBoW(kf, kf)、searchByProjection(kf, Tcw, …)×2  ~450
    relative_pose_ransac/            RelativePoseRansac（原版 Sim3Solver，s = 1）              ~250
    optimize_relative_pose/          optimizeRelativePose（原版 OptimizeSim3，nls）             ~200
    correct_loop/                    CorrectLoop、SearchAndFuse                               ~350
    merge_maps/                      MergeLocal、MergeLocal2、CheckObservations               ~700
    global_ba/                       step_global_ba、apply_global_ba、spanning tree 的傳遞     ~250
    其餘葉節點                        apply_loop_result、stop_requested、empty_queue、is_pending  ~150
    docs/DESIGN.md
  ba_solver/
    pose_graph.*                     pose_graph、pose_graph_4dof、pose_graph_merge             ~450
    global.*                         global（重用 local 的重投影）                              ~120
    merge_inertial.*                 merge_inertial、inertial_bias                             ~250
    ba_solver.cpp                    + 分段迭代                                                +60
  frame_gate/                        測試用                                                    ~50
libs/nls/                            + 接著算（solveSome）；之後 parallel_for                   +100
graphs/
  slam_job.graph.yaml、slam_merge_job.graph.yaml
  trees/local_mapping/correct_map.tree.yaml、receive_inputs.tree.yaml
                                                                     合計約 5,000 行（不含測試）
```

`initialize_imu` 第 3 階段的 spanning tree 傳遞抽到 `orb_local_mapping/shared/`，與 `apply_global_ba` 共用。

## 驗證

| 階段 | 對象 | 方式 |
|---|---|---|
| 0 | relocalization | 跑 V1_02，`relocalize` 有候選（目前永遠沒有）；`wake` 下 job 能自己結束 |
| 0 | `frame_gate` | 沒有 `stop_box`，出現第二張地圖 |
| 1 | `DetectNBestCandidates`、`searchByBoW(kf, kf)`、`searchByProjection`、`RelativePoseRansac`、`optimizeRelativePose` | `fixture_gen` 的答案（合成場景：兩段軌跡經過同一處） |
| 1 | 偵測整體 | 跑 V1_02，與原版的 `[LC]` 輸出（`config.debug.loop_closing`）對照偵測到 loop 的 keyframe 與時間 |
| 2 | `CorrectLoop` 的同步部分、`pose_graph`、`pose_graph_4dof` | fixture |
| 2 | 整體 | ATE、loop 次數與原版相近 |
| 3 | 分段迭代 | `nls_test`：分段跑與一次跑完結果相同 |
| 3 | GBA | ATE；Pi 5 上 GBA 的耗時、寫回的耗時 |
| 4 | `MergeLocal2`、`MergeInertialBA`、`MergeLocal` | fixture；`slam_merge_job` 合併後的 ATE |

## 待決定

| 事項 | 現況 | 何時決定 |
|---|---|---|
| `wake` 下 job 的結束 | 先用 `wake` | 階段 0 實測 |
| ~~`loop_closing.keyframe` 的佇列深度~~ | **已定：16**（偵測平均 2.8 ms，最大 19 ms） | |
| nls 平行化的程度 | 單執行緒 | 階段 3 量 GBA 的耗時 |
| 寫回時跳過沒變的點的門檻 | 不跳過 | 階段 3 量寫回的耗時 |
| ~~Sim3 或 SE3~~ | **已定：SE3** | |
| ~~誰寫 atlas~~ | **已定：只有 local mapping（方案 B）** | |
| ~~被刪掉的候選~~ | **已定：忽略並計數** | |
| ~~地圖合併~~ | **已定：要做（階段 4）** | |
