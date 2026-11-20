# orb_local_mapping 設計：以行為樹實作 ORB-SLAM3 的 local mapping

這份文件只定架構：圖上怎麼連、資料放在哪裡、樹的邏輯、鎖怎麼拿。演算法的部分目前是空殼，架構確認後再填。

**目前的狀態**：

| | 狀態 |
|---|---|
| 樹檔、圖檔、全部葉節點的埠與資源 | 完成，`fw check graphs/slam_job.graph.yaml` 通過（含環的規則） |
| 簿記：`read_params`、`apply_tracking_edits`、`handle_mapper_command`、`publish_mapper_status`、`has_new_keyframe`、`queue_empty`、`ready_for_local_ba`、`enough_keyframes_for_ba`、`is_inertial_uninitialized`、`mapper_idle`、`reset_if_requested`、`send_to_loop_closing` | 已實作 |
| `process_new_keyframe`（`ProcessNewKeyFrame`） | 已實作（`shared/process_keyframe`） |
| `local_bundle_adjustment` | 只有流程：送出空的問題、等 `ba_result` 的序號、回傳 SUCCESS；動作窗、問題的內容與寫回是 TODO |
| `ba_solver` | 只有流程：把問題原樣送回 |
| `map_point_culling`（`MapPointCulling`） | 已實作（`map_point_culling/culling`），單元測試 `orb_local_mapping_map_point_culling_test` |
| `create_new_map_points`（`CreateNewMapPoints`、`SearchForTriangulation`、KB8 的 `TriangulateMatches`） | 已實作（`create_new_map_points/triangulation`），與原版的 fixture 一致：`orb_local_mapping_new_map_points_test`（`tests/fixtures/orb_slam3/new_map_points`） |
| `search_in_neighbors`、`keyframe_culling`、`initialize_imu`、`begin_inertial_ba` | 空殼（`initialize_imu` 直接 SUCCESS，`begin_inertial_ba` 直接 FAILURE） |
| tracking 端的 `tracking_ok_seq` | 型別已加，`apply_mapper_update` 尚未改 |

**骨架的實跑**（`fw launch graphs/slam_job.graph.yaml`，V1_02_medium，stereo-inertial）：tracking 從頭到尾都是 OK，處理了 300 多個 keyframe（以前臨時加入 `ProcessNewKeyFrame` 時是 324 個），每影格 240–500 個 inlier。從第 3 個 keyframe 起，每個 keyframe 都完成一次 `ba_problem → ba_solver → ba_result` 的 wake 來回。tracking 在 local mapping 忙時不送 keyframe，所以佇列自然不會超過 1 個。另外以暫時的追蹤輸出測過佇列有 3 個的情形（每 10 個 keyframe 重複放入 3 次，`ba_solver` 多睡 300 ms）：每個 keyframe 各用一個 tick，tick 之間沒有新資料時由 fw 自己補上，三個都處理完才進 `mapper_idle`（見「樹的邏輯」）。

**加入 `map_point_culling`、`create_new_map_points` 之後的實跑**（同上，以暫時的追蹤輸出觀察，之後已移除）：tracking 仍然從頭到尾都是 OK，300 個 keyframe 時每影格 280–550 個 inlier。每個 keyframe 三角化出 40–110 個新點，`MapPointCulling` 之後地圖的點數穩定增加（keyframe 100 時約 2,900 個、300 時約 8,000 個），`recent_points` 保持在 300–700 個。local BA 仍是空殼，所以新點的位置沒有經過最佳化。

`SearchForTriangulation` 的 matcher 是 `ORBmatcher(0.6, false)`，不做方向檢查，所以原本計畫的 `rotation_histogram.hpp` 搬到 `types/orb/` 不需要做。

- 樹檔：[`graphs/trees/local_mapping/`](../../../graphs/trees/local_mapping/)
- 圖檔：[`graphs/slam_job.graph.yaml`](../../../graphs/slam_job.graph.yaml)（tracking + local mapping；[`tracking_job.graph.yaml`](../../../graphs/tracking_job.graph.yaml) 保持不變）
- tracking 的設計：[`orb_tracking/docs/DESIGN.md`](../../orb_tracking/docs/DESIGN.md)。本文沿用其中的原則（以 id 引用、型別只放資料、程式碼依使用者分散放置），不再重複。
- 原始碼的對照：ORB-SLAM3 v1.0 的 `LocalMapping`（`Run`、`ProcessNewKeyFrame`、`MapPointCulling`、`CreateNewMapPoints`、`SearchInNeighbors`、`KeyFrameCulling`、`InitializeIMU`），以及 `Optimizer` 的 `LocalBundleAdjustment`、`LocalInertialBA`、`InertialOptimization`、`FullInertialBA`。

## 範圍

| | 這一步 | 之後 |
|---|---|---|
| 感測器 | stereo 與 stereo-inertial，KB8（與 tracking 相同） | |
| local mapping | `Run()` 的一次迴圈的全部步驟，含 IMU 初始化與 VIBA 1、2 | `Stop`／`Release`（由 loop closing 要求）：樹上已留位置，`stop_requested` 永遠為 false |
| 最佳化 | 新的 `ba_solver` block，在鎖外計算 local BA、local inertial BA、inertial-only 初始化、full inertial BA | loop closing 的 global BA、essential graph：另一個 `ba_solver` 實例或另一種問題 |
| loop closing | 不做；local mapping 每處理完一個 keyframe 送出 `loop_keyframe`（取代 `mpLoopCloser->InsertKeyFrame`） | 接上 `loop_keyframe` 與 `atlas` |
| 單目的 `ScaleRefinement` | 不做（只用於單目） | |

**KeyFrameDatabase 的加入**：原版由 loop closing 的 place recognition 把 keyframe 加進 BoW 索引（`mpKeyFrameDB->add`），local mapping 不加。沒有 loop closing 時 relocalization 永遠找不到候選。已定照原版（見[待決定](#待決定)）。

## 圖

```
 ┌───────────────────────────────────── box main ────────────────────────────────────────────┐
 │  …（與 tracking_job 相同：driver → extractor × 2 → matcher → imu_preintegration）           │
 │                          │ frame_imu                                                       │
 │                          ▼                                                                 │
 │   ┌──────────── tracking (behavior) ─────────────┐                                         │
 │   │ frame (trigger)   mapper (latest) ◄──────────┼──────────────────────────────┐          │
 │   │ ──► pose  tracking  atlas_edits  mapper_cmd  │                              │          │
 │   └──────────────────────┬─────────┬────────┬────┘                              │          │
 │              (latest)    │ trigger │ trigger│                                   │          │
 │                          ▼         ▼        ▼                                   │          │
 │   ┌──────────── local_mapping (behavior) ──────────────────────────────────┐    │          │
 │   │ tracking (latest)  atlas_edits (trigger)  mapper_cmd (trigger)         │    │          │
 │   │ params (latest) ◄── config://orb_tracking.yaml                         │    │          │
 │   │ ba_result (wake) ◄──────────────────────┐                              │    │          │
 │   │                                         │                              │    │          │
 │   │ ──► mapper ────────────────────────────────────────────────────────────┼────┘          │
 │   │ ──► ba_problem ──┐                      │                              │               │
 │   │ ──► loop_keyframe ──► loop_closing（之後）                              │               │
 │   └──────────────────┼──────────────────────┼──────────────────────────────┘               │
 │                      ▼ trigger              │                                              │
 │              ┌─── ba_solver ───┐  result ──┘     （不接觸任何資源）                         │
 │              └─────────────────┘                                                           │
 │                                                                                            │
 │   atlas (r: tracking, w: local_mapping)   tracker (w: tracking)                            │
 │   local_mapper (w: local_mapping)      vocabulary (r: tracking, local_mapping)   （資源）  │
 └────────────────────────────────────────────────────────────────────────────────────────────┘
```

- **local_mapping** 是 behavior block，樹是 [`local_mapping.tree.yaml`](../../../graphs/trees/local_mapping/local_mapping.tree.yaml)。tick 的來源有三個：
  - `atlas_edits`（`trigger`）：tracking 對地圖的修改（[types/map/docs/DESIGN.md](../../../types/map/docs/DESIGN.md#tracking-改成只讀)），由 local mapping 寫入 `atlas`。其中新 keyframe 對應 `InsertKeyFrame`，`reset_all`／`reset_active_map` 對應 `RequestReset`／`RequestResetActiveMap`；
  - `mapper_cmd`（`trigger`）：tracking 的 `InterruptBA`；
  - `ba_result`（`wake`）：`ba_solver` 算完的結果。`wake` 不計入環的規則，也不計入資料結束的判定。
- `tracking`（`latest`）：tracking 的逐影格狀態。local mapping 只讀其中的 `state`（`CreateNewMapPoints` 的 `bCoarse`、VIBA 的 `mState == OK`）與 `inliers`（`LocalInertialBA` 的 `bLarge`），取代原版直接讀 `mpTracker`。
- `params`：與 tracking 同一份 `config://orb_tracking.yaml`、同一個型別 `OrbTrackingParams`，只用到 `sensor`。不另開一份設定，免得兩邊的 `sensor` 不一致。
- **輸出**：
  - `mapper`（`tracking/MapperStatus`）→ `tracking.mapper`。behavior block 內建的輸出已經叫 `status`，所以這個埠叫 `mapper`。
  - `ba_problem` → `ba_solver.problem`：最佳化問題，在鎖內從 `atlas` 複製出來（[鎖外計算](#鎖外計算)）。
  - `loop_keyframe`（`map/KeyFrameRef`）：每處理完一個 keyframe 送出，給之後的 loop closing。
- **ba_solver** 是一般的 algorithm（class 形式），沒有任何資源：收到 `ba/Problem` 就解，送出 `ba/Result`。

**環**：

| 環 | 為什麼成立 |
|---|---|
| `tracking.atlas_edits → local_mapping → mapper → tracking.mapper` | `tracking.mapper` 是 `latest`，讀它的葉節點都以 `fw::Optional` 接收（R1、R2） |
| `tracking.tracking → local_mapping.tracking` | 同上，`local_mapping.tracking` 是 `latest`，以 `fw::Optional` 接收 |
| `local_mapping.ba_problem → ba_solver → ba_result` | `ba_result` 是 `wake`，不計入環的規則；`ba_solver.problem` 的佇列深度 2，不觸發 R5 的警告 |

## 原版的執行緒在這裡怎麼跑

原版 `LocalMapping::Run()` 是一條執行緒的無窮迴圈，每一輪：有 keyframe 就完整處理一個，然後 `ResetIfRequested`、`SetAcceptKeyFrames(true)`、睡 3 ms。這裡沒有執行緒，改由 tick 推動：

| 原版 | 這裡 |
|---|---|
| `InsertKeyFrame`：放進 `mlNewKeyFrames`，`mbAbortBA = true` | `atlas_edits` 的 tick；`apply_tracking_edits` 把 keyframe 寫入 `atlas`、放進 `local_mapper.queue`，設 `abort_ba` |
| `while(1)` 每一輪處理一個 keyframe | `Repeat(-1)` 每個 tick 處理一個 keyframe，fw 自行補 tick 直到佇列空了，或某一步要等最佳化（RUNNING） |
| `Optimizer::LocalBundleAdjustment` 在 LM 執行緒內算，結束才拿 `mMutexMapUpdate` 寫回 | stateful action：`ON_START` 在鎖內複製問題並送出，回傳 RUNNING；`ba_result` 的 wake tick 進到 `ON_RUNNING`，在鎖內寫回 |
| `CheckNewKeyFrames()`（佇列是否有下一個） | 看 `local_mapper.queue`。最佳化進行中到達的 keyframe 由 `apply_tracking_edits` 放進佇列，所以判斷與原版相同 |
| `mbAcceptKeyFrames`：處理時 false，閒置時 true | `accept_keyframes` = 沒有 keyframe 正在處理；每個 tick 都送出 `mapper` |
| `RequestResetActiveMap` 讓 tracking 等到 LM 重設完，tracking 才清掉地圖 | tracking 不等：重設是一筆 edit，`apply_tracking_edits` 依序套用（清掉地圖），丟掉排在它之前的 keyframe，LM 自己的 `ResetIfRequested` 在這個 keyframe 處理完時（或閒置時立即）執行，與原版的位置相同 |

**樹的根是 `ReactiveSequence`**：最佳化進行中（某個子節點 RUNNING）時，新的 `atlas_edits` 或 `mapper_cmd` 的 tick 仍然要被接收。`ReactiveSequence` 每個 tick 都從第一個子節點重新執行，所以 `receive_inputs` 子樹每個 tick 都會跑；後面 RUNNING 的處理子樹則從中斷的地方繼續。`receive_inputs` 只含同步的葉節點，不會打斷後面的 RUNNING 節點。

**只處理新到的資料**：鍵只保存最新的值，`ba_result` 的 wake tick 時鍵 `atlas_edits` 仍是上一筆。`apply_tracking_edits`、`handle_mapper_command` 各包在 `SkipUnlessUpdated` 裡，只在這個 tick 有新的值時執行。

## 鎖外計算

local BA 在 EuRoC 上一次約 20–60 ms（Pi 上更久），IMU 初始化的 full inertial BA（100 次迭代、整張地圖）可達數百 ms 到秒。behavior block 每個 tick 持有它用到的所有資源。`atlas` 是 `versioned`，tracking 不會等；但若在 tick 內算，這段時間 local mapping 收不到 tracking 的 edit（tracking 新 keyframe 的 `ProcessNewKeyFrame` 也跟著延後），之後 loop closing 寫 `atlas` 時也要等。所以依[鎖外計算](../../../docs/concepts/resources.md#長時間運算鎖外計算)拆成三段：

```
local_mapping (tick N, 持有 atlas)        ba_solver (不持有資源)          local_mapping (tick N+k, 持有 atlas)
  ON_START: 建問題（複製）──ba_problem──►   解                 ──ba_result──►  ON_RUNNING: 依 id 寫回
            回傳 RUNNING                                                            回傳 SUCCESS
```

- **問題是複本**：keyframe 的位姿、速度、bias、預積分（`imu::Preintegrated` 整份複製，不共用 atlas 內的 `shared_ptr`）、相機、map point 的位置、觀測（像素、層級的 σ²、左右眼）、固定與否。問題不含任何指向 `atlas` 的指標或參考。
- **寫回依 id**：計算期間 tracking 會加入 keyframe 與點，也可能 `ResetActiveMap`。寫回時只動問題裡有的 id；已經 bad、已經不在原來地圖的跳過；新加入的不動。原版的 LBA 同樣與 tracking 同時執行，只有寫回時拿 `mMutexMapUpdate`，行為相同。
- **序號**：每個問題帶 `seq`，`local_mapper.ba_seq` 記下等待中的序號。對不上的結果（被 halt 之後才到的、重設之前送出的）丟棄。
- **ba_solver 的佇列深度 2**：local mapping 同一時間只會有一個問題在算，深度只是保險，不丟問題。

**中止（`mbAbortBA`）**：原版有三個地方看這個旗標：

| 原版 | 這裡 |
|---|---|
| `LocalBundleAdjustment` 在 `optimize` 之前看到旗標就直接返回 | 相同：`local_bundle_adjustment` 的 `ON_START` 看 `abort_ba`，已設就不送問題、回傳 SUCCESS |
| g2o 的 `setForceStopFlag`：每次迭代之間看旗標，提早結束 | **不做**。`ba_solver` 在一次呼叫內算完，看不到之後的旗標。差異：keyframe 來得很快時，LM 晚幾十 ms 才處理下一個。結果仍是一次正常的最佳化 |
| `KeyFrameCulling` 的 `count > 20 && mbAbortBA` | 相同：`keyframe_culling` 看 `abort_ba` |

`LocalInertialBA` 在 `optimize` 之後才設 force-stop 旗標，等於不會中止，這裡也一樣。

**IMU 初始化也拆開**：`InitializeIMU` 有兩次長的最佳化，中間與之後都要改地圖，所以 `initialize_imu` 是有三個階段的 stateful action：

| 階段 | 在鎖內（tick） | 在鎖外（ba_solver） |
|---|---|---|
| 1 | 條件（≥10 個 keyframe、≥1 s、沒有重設要求）、處理佇列中的 keyframe、依時間排序的 keyframe、重力方向與速度的初值（`Rwg`、`SetVelocity`）、建 inertial-only 問題 | `InertialOptimization`（`Rwg`、scale、bias、速度） |
| 2 | scale < 0.1 就結束；寫回速度與 bias、`ApplyScaledRotation`、`UpdateFrameIMU`（`mapper` 的 `imu_update_seq`）、`bImu`、`SetImuInitialized`；`bFIBA` 時建 full inertial BA 問題 | `FullInertialBA`（priorA ≠ 0 時帶 bias 先驗） |
| 3 | 處理佇列中的 keyframe；從 `keyframe_origins` 沿 spanning tree 把修正傳到最佳化之後才加入的 keyframe（`mTcwGBA` 的傳遞）；修正 map point；佇列中剩下的 keyframe 設為 bad（原版如此）；tracking 的 `mState = OK`（`mapper` 的 `tracking_ok_seq`）；`IncreaseChangeIndex` | |

原版「處理佇列中的 keyframe」會改變 `mpCurrentKeyFrame`，後面的 `KeyFrameCulling` 與送給 loop closing 的都是最後處理的那一個；`local_mapper.current` 照這樣改。

## 資料放在哪裡

| 放在 | 內容 | 理由 |
|---|---|---|
| **埠** | `atlas_edits`、`mapper_cmd`、`tracking`、`params`；`mapper`、`loop_keyframe`；`ba_problem`、`ba_result` | 單向、不可變 |
| **資源 `atlas`**（write） | keyframe、map point、地圖（含 `imu_initialized`、`imu_ba1`、`imu_ba2`）、BoW 索引 | tracking 只在每幀開頭讀一次（`sync_atlas`），寫入都由 local mapping 做 |
| **資源 `local_mapper`**（`local_mapping/LocalMapper`，write，只有 local_mapping 用） | `LocalMapping` 的成員：keyframe 佇列（`mlNewKeyFrames`）、目前的 keyframe、`mlpRecentAddedMapPoints`、`abort_ba`、重設要求、`mbInertial`、`mTinit`、`mbBadImu`、`bInitializing`、IMU 初始化的狀態（`mRwg`、`mScale`、`mbg`、`mba`、`mIdxInit`、`mnKFs`）、`UpdateFrameIMU` 與 `mState = OK` 的序號、等待中的最佳化（序號、階段、寫回用的資料）、查詢暫存 | 與 `tracker` 同理：葉節點之間共用的狀態只能放在鍵或資源；這些成員大、每次都和 `atlas` 一起讀寫 |
| **資源 `vocabulary`**（read） | ORB 詞彙樹 | `ProcessNewKeyFrame` 的 `ComputeBoW` |
| **鍵** | `inertial`（樹的分支用，由 `receive_inputs` 依 `params.sensor` 寫入） | |

**查詢暫存**：原版放在 `KeyFrame`、`MapPoint` 上的 `mnBALocalForKF`、`mnBAFixedForKF`、`mnFuseTargetForKF`、`mnFuseCandidateForKF`、`mnBAGlobalForKF`、`mTcwGBA`、`mTcwBefGBA`、`mVwbGBA`、`mBiasGBA`、`mPosGBA` 由 local mapping 自己保存在 `local_mapper`（以 id 為索引）。之後 loop closing 的 global BA 也用 `mnBAGlobalForKF` 這組，由它自己另存一份。

**鎖的範圍**：每個 tick 取一次 `atlas`、`local_mapper` 的寫入鎖與 `vocabulary` 的讀取鎖。`atlas` 是 `versioned`，鎖與發布是分開的：寫入鎖持有到 tick 結束，但每個寫入 `atlas` 的葉節點回傳時就發布一個新版本（同一個 tick 裡後面的葉節點接著在它上面寫）。所以只讀 `atlas` 的葉節點要宣告成 `read`，否則每次都多一個空的版本。

- 只有接收資料的 tick（最佳化進行中到達的 `atlas_edits`、`mapper_cmd`）很短。
- 處理一個 keyframe 的同步部分（`ProcessNewKeyFrame` 到 `SearchInNeighbors`、`KeyFrameCulling`）在一個 tick 內完成。`atlas` 是 `versioned`，tracking 不等它，讀到的是最近發布的版本，與原版靠每個物件自己的 mutex 讓 tracking 同時進行相近（[types/map/docs/DESIGN.md](../../../types/map/docs/DESIGN.md#第-3-步的驗證)）。
- 新 keyframe 的 BoW 與觀測在 `process_new_keyframe` 回傳時就發布，tracking 的下一幀讀得到（原版的 `ProcessNewKeyFrame` 也很快）。它之後同一個 tick 的三角化、fuse 不會延後這件事，所以不必拆出去；耗時的工作只要別放進 `process_new_keyframe` 或它之前。

## 樹的邏輯

樹分成三個檔案：

```
local_mapping                          (根：atlas_edits、mapper_cmd 的 trigger，ba_result 的 wake)
└ ReactiveSequence
  ├ SubTree receive_inputs ───────────────────────────────────────────────┐
  ├ ForceSuccess                                                          │
  │ └ Repeat (-1)                      Run() 的 while(1)：一次一個 keyframe │
  │   └ Sequence                                                          │
  │     ├ has_new_keyframe             CheckNewKeyFrames() && !mbBadImu    │
  │     └ SubTree process_keyframe ────────────────────────────────────┐  │
  └ mapper_idle                        else 分支（Stop，之後）、         │  │
                                       ResetIfRequested、               │  │
                                       SetAcceptKeyFrames(true)、送出 mapper │
                                                                       │  │
receive_inputs                                                         │ ◄┘
└ Sequence
  ├ read_params                        inertial（params.sensor）
  ├ ForceSuccess( SkipUnlessUpdated(atlas_edits) → apply_tracking_edits ) InsertKeyFrame、RequestReset*
  ├ ForceSuccess( SkipUnlessUpdated(mapper_cmd)  → handle_mapper_command ) InterruptBA
  └ publish_mapper_status              accept、佇列長度、initializing、bad_imu…
                                                                       │
process_keyframe                       Run() 的一輪                     ◄┘
└ Sequence
  ├ process_new_keyframe               ProcessNewKeyFrame（取出、BoW、觀測、UpdateConnections、AddKeyFrame）
  ├ map_point_culling                  MapPointCulling
  ├ create_new_map_points              CreateNewMapPoints；之後 mbAbortBA = false
  ├ ForceSuccess( Sequence[ queue_empty, search_in_neighbors ] )        if(!CheckNewKeyFrames()) SearchInNeighbors
  ├ ForceSuccess
  │ └ Sequence
  │   ├ ready_for_local_ba             !CheckNewKeyFrames() && !stopRequested()
  │   ├ ForceSuccess( Sequence[ enough_keyframes_for_ba, local_bundle_adjustment ] )
  │   │                                KeyFramesInMap() > 2：LocalInertialBA 或 LocalBundleAdjustment
  │   ├ ForceSuccess( Sequence[ is_inertial_uninitialized, initialize_imu {1e2, 1e5, fiba} ] )
  │   ├ keyframe_culling               KeyFrameCulling
  │   └ ForceSuccess
  │     └ Fallback                     (mTinit < 50 && inertial && IMU 已初始化 && tracking OK)
  │       ├ Sequence[ begin_inertial_ba {stage: 1}, initialize_imu {1, 1e5, fiba} ]   VIBA 1（mTinit > 5）
  │       └ Sequence[ begin_inertial_ba {stage: 2}, initialize_imu {0, 0, fiba} ]     VIBA 2（mTinit > 15）
  ├ send_to_loop_closing               mpLoopCloser->InsertKeyFrame → loop_keyframe
  └ reset_if_requested                 迴圈結尾的 ResetIfRequested
```

閱讀時的幾個要點：

- **RUNNING 只出現在 `local_bundle_adjustment` 與 `initialize_imu`**。其餘葉節點都是同步的函式形式。RUNNING 時 `Repeat` 回傳 RUNNING，`mapper_idle` 不執行；下一個 tick 由 `ba_result` 的 wake 引發。
- **`Repeat(-1)` 每一輪之間讓出 tick**（已實測，BT.CPP 4.11 的語意：子節點同步 SUCCESS 後 `Repeat` 回傳 RUNNING 並發出 wake-up，fw 收到後自己補一個 tick）。所以一個 keyframe 一個 tick：每一輪之間 `receive_inputs` 會重跑一次（新 keyframe 與指令照樣收進來），資源的鎖也在每一輪之間放開一次，tracking 可以插進來。佇列空了，`has_new_keyframe` FAILURE，`Repeat` FAILURE，`ForceSuccess` 收掉，才進 `mapper_idle`。
- **RUNNING 跨 tick 時不會被中止**：`ReactiveSequence` 只在前面的子節點 FAILURE 時中止後面的子節點，而 `receive_inputs` 的葉節點都只回 SUCCESS。等最佳化時進來的 tick 重跑 `receive_inputs` 之後，直接回到等待中的葉節點的 `running()`。`abort_ba` 擋不住已送出的 problem（`ba_solver` 看不到它），結果照樣寫回；結果若一直不回來樹會停在 RUNNING，之後可考慮包 `Timeout`。
- **`initialize_imu` 出現三次**，各是獨立的物件，常值 `prior_g`、`prior_a`、`fiba` 不同。它們不保存自己的狀態以外的東西：等待中的階段與寫回用的資料在 `local_mapper`，`ON_HALTED` 清掉。
- **VIBA 的 `else if`**：原版是 `if (!BA1) { if (mTinit > 5) … } else if (!BA2) { if (mTinit > 15) … }`。`begin_inertial_ba {stage: 2}` 的條件包含「BA1 已完成」，所以 BA1 還沒做時不會跳去做 BA2；`SetIniertialBA1/2` 在呼叫 `InitializeIMU` 之前設定，與原版相同（即使 `InitializeIMU` 提早返回，旗標也已設）。它要改地圖的旗標，所以是 action 而不是 condition。
- **`mbBadImu` 的重設**：`local_bundle_adjustment` 在 inertial 分支中，若 `mTinit < 10` 且移動距離 < 0.02 m，設定 `bad_imu` 與重設 active map 的要求，與原版相同。原版的 tracking 只在 LM 這一輪結束（`ResetIfRequested` 清掉 `mbBadImu`）之前看得到這個旗標；這裡 BA 進行中的每個 tick 都送出 `mapper`，tracking 看得到它的時間窗與原版相近。
- **`has_new_keyframe` 的 `!mbBadImu`**：原版 bad IMU 時不處理 keyframe，等 `ResetIfRequested` 清掉佇列。這裡相同，`mapper_idle` 會執行重設。

## 葉節點

`atlas`、`local_mapper`、`vocabulary` 在樹檔中不寫，依同名規則接到樹的資源。名稱在整個專案中唯一（tracking 已用 `state_is`、`is_true`）。

| 葉節點 | 種類 | 埠（鍵） | 資源 | 對應的 ORB-SLAM3 |
|---|---|---|---|---|
| `read_params` | action | `params`、`inertial` out | local_mapper w | `mbInertial` |
| `apply_tracking_edits` | action | `atlas_edits` | atlas w、local_mapper w | 套用 tracking 的修改；`InsertKeyFrame`（`mbAbortBA = true`）、`RequestReset`、`RequestResetActiveMap`（不等待） |
| `handle_mapper_command` | action | `mapper_cmd` | local_mapper w | `InterruptBA` |
| `publish_mapper_status` | action | `mapper` out | atlas r、local_mapper r | tracking 讀的 `AcceptKeyFrames`、`KeyframesInQueue`、`IsInitializing`、`isStopped`、`stopRequested`、`mbBadImu`、`mbFarPoints` |
| `has_new_keyframe` | condition | | local_mapper r | `CheckNewKeyFrames() && !mbBadImu` |
| `mapper_idle` | action | `mapper` out | atlas w、local_mapper w | `Stop()`（之後）、`ResetIfRequested`、`SetAcceptKeyFrames(true)` |
| `process_new_keyframe` | action | | atlas w、local_mapper w、vocabulary r | `ProcessNewKeyFrame`、`SetAcceptKeyFrames(false)` |
| `map_point_culling` | action | | atlas w、local_mapper w | `MapPointCulling` |
| `create_new_map_points` | action | `tracking` Optional | atlas w、local_mapper w | `CreateNewMapPoints`（`SearchForTriangulation`、三角化）；結束時 `mbAbortBA = false` |
| `queue_empty` | condition | | local_mapper r | `!CheckNewKeyFrames()` |
| `search_in_neighbors` | action | | atlas w、local_mapper w | `SearchInNeighbors`（`Fuse` 兩個方向） |
| `ready_for_local_ba` | condition | | local_mapper r | `!CheckNewKeyFrames() && !stopRequested()` |
| `enough_keyframes_for_ba` | condition | | atlas r | `mpAtlas->KeyFramesInMap() > 2` |
| `local_bundle_adjustment` | stateful action | `inertial`、`tracking` Optional、`ba_result` Optional、`ba_problem` out、`mapper` out | atlas w、local_mapper w | inertial 且 IMU 已初始化：`mTinit` 的累加、移動不足的重設、`LocalInertialBA`；否則 `LocalBundleAdjustment` |
| `is_inertial_uninitialized` | condition | `inertial` | atlas r、local_mapper r | `!isImuInitialized() && mbInertial` |
| `initialize_imu` | stateful action | `prior_g`、`prior_a`、`fiba` 常值；`ba_result` Optional、`ba_problem` out、`mapper` out | atlas w、local_mapper w、vocabulary r | `InitializeIMU(priorG, priorA, bFIBA)` |
| `keyframe_culling` | action | | atlas w、local_mapper r | `KeyFrameCulling` |
| `begin_inertial_ba` | action | `stage` 常值、`inertial`、`tracking` Optional | atlas w、local_mapper r | VIBA 的條件與 `SetIniertialBA1/2` |
| `send_to_loop_closing` | action | `loop_keyframe` out | atlas r、local_mapper r | `mpLoopCloser->InsertKeyFrame` |
| `reset_if_requested` | action | | atlas w、local_mapper w | `ResetIfRequested` |

兩個 stateful action 以 class 註冊：`LocalBundleAdjustment`（樹上的名稱 `local_bundle_adjustment`）與 `InitializeImu`（`initialize_imu`、`viba1`、`viba2`）。

`process_new_keyframe` 讀 `vocabulary`；`initialize_imu` 在階段 1、3 也要執行 `ProcessNewKeyFrame`，所以同樣讀 `vocabulary`。`ProcessNewKeyFrame` 與 `ResetIfRequested` 的本體放在 `orb_local_mapping/shared/`。

## ba_solver

```
ba_solver (algorithm class)
  ON(problem: ba/Problem, trigger, QUEUE(2, drop_oldest)) → result: ba/Result
```

`ba/Problem` 的 `kind` 決定要解哪一種：

| kind | 原版 | 頂點 | 邊 |
|---|---|---|---|
| `local` | `LocalBundleAdjustment` | 局部 keyframe 的位姿（固定的另列）、map point | KB8 雙目的重投影（左、右眼各一條）、Huber |
| `local_inertial` | `LocalInertialBA` | 時間窗內 keyframe 的位姿、速度、bias；固定的上一個；covisible 的位姿固定；map point | 重投影、IMU 預積分、gyro／acc random walk；`bLarge`、`bRecInit` |
| `inertial_init` | `InertialOptimization(Rwg, scale, bg, ba, …, priorG, priorA)` | 每個 keyframe 的速度、共用的 bias、`Rwg`、scale；位姿固定 | IMU 預積分、bias 先驗 |
| `full_inertial` | `FullInertialBA(map, 100, …, priorG, priorA)` | 整張地圖的 keyframe 狀態與 map point | 重投影、IMU、bias 先驗（priorA ≠ 0） |

`ba/Result` 依 id 列出新的狀態（位姿、速度、bias、點的位置），以及要刪除的觀測（outlier 的 `(keyframe, point)`，原版在寫回時 `EraseMapPointMatch`、`EraseObservation`）；`inertial_init` 另有 `Rwg`、scale、bias。

**解法器**：這些是大型稀疏問題（local BA 約 20 個 keyframe、數千點；full inertial BA 是整張地圖）。依[原則](../../orb_tracking/docs/DESIGN.md#原則)不引入 g2o，自己寫 Levenberg-Marquardt 加 Schur complement（先消去點），照 g2o 的停止條件與阻尼更新；inertial 的邊與 tracking 的 `pose_inertial_optimizer` 同一套數學。見[待決定](#待決定)。

## 介面的變更（tracking 端）

| 變更 | 原因 |
|---|---|
| `tracking/MapperStatus` 多一個 `tracking_ok_seq`（已加入型別） | `InitializeIMU` 結尾的 `mpTracker->mState = Tracking::OK` |
| `apply_mapper_update` 多一個 `state` inout 埠，遇到新的 `tracking_ok_seq` 時設 `state = OK`（之後做） | 同上；只有它在 tracking 的樹上讀 `mapper` 並能改 `state` |

`UpdateFrameIMU` 原版呼叫兩次（先以 `mScale`，再以 1.0）。第二次 scale 為 1，只更新 bias 與上一影格的狀態；`imu_update_seq` 送出的是兩次合併後的結果（scale、`vpKF[0]` 的 bias、目前的 keyframe），tracking 的 `apply_mapper_update` 已照這樣處理。

## 程式碼的配置

依 tracking 設計的[原則](../../orb_tracking/docs/DESIGN.md#原則)放置。使用者增加而要往上搬的：

| 程式 | 從 | 到 | 新的使用者 |
|---|---|---|---|
| `imuInitialized`、`inertialBA2`（`tracking_ops`） | `orb_tracking/shared/` | `types/map/atlas_ops` | 好幾個 LM 葉節點 |
| inertial 的邊（預積分殘差與 Jacobian） | `orb_tracking/track_local_map/pose_inertial_optimizer` | `types/imu/` | `ba_solver` |

### 配置與行數（估計）

```
types/
  ba/problem.hpp                 ba/Problem、ba/Result：複製出來的狀態、觀測、結果         ~200   （空殼已建）
  local_mapping/local_mapper.hpp local_mapping/LocalMapper（資源）                       ~150   （空殼已建）
  tracking/mapper_status.hpp     + tracking_ok_seq                                     +2     （已加）
  map/atlas_ops                  + ApplyScaledRotation、KeyFramesInMap、imuInitialized 等  ~80
  imu/inertial_residual.hpp      預積分殘差與 Jacobian（從 pose_inertial_optimizer 搬來）  ~250
algorithms/
  orb_local_mapping/shared/
    process_keyframe.hpp/.cpp    ProcessNewKeyFrame：process_new_keyframe、initialize_imu  ~80
    mapper_ops.hpp/.cpp          ResetIfRequested、狀態訊息                              ~80
  orb_local_mapping/<葉節點>/
    create_new_map_points/       + SearchForTriangulation、epipolar 檢查、三角化          ~400
    search_in_neighbors/         + Fuse（keyframe ← 點、點 → keyframe）                  ~300
    local_bundle_adjustment/     + 局部視窗與固定 keyframe 的選取、問題的建立、寫回          ~400
    initialize_imu/              + 三個階段、spanning tree 的修正傳遞                     ~450
    keyframe_culling/                                                                 ~150
    其餘約 15 個                  每個約 20–60 行                                        ~600
  orb_local_mapping/docs/DESIGN.md
  ba_solver/
    ba_solver.cpp                block                                                 ~80    （空殼已建）
    sparse_lm.hpp/.cpp           LM + Schur complement（點先消去）                         ~600
    edges.hpp/.cpp               KB8 雙目重投影、IMU、random walk、先驗                    ~600
    inertial_init.hpp/.cpp       InertialOptimization（Rwg、scale）                       ~300
graphs/slam_job.graph.yaml
graphs/trees/local_mapping/*.tree.yaml
                                                                         合計約 4,500–5,000 行
```

## 與原版一致的驗證

reference 刪除之前要以原版產生的標準答案（`reference_orb_source/fixture_gen/`，寫到 `tests/fixtures/orb_slam3/`），沿用 tracking 的合成場景（KB8 雙目、解析軌跡與 IMU）：

| 對象 | 內容 |
|---|---|
| `LocalBundleAdjustment` | 局部 keyframe、固定 keyframe、點與觀測、初始值 → 位姿、點、刪除的觀測 |
| `LocalInertialBA` | 上述加速度、bias、預積分；`bLarge`、`bRecInit` 兩種 → 狀態、刪除的觀測 |
| `InertialOptimization` | keyframe 的位姿與預積分、`Rwg`、scale、bias 的初值、priorG、priorA → `Rwg`、scale、bias、速度 |
| `FullInertialBA` | 整張小地圖 → 狀態、點 |
| `CreateNewMapPoints` 的三角化 | **已產生**（`new_map_points/`）：六個 keyframe 的小地圖 → covisibility 的順序、每個鄰居的 `SearchForTriangulation`、新點（keyframe、索引、位置） |
| `Fuse` | keyframe、投影的點 → 加入或替換的結果 |

`MapPointCulling`、`KeyFrameCulling`、`ProcessNewKeyFrame` 是簿記，以 `types_atlas_test` 那種合成 keyframe 做單元測試，不需要原版的答案。整條軌跡直接以 bag 的 ground truth 算 ATE。

## 待決定

| 事項 | 選項 | 建議 |
|---|---|---|
| ~~BoW 索引（KeyFrameDatabase）由誰加入~~ | **已定 (a)**：照原版，由 loop closing 加入；local mapping 不加 | loop closing 做好之前 relocalization 沒有候選 |
| ~~`ba_solver` 的解法器~~ | **已定 (a)**：自己寫 LM + Schur，做成通用的 static 函式庫（只依賴 Eigen，不依賴 fw），之後光達 BA 共用 | 函式庫的位置與介面見下 |
| ~~`Repeat(-1)` 是否在同一個 tick 內迴圈~~ | 已實測：每輪讓出 tick，fw 自行補 tick，佇列照樣處理完 | 不需改設計 |
| ~~同步部分讓 tracking 等多久~~ | **已定：不拆**。tracking 是 `versioned` 的讀取者，不等寫入鎖；`process_new_keyframe` 的結果在它回傳時就發布 | 只有之後 loop closing 也寫 `atlas` 時，它要等整個 tick；到時再量 |

**非線性最小平方的函式庫（待定位置，建議 `libs/nls/`）**：純數學，只依賴 Eigen，不依賴 fw，建成 STATIC 函式庫連進 blocks 並有自己的單元測試。視覺與光達的 BA 共用，所以不照「最小的資料夾」放在 `ba_solver/` 下。

- 函式庫：參數區塊（含流形：歐式、SO3、SE3）、殘差區塊（解析 Jacobian）、robust kernel（Huber）、固定區塊、標記為可消去的區塊做 Schur complement（沒有可消去的區塊時直接解）、縮減系統用 Eigen 的稀疏 Cholesky、照 g2o 的 LM 阻尼更新與停止條件。
- `ba_solver` 自己的部分：重投影殘差（單目、雙目、KB8）、IMU 預積分殘差、bias random walk、先驗，以及 `ba::Problem` 與函式庫之間的轉換。光達 BA 之後寫自己的殘差，共用同一個解法器。
