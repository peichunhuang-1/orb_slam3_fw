# orb_tracking 設計：以行為樹實作 ORB-SLAM3 的 tracking

這份文件只定架構：圖上怎麼連、資料放在哪裡、樹的邏輯。葉節點與型別的內容（函式本體、class 的成員）在架構確認後再填。

- 樹檔：[`graphs/trees/tracking/`](../../../graphs/trees/tracking/)
- 圖檔：[`graphs/tracking_job.graph.yaml`](../../../graphs/tracking_job.graph.yaml)
- 原始碼的對照：ORB-SLAM3 v1.0 的 `Tracking`（`Track()`，以及 `StereoInitialization`、`TrackReferenceKeyFrame`、`TrackWithMotionModel`、`TrackLocalMap`、`PreintegrateIMU`、`Relocalization` 等）。reference 之後會刪除，本文以 ORB-SLAM3 的函式名稱指稱原始的程式。

## 範圍

| | 這一步 | 之後 |
|---|---|---|
| 感測器 | stereo 與 stereo-inertial，相機皆為 KB8 | |
| tracking | `Track()` 的全部分支，含 relocalization（BoW + keyframe 資料庫 + MLPnP）與 IMU 的預積分、預測、重設 | localization only 模式（`mbOnlyTracking`、`mbVO`）：`track` 子樹多一個分支 |
| local mapping、loop closing | 不做；介面先定好（`keyframe`、`mapper_cmd` 輸出，`mapper` 輸入） | 各自成為一個 block，接上這些埠與 `atlas` 資源 |
| 軌跡 | 逐影格送出 `pose` | 讀 `atlas` 與 `tracker` 的 block，在資料結束時以 `SaveTrajectoryEuRoC` 的方式寫到 `out://` |

沒有 local mapping 時：keyframe 只帶著 `CreateNewKeyFrame` 建立的 stereo 點進入地圖，沒有 local BA 與 culling；IMU 永遠不會完成初始化（初始化由 local mapping 執行），stereo-inertial 會以「IMU 未初始化」的分支執行。這兩點都與 ORB-SLAM3 把 local mapping 關掉時的行為相同。

## 圖

```
 ┌──────────────────────────────── box main ──────────────────────────────────────────────┐
 │                                                                                        │
 │ euroc_mcap_driver                                                                      │
 │   left  ──► orb_extractor_left  ── features ──►┐                                       │
 │   right ──► orb_extractor_right ── features ──►┤ orb_matcher                           │
 │   calib ──────────────────────────────────────►┘   │ stereo (orb/StereoFrame)          │
 │     │                                              ▼                                   │
 │     ├──────────────────────────────────► imu_preintegration ◄── imu                    │
 │     │                                              │ frame_imu (orb/StereoFrame + imu) │
 │     │                                              ▼                                   │
 │     │   ┌─────────────────────── tracking (behavior block) ──────────────────────┐    │
 │     └──►│ calib (latest)     frame (trigger)                                      │    │
 │         │ params (latest) ◄── config://orb_tracking.yaml                          │    │
 │         │ mapper (latest) ◄── local_mapping.status                    （之後）    │    │
 │         │                                                                         │    │
 │         │ ──► pose   tracking   keyframe ──► local_mapping（之後）   mapper_cmd ──►│    │
 │         └─────────────────────────────────────────────────────────────────────────┘    │
 │                 ▲ write              ▲ write               ▲ read                      │
 │               atlas               tracker              vocabulary        （資源）      │
 │                 └──► local_mapping、loop_closing（之後）                               │
 └────────────────────────────────────────────────────────────────────────────────────────┘
```

- **tracking** 是 behavior block，樹是 [`tracking.tree.yaml`](../../../graphs/trees/tracking/tracking.tree.yaml)。只有一個 tick 的來源：`frame`（`trigger`），每個 stereo 影格 tick 一次，對應 `GrabStereoFeatures` → `Track()`。`calib`、`params`、`mapper` 是 `latest`。
- **感測器由 `params.sensor` 決定**（`stereo` 或 `stereo_inertial`）。`make_frame` 依它寫入鍵 `inertial`，並檢查 `frame.imu` 是否存在：stereo-inertial 卻沒有預積分時回報錯誤。只做 stereo 時，圖上把 `orb_matcher.stereo` 直接接到 `tracking.frame`，拿掉 `imu_preintegration`。
- **`orb_matcher` 多一個輸出 `stereo`**（`orb/StereoFrame`）：兩眼的 keypoint、描述子、`scale_factors` 與配對結果（`orb::Match` 的內容），**不含金字塔**。`Frame` 需要左右兩眼的特徵（KB8 stereo 的 `N = Nleft + Nright`），由 matcher 一次送出，tracking 就不必 sync 三路資料，也不會因為排隊而持有 extractor 的 GPU 金字塔 buffer。原本的 `match` 輸出保留。
- **`imu_preintegration`**（新的 algorithm，class 形式）。詳見[IMU 預積分](#imu-預積分)。
- **輸出**：
  - `pose`：每個追蹤成功的影格送出 body 在 world 中的位姿（由 `Tcw` 與 `calib.T_body_left` 換算），與 `groundtruth` 同一種，可以直接比較。
  - `tracking`：每個影格一筆，含狀態、inlier 數、地圖與 keyframe 數。
  - `keyframe`：每建立一個 keyframe 送出一筆，交給 local mapping（取代 `InsertKeyFrame`）。
  - `mapper_cmd`：要求 local mapping 重設地圖或中斷 BA。
  - behavior block 另有內建的 `status`。

## IMU 預積分

ORB-SLAM3 在 tracking 的執行緒內做 `PreintegrateIMU`：從佇列取出上一影格到這一影格之間的樣本，在兩端內插，然後積分兩次。一次是以 `mLastFrame` 的 bias 為線性化點、從上一影格起算；另一次累加到「從上一個 keyframe 起算」的預積分。這裡把積分移到上游的 block，tracking 只用 bias 的 Jacobian 修正：

**`imu_preintegration`**：`ON(imu)` 累積樣本；`ON(frame)` 收到 `orb/StereoFrame` 後，等到有一筆時間 ≥ 影格時間 − 1 ms 的樣本（`PreintegrateIMU` 停止選取的那一筆），就做 `PreintegrateIMU` 的端點內插與積分。積分的**線性化點固定在 b̄ = 0**，得到 ΔR、ΔV、ΔP、共變異數與 Jacobian（`JRg`、`JVg`、`JVa`、`JPg`、`JPa`）。結果放進影格的 `imu` 欄位，以 `fw::make_msg(影格, 影格的時間)` 送出，時間戳記與影格相同。

- IMU 雜訊與 `T_b_c1` 取自 `calib`（與 ORB-SLAM3 的 `IMU::Calib` 相同）。
- 第一個影格沒有上一影格，`imu` 留空，對應 `PreintegrateIMU` 的「non prev frame」。
- 影格的時間比上一個早時清掉累積的樣本，從這個影格重新起算，對應 `Track()` 遇到時間倒退時的 `mlQueueImuData.clear()`。
- 資料結束時，還在等待 IMU 的影格以已有的樣本積分後送出。

**tracking 端**：

- 這一影格的預積分：`SetNewBias(目前的 bias)`，經由 Jacobian 一階修正。這本來就是 ORB-SLAM3 的機制（`GetUpdatedDeltaRotation` 等），pose inertial optimization 也是這樣使用預積分。
- 從上一個 keyframe 起算的預積分：把各影格的預積分**串接**起來（ΔR = ΔR₁ΔR₂、ΔV = ΔV₁ + ΔR₁ΔV₂、ΔP = ΔP₁ + ΔV₁Δt₂ + ΔR₁ΔP₂，Jacobian 與共變異數同樣依鏈鎖律合併）。所有片段的線性化點都是 b̄，所以串接是精確的。建立 keyframe 時重新開始。
- **原始樣本保留在預積分裡**（內插後的樣本，每影格約 10 筆）：local mapping 在 IMU 初始化後 gyro bias 變化超過 0.01 時呼叫 `Reintegrate`，KF culling 時呼叫 `MergePrevious`，兩者都需要原始樣本。

**與 ORB-SLAM3 的差異**：只有線性化點不同。ORB-SLAM3 用上一影格或上一個 keyframe 的 bias，這裡用 0；Jacobian 修正忽略的二階項就是兩者的差異。

- IMU 初始化之前 bias 為 0，結果完全相同。
- 初始化之後，一個影格約 50 ms、gyro bias 約 0.02 rad/s 時，誤差約 (b·Δt)² ≈ 1e-6 rad；keyframe 區間約 0.5 s 時約 1e-4。bias 變化大時，local mapping 本來就會 `Reintegrate`。
- 結果因此不會逐位元相同；預積分的標準答案同時記錄了「在 0 積分再修正」與「在 bias 重新積分」兩種結果，單元測試可直接量這個差異。

**bias 回饋（之後再決定）**：把 tracking 的 bias 回傳給 `imu_preintegration` 當作線性化點，可以讓結果更接近 ORB-SLAM3。這會形成 `tracking → imu_preintegration → tracking` 的環；tracking 只有一個 `trigger` 輸入，回饋的埠用 `latest` 加 `fw::Optional`，就符合[環的規則](../../../docs/concepts/feedback.md)（若 tracking 以 `sync` 接收就違反 R3）。代價是線性化點會因影格而異，串接前要先把各段修正到同一個 bias，不再精確。先不接，量過差異再決定。

## 資料放在哪裡

`Tracking` 的狀態分成四類，各放在最適合的位置：

| 放在 | 內容 | 理由 |
|---|---|---|
| **埠**（不可變的訊息） | stereo 影格與它的預積分、calib、params、位姿、狀態、keyframe 事件 | 單向流動、產生後不再修改 |
| **資源 `atlas`**（`map/Atlas`，write） | 所有地圖、keyframe、map point、相機模型；加上 BoW 反向索引（ORB-SLAM3 的 `KeyFrameDatabase`） | 之後 local mapping、loop closing 都要修改它，必須是共享資源。KeyFrameDatabase 跟著 keyframe 一起新增刪除，放在同一個鎖之下 |
| **資源 `tracker`**（`tracking/Tracker`，write） | `Tracking` 除了 `mState` 以外的成員：`mCurrentFrame`、`mLastFrame`、速度模型、reference keyframe 與上一個 keyframe 的 id、local map（keyframe id 與 point id 的清單）、temporal points、IMU 的 calib／從上一個 keyframe 起算的預積分／`mLastBias`、`mnLastRelocFrameId`、`mTimeStampLost`、相機與 `mTlr`、軌跡清單（`mlRelativeFramePoses` 等）、待執行的重設要求 | 樹的每個葉節點是獨立的物件，彼此之間共用狀態只能經由鍵或資源。這些成員很大（兩個 `Frame`），且每次都要和 `atlas` 一起讀寫，放在鍵上會一直複製 |
| **資源 `vocabulary`**（`map/Vocabulary`，read） | ORB 詞彙樹（與 DBoW2 的 ORBvoc 同一份資料） | 很大、唯讀、之後 loop closing 也要用。`store:` 在啟動時載入；`fw_save` 不做事，不會寫回 |
| **鍵**（blackboard） | `state`（`tracking/State`）、`inertial`、`ok`（`bOK`，只在 `track` 子樹內） | 決定樹的分支，樹檔中看得到，執行時在樹分頁的 Keys 面板上可以觀察 |

**鎖的範圍**：behavior block 每個 tick 取一次它用到的資源，整個 tick 持有 `atlas` 與 `tracker` 的寫入鎖，相當於 ORB-SLAM3 在 `Track()` 中持有 `mMutexMapUpdate`。之後的 local BA 與 loop closing 依[鎖外計算](../../../docs/concepts/resources.md#長時間運算鎖外計算)的方式，只在複製與寫回時短暫持有 `atlas`，與 ORB-SLAM3 相同。

**以 id 引用，不用指標**：ORB-SLAM3 的物件以指標互相引用（`Frame::mvpMapPoints`、`KeyFrame` 的 covisibility、`MapPoint` 的觀測）。這裡一律改用 id：`tracker`、`Frame`、`KeyFrame`、`MapPoint` 之間只存 keyframe id 與 point id，使用時向 `atlas` 查。因此 `tracker` 不保留任何指向資源內部的指標，符合 fw 的一般規則；重設或刪除後查不到的 id 直接略過，與 ORB-SLAM3 的 `isBad()` 檢查同義。keyframe 與 map point 存放在以 id 為鍵的 `fw::TiledMap`（[types/map/docs/DESIGN.md](../../../types/map/docs/DESIGN.md)），每次查詢約 50 ns（5 萬點時）。其他 block 對 keyframe 的引用同樣用 id（`map/KeyFrameRef`）。

**重設延後到下一個 tick**：ORB-SLAM3 的 `System::ResetActiveMap()` 只設定旗標，真正的 `Tracking::ResetActiveMap()` 在下一次 `TrackStereo` 的開頭執行。這裡照樣做：葉節點在 `tracker` 記下重設要求，根樹的第一個葉節點 `apply_pending_reset` 在下一個 tick 執行，同時送出 `mapper_cmd`。`CreateMapInAtlas()` 在 ORB-SLAM3 是立即執行的，這裡也立即執行。

## 樹的邏輯

樹分成五個檔案，結構與 `Track()` 的程式一一對應：

```
tracking                         (根：每個 stereo 影格 tick 一次)
└ Sequence
  ├ apply_pending_reset          System::TrackStereo：執行上一個影格要求的 ResetActiveMap
  ├ make_frame                   GrabStereoFeatures：建 Frame（calib 變了就重建相機，newParameterLoader）；
  │                              寫入 inertial；frame.imu 交給 tracker
  ├ apply_mapper_update          local mapping 的 UpdateFrameIMU（IMU 初始化後的 scale 與 bias）
  └ IfThenElse
    ├ frame_discontinuity?       mbBadImu、時間倒退、跳超過 1 s（inertial）
    ├ handle_discontinuity       → 要求重設或 CreateMapInAtlas，這個影格到此為止
    └ track_frame ─────────────────────────────────────────────────────────┐
                                                                           │
track_frame                                                                ◄┘
└ Sequence
  ├ begin_frame                  NO_IMAGES_YET→NOT_INITIALIZED、SetNewBias（以 Jacobian 修正
  │                              frame.imu）、串接從上一個 keyframe 起算的預積分、地圖變更索引
  ├ IfThenElse
  │ ├ state_is NOT_INITIALIZED?
  │ ├ initialize                 Fallback[ stereo_initialization, remember_last_frame ]
  │ └ track ───────────────────────────────────────────────────────────────┐
  └ publish_frame                軌跡清單（OK、RECENTLY_LOST），送出 pose、tracking │
                                                                           │
track                                                                      ◄┘
└ IfThenElse
  ├ state_is LOST?               （影格開始時就是 LOST）
  ├ start_new_map                「A new map is started」：<10 keyframe 重設，否則新地圖
  └ Sequence
    ├ Fallback  (tracked)        ── bOK ──
    │ ├ Sequence
    │ │ ├ estimate_pose ─────────────────────────────────────────────────┐
    │ │ ├ track_local_map        TrackLocalMap                           │
    │ │ └ tracking_ok            state = OK, ok = true                   │
    │ └ track_failed             ok = false；OK→RECENTLY_LOST（inertial 的重設規則）
    ├ imu_bias_bookkeeping       reloc + N 時 ResetFrameIMU；mLastBias    │
    ├ IfThenElse                                                         │
    │ ├ tracked_or_recently_lost?   bOK || RECENTLY_LOST                 │
    │ ├ Sequence                                                         │
    │ │ ├ update_motion_model    mVelocity                               │
    │ │ ├ clean_vo_matches       沒有觀測的點、temporal points            │
    │ │ ├ Fallback[ Sequence[ need_new_keyframe, create_new_keyframe ], AlwaysSuccess ]
    │ │ └ discard_outliers                                               │
    │ └ AlwaysSuccess                                                    │
    └ IfThenElse                                                         │
      ├ state_is LOST?           （這個影格變成 LOST）                    │
      ├ handle_lost              ≤10 keyframe 或 IMU 未初始化就重設，否則新地圖
      └ finish_frame             mpReferenceKF、mLastFrame = current     │
                                                                         │
estimate_pose        （SUCCESS = 初始位姿估計的 bOK）                     ◄┘
└ IfThenElse
  ├ state_is OK?
  ├ Fallback
  │ ├ Sequence
  │ │ ├ check_replaced_in_last_frame
  │ │ └ IfThenElse
  │ │   ├ use_reference_keyframe?    沒有速度且 IMU 未初始化，或剛重定位
  │ │   ├ track_reference_keyframe   （BoW）
  │ │   └ Fallback[ track_with_motion_model, track_reference_keyframe ]
  │ └ ForceFailure( lost_after_pose_failure )   OK→RECENTLY_LOST（>10 keyframe）或 LOST
  └ IfThenElse                   （RECENTLY_LOST）
    ├ is_true inertial?
    ├ Sequence[ lost_time_within 5 s, predict_state_imu ]      逾時：state = LOST
    └ Fallback[ relocalize, ForceFailure( lost_time_within 3 s ) ]
```

閱讀時的幾個要點：

- **`state` 只存在鍵上**。`Tracker` 沒有 `mState`；需要讀或改狀態的葉節點都以 `state` 埠接到同一個鍵（改的一方用 `fw::InOut`）。`mLastProcessedState` 由 `begin_frame` 記在 `tracker`。
- **`return` 的對應**：`Track()` 中途的 `return`，在樹上是 `IfThenElse` 的另一個分支，例如 `handle_discontinuity`、`start_new_map`、`handle_lost`，而不是讓整棵樹 FAILURE。根節點每個影格都正常結束，`status` 的 `result` 只表示這次 tick 跑完了，追蹤結果看 `tracking` 輸出。
- **`bOK` 的兩種表示**：在 `tracked` 之內以節點的成功或失敗表示，`tracked` 結束時由 `tracking_ok`／`track_failed` 寫入鍵 `ok`，後面的步驟（`imu_bias_bookkeeping`、keyframe 的決定）再讀它。
- **兩個 LOST 檢查**：`track` 開頭的 `state_is LOST` 是「上一個影格留下的 LOST」，對應 `Track()` 前段的 `else if (mState == LOST)`；結尾的是「這個影格才變成 LOST」，對應最後的 `if (mState == LOST)`。兩者的重設條件不同（`<10` 與 `≤10`、IMU），所以是兩個葉節點。
- **同一種葉節點出現多次**：`track_reference_keyframe`、`lost_time_within`、`state_is` 都出現不只一次，各是獨立的物件。它們不保存自己的狀態，狀態都在 `tracker` 與鍵上，所以沒有影響。
- **沒有 local mapping 時**：讀 `mapper` 的葉節點都以 `fw::Optional` 接收；沒有值時視為一個閒置、接受 keyframe、尚未完成 IMU 初始化的 local mapper。

## 葉節點

全部是函式形式的 condition 或 action，不需要 stateful action，因為每個步驟都在一個 tick 內完成，ORB-SLAM3 的 tracking 本來就是同步的。`atlas`、`tracker`、`vocabulary` 在樹檔中不寫，依同名規則接到樹的資源。

| 葉節點 | 種類 | 埠（鍵） | 資源 | 對應的 ORB-SLAM3 |
|---|---|---|---|---|
| `apply_pending_reset` | action | `state` inout、`mapper_cmd` out | atlas w、tracker w | `System::TrackStereo` 的 `mbReset`／`mbResetActiveMap` → `Tracking::Reset`／`ResetActiveMap` |
| `make_frame` | action | `frame`、`calib`、`params`、`inertial` out | atlas w（`AddCamera`）、tracker w | `newParameterLoader`、`GrabStereoFeatures`（`Frame` 的建構）；`frame.imu` 交給 tracker |
| `apply_mapper_update` | action | `mapper` Optional | atlas r、tracker w | `UpdateFrameIMU`（由 local mapping 呼叫） |
| `frame_discontinuity` | condition | `state`、`mapper` Optional | atlas r、tracker r | `Track()` 開頭的 `mbBadImu` 與時間戳記檢查 |
| `handle_discontinuity` | action | `state` inout、`mapper` Optional | atlas w、tracker w | 同上的處置：`ResetActiveMap` 要求、`CreateMapInAtlas` |
| `begin_frame` | action | `state` inout | atlas w（`SetLastMapChange`）、tracker w | `SetNewBias`、`PreintegrateIMU` 的後半（這裡改為修正與串接）、`mbCreatedMap`、`GetMapChangeIndex` |
| `state_is` | condition | `state`、`value` 常值 | | `mState == …` |
| `is_true` | condition | `value` | | 分支條件（`inertial`） |
| `stereo_initialization` | action | `state` inout、`keyframe` out | atlas w、tracker w | `StereoInitialization` |
| `remember_last_frame` | action | | tracker w | 初始化失敗時的 `mLastFrame = Frame(mCurrentFrame)` |
| `start_new_map` | action | `state` inout | atlas w、tracker w | `Track()` 的 `else if (mState == LOST)` 分支 |
| `check_replaced_in_last_frame` | action | | atlas r、tracker w | `CheckReplacedInLastFrame` |
| `use_reference_keyframe` | condition | | atlas r、tracker r | `(!mbVelocity && !isImuInitialized) \|\| id < reloc + 2` |
| `track_reference_keyframe` | action | | atlas w、tracker w、vocabulary r | `TrackReferenceKeyFrame`（`ComputeBoW`、`SearchByBoW`、`PoseOptimization`） |
| `track_with_motion_model` | action | | atlas w、tracker w | `UpdateLastFrame`、`TrackWithMotionModel` |
| `lost_after_pose_failure` | action | `state` inout | atlas r、tracker w | `if (!bOK)` 之後的 RECENTLY_LOST／LOST 判定 |
| `lost_time_within` | action | `state` inout、`max_s` 常值 | tracker r | `mTimeStamp - mTimeStampLost > …` → LOST |
| `predict_state_imu` | action | | atlas r、tracker w | `PredictStateIMU`（IMU 未初始化：FAILURE） |
| `relocalize` | action | | atlas w、tracker w、vocabulary r | `Relocalization`（KeyFrameDatabase、MLPnP） |
| `track_local_map` | action | `state`、`mapper` Optional | atlas w、tracker w | `TrackLocalMap`（`UpdateLocalMap`、`SearchLocalPoints`、pose (inertial) optimization） |
| `tracking_ok` | action | `state` inout、`ok` out | | `bOK` → `mState = OK` |
| `track_failed` | action | `state` inout、`ok` out | atlas r、tracker w | `else if (mState == OK)` 的 RECENTLY_LOST 與 inertial 重設要求 |
| `imu_bias_bookkeeping` | action | `ok` | atlas r、tracker w | `ResetFrameIMU`、`mLastBias` |
| `tracked_or_recently_lost` | condition | `ok`、`state` | | `bOK \|\| mState == RECENTLY_LOST` |
| `update_motion_model` | action | | tracker w | `mVelocity` |
| `clean_vo_matches` | action | | atlas r、tracker w | Clean VO matches、Delete temporal MapPoints |
| `need_new_keyframe` | action | `ok`、`state`、`params`、`mapper` Optional、`mapper_cmd` out | atlas r、tracker r | `NeedNewKeyFrame` 加上插入的條件；`InterruptBA` 改為送出 `mapper_cmd` |
| `create_new_keyframe` | action | `mapper` Optional、`keyframe` out | atlas w、tracker w | `CreateNewKeyFrame`；`InsertKeyFrame` 改為送出 `keyframe` |
| `discard_outliers` | action | | tracker w | 丟掉 outlier 的 map point |
| `handle_lost` | action | `state` inout | atlas w、tracker w | `Track()` 結尾的 `if (mState == LOST)` |
| `finish_frame` | action | | tracker w | `mpReferenceKF`、`mLastFrame = Frame(mCurrentFrame)` |
| `publish_frame` | action | `state`、`pose` Optional out、`tracking` out | atlas r、tracker w | 軌跡清單；`pose` 為 body 在 world 中的位姿 |

`need_new_keyframe` 會送出 `mapper_cmd`（中斷 BA），condition 不能有輸出，所以它是 action；回傳 SUCCESS 表示要插入 keyframe。

## 程式碼的配置

### 原則

ORB-SLAM3 的程式不整包移植。邏輯與數學與原版一致，實作重寫，依下列規則放置：

**程式碼放在涵蓋它所有使用者的最小資料夾：**

| 誰使用 | 放在哪裡 | 例子 |
|---|---|---|
| 只有一個葉節點 | 該葉節點自己的資料夾 | MLPnP 只有 `relocalize` 用：`orb_tracking/relocalize/` |
| tracking 的好幾個葉節點 | `algorithms/orb_tracking/shared/` | `PoseOptimization`：4 個葉節點使用 |
| 好幾個 block（tracking、matcher，之後的 local mapping、loop closing） | `types/<領域>/`，和它操作的資料放在一起 | KB8 投影：matcher 與 tracking |

使用者增加時往上搬一層，例如 local mapping 也要用某段 tracking 的程式時，把它從 `orb_tracking/shared/` 搬到 `types/`。

**型別只放資料，加上維持資料一致的操作**（例如加觀測時同步更新 covisibility）：

- 不放 mutex：ORB-SLAM3 在每個 `KeyFrame`、`MapPoint` 上的鎖，由 fw 資源的鎖取代。
- 不放序列化：存檔之後以 `fw_save`／`fw_load` 實作，不需要 boost。
- 以 id 互相引用，不用指標（[資料放在哪裡](#資料放在哪裡)）。
- 查詢的邏輯放在使用它的人那裡。例如 BoW 反向索引在 `types/map/`，relocalization 的候選查詢（`DetectRelocalizationCandidates`）在 `relocalize/`，之後 loop closing 的查詢在 loop closing 裡。

**不引入第三方函式庫**（g2o、Sophus、DBoW2 都不用）：

| ORB-SLAM3 使用 | 這裡 | 理由 |
|---|---|---|
| g2o | Eigen 寫的 Gauss-Newton／LM | tracking 的最佳化都是小型稠密問題（純位姿 6 維、pose-inertial 15 或 30 維） |
| Sophus | Eigen，加上 `types/geometry/so3.hpp`（exp、log、右 Jacobian） | 只用到這些 |
| DBoW2 | `types/map/vocabulary`：讀詞彙樹、transform（BowVector、FeatureVector）、L1 分數 | tracking 只用到這些 |

之後的 local BA 與 global BA 是大型稀疏問題，到時候再決定自己寫 Schur complement + LM，或改用 Ceres（apt）。

### 配置與行數（估計）

```
types/
  geometry/so3.hpp               SO3 exp/log/右 Jacobian                                       ~120
  geometry/se3.hpp               剛體變換：四元數加平移，運算與 Sophus 相同                        ~100
  camera/                        KB8、StereoRig、rig_from_calib：從 orb_matcher 搬來（namespace camera） 既有
  orb/hamming.hpp                從 orb_matcher/core 搬來（orb::hamming256）                       既有
  orb/stereo_frame.hpp           orb_matcher 的新輸出                                            ~60
  orb/feature_grid.hpp           64×48 的 keypoint 格子與視窗搜尋（GetFeaturesInArea）             ~70
  orb/frame_features.hpp         Frame 與它產生的 keyframe 共用的特徵（不複製）、尺度層級            ~70
  imu/preintegrated.hpp/.cpp     積分、Jacobian 修正、串接、Reintegrate                           ~350
  map/ids.hpp keyframe.hpp map_point.hpp map.hpp atlas.hpp keyframe_ref.hpp
                                 純資料，以 id 互相引用；keyframe 與 map point 用 TiledMap         ~400
  map/atlas_ops.hpp/.cpp         加觀測、covisibility、spanning tree、bad、Replace、法向與深度、
                                 代表描述子、建立 stereo 點                                         ~450
  map/bow_index.hpp/.cpp         keyframe 的 BoW 反向索引（只有索引）                              ~70
  map/vocabulary.hpp/.cpp        詞彙樹、fw_load、transform、score                                ~350
  tracking/frame.hpp/.cpp        Frame：特徵、point id、outlier、位姿、IMU                         ~180
  tracking/tracker.hpp state.hpp status.hpp mapper_status.hpp mapper_command.hpp               ~200
algorithms/
  orb_tracking/shared/
    pose_optimizer.hpp/.cpp      PoseOptimization：Huber、4 輪重新判定 outlier                    ~250
    search_by_bow.hpp/.cpp       SearchByBoW(KF, Frame)：track_reference_keyframe、relocalize      ~150
    rotation_histogram.hpp       ComputeThreeMaxima：三個搜尋共用                                  ~50
  orb_tracking/<葉節點>/          一個葉節點一個資料夾，只有它用的東西放在裡面：
    track_with_motion_model/     + 投影搜尋（Frame → Frame）                                      ~250
    track_local_map/             + local map 更新、投影搜尋（點 → Frame）、pose-inertial 最佳化     ~800
    relocalize/                  + 候選查詢、MLPnP、投影搜尋（KF → Frame）                          ~900
    其餘約 28 個                  每個約 30–80 行                                                  ~1,500
  orb_tracking/docs/DESIGN.md
  imu_preintegration/            algorithm block                                                ~200
  orb_matcher/                   多一個 stereo 輸出                                               ~50
params/orb_tracking_params.hpp   config/orb_tracking.yaml
tools/orb_vocabulary/            ORBvoc.txt → .bin                                              ~150
tests/fixtures/orb_slam3/        ORB-SLAM3 產生的標準答案（見下）
data/orb_vocabulary/
  ORBvoc.bin.tar.gz              進 git
  ORBvoc.bin                     解壓後的檔案，不進 git（.gitignore）；建置時若不存在就從壓縮檔解開
graphs/tracking_job.graph.yaml
graphs/trees/tracking/*.tree.yaml
                                                                                 合計約 6,000–6,500 行
```

### 型別的實作細節

- **順序以 id 取代指標**：ORB-SLAM3 有幾處以指標為鍵的容器，走訪順序就是指標位址的順序，例如 `MapPoint::mObservations`、`mConnectedKeyFrameWeights`、`Map` 的集合。這裡都改以 id 為鍵，結果因此是決定性的。順序會影響結果的地方有：代表描述子同分時取哪一個、covisibility 同權重時的先後、`UpdateConnections` 選哪個 keyframe 當 parent，以及 `GetAllMapPoints` 的順序。
- **TiledMap**：keyframe 與 map point 存在以 id 為鍵的 `fw::TiledMap`，`std::map`、`std::set` 改用排序過的 vector（`types/map/flat.hpp`），走訪順序同樣依 id（[types/map/docs/DESIGN.md](../../../types/map/docs/DESIGN.md)）。
  - 變成 bad 之後物件仍然保留，與 ORB-SLAM3 相同，因為 `GetReplaced` 與軌跡的 reference keyframe 還會用到。
  - `ResetActiveMap` 之後，該地圖的 keyframe 也保留，只是不再屬於任何地圖，同樣與原版一致。
  - 只有 `Reset`（`clearAtlas`）會清空陣列，keyframe 的 id 從 0 重新編號（原版 `KeyFrame::nNextId = 0`）。map point 的 id 也跟著從 0 重新編號；原版不重設，但沒有邏輯依賴它。
- **只支援 KB8 雙目**：每個 keyframe 都有兩台相機，所以每個觀測讓 `nObs` 加 1，不使用 `mvuRight` 的分支。
- **查詢用的暫存不放在型別裡**：ORB-SLAM3 放在 `KeyFrame`、`MapPoint` 上的查詢暫存，例如 `mnRelocQuery`、`mnTrackReferenceForFrame`、`mTrackProjX`、`mbTrackInView`，由使用的葉節點自己保存。
- **詞彙檔**：`data/orb_vocabulary/ORBvoc.bin.tar.gz` 由 `tools/orb_vocabulary` 從 ORBvoc.txt 轉出，與 reference 的 `ORBvoc.bin` 逐位元組相同。建置時若 `ORBvoc.bin` 不存在，就從壓縮檔解開。

### shared 的實作細節

- **`pose_optimizer`**：把 g2o 的 Levenberg-Marquardt 照原樣寫出來，包括停止條件、`exp(δ)·T` 的左乘更新、LDLT，以及 Huber 的 δ² 存成 float。原版有幾個容易漏掉的行為也照做：
  - 每一輪都從進入時的位姿重新開始：原版每輪從 Frame 重設頂點，而 Frame 的位姿到最後才寫回。
  - 判定 outlier 用的 chi2 是最後一次算出的誤差。步驟被拒絕時，g2o 留下的是被拒絕那一步的誤差。
  - 與 pose_optimization 的 8 組標準答案比對：inlier 數與每個 outlier 判定都相同，位姿差在 float 的捨入以內。
- **KB8 投影**：`camera::KannalaBrandt8` 多了 `project_orb`、`project_jacobian_orb`，照原版的寫法實作（double 的點也以 `atan2f`、`sqrtf` 算 θ、ψ）。原本的 `project` 不變，matcher 繼續使用。
- **rotation histogram**：原版的 bin 是 `round(rot / 30)`（把 `HISTO_LENGTH` 當成角度寬度，而不是格數），所以只用到 0..12 格。這會影響保留哪些配對，照原樣實作。
- **SearchByBoW**：照原版的兩個特性實作：
  - 右眼只有在左眼的最佳距離 ≤ TH_LOW 時才比對，不論左眼的比例測試是否通過。
  - 右眼沒有比例測試，原版的條件是 `|| true`。
- **pose-inertial 最佳化**（`track_local_map/pose_inertial_optimizer`）：把 g2o 的 Gauss-Newton（每輪 10 次，沒有停止條件）照原樣寫出。原版這些行為也照做：
  - 每一輪判定 outlier 用的誤差，是那一輪最後一次更新**之前**算的；
  - 留在 Frame 上的先驗用最後的線性化點算 H。原版的對稱化寫成 `(H + H) / 2`，等於沒有對稱化，這裡照寫，再把 1e-12 以下的特徵值設為 0；
  - LastFrame 版本把上一影格邊緣化：SVD 偽逆，門檻 1e-6。

  與 pose_inertial 的 12 組標準答案比對：inlier 數與 outlier 判定完全相同，狀態差在 1e-6 以內，先驗 H 的相對差約 1e-8。另外以本設計的作法（b̄ = 0 預積分，再以 Jacobian 修正到 keyframe 的 bias）重跑 LastKeyFrame：inlier 數相同，位置差不超過 3e-6 m。
- **MLPnP**（`relocalize/mlpnp_solver`）：
  - 原版的 GN Jacobian 是自動產生的展開式，這裡改用同一個導數的解析形式 `−R[X]×Jr(w)`。`computePose` 與標準答案的差在 1e-15。
  - 原版的 `Refine` 雖然用最佳的 inlier 重解，結果卻沒有使用，檢查並回傳的仍是這一輪的假設。這裡照這個行為，直接重新檢查這一輪的假設。
  - RANSAC 的取樣用 solver 自己的亂數產生器（固定種子），與原版的取樣不同，所以兩者都和真值比：位姿誤差與真正 inlier 的比例相當。
- **KB8 反投影**：`camera::KannalaBrandt8::unproject_orb` 照原版寫：z = 1 的射線，float 的牛頓法，與標準答案的 bearing 相同。
- **`mnFramesToResetIMU`**：reference 的版本沒有初始化這個成員。ORB-SLAM3 v1.0 設為 `mMaxFrames`，這裡也取 `params.fps`。

### 葉節點的實作細節

- **`inertial` 也記在 `tracker`**：`make_frame` 依 `params.sensor` 寫入鍵 `inertial`（樹的分支用）與 `tracker.inertial`（葉節點用，相當於 `mSensor == IMU_STEREO`），葉節點不必都接 `inertial` 埠。`mMaxFrames`、`mnFramesToResetIMU` 同樣由 `make_frame` 從 `params.fps` 寫入 `tracker`。
- **map point 的投影暫存**（`Tracker::point_scratch`，以 point id 為索引）：原版放在 `MapPoint` 上的 `mbTrackInView(R)`、`mTrackProjX(R)`、`mnTrackScaleLevel(R)`、`mTrackViewCos(R)`、`mTrackDepth`、`mnLastFrameSeen`。這些值跨影格保留，與原版相同，因為有兩處會讀到舊值：
  - 位姿估計丟掉的 outlier 標為這一影格已見過，local map 不再投影它，但它的 view 旗標只清掉 outlier 那一眼（`TrackReferenceKeyFrame` 另外清左眼），另一眼留著上一次投影的結果，搜尋時照用；
  - pose-inertial 最佳化讀 `mTrackDepth`，只有左眼在視野內時才更新。
- **只在一個影格內有效的標記**（keyframe 與 point 的 `mnTrackReferenceForFrame`）改為葉節點內的集合。relocalization 的 `mnRelocQuery`、`mnRelocWords`、`mRelocScore` 會讀到之前查詢留下的分數，所以放在 `Tracker::reloc`，以 keyframe id 為索引。
- **原版會解參考空指標的地方**改用下一個可行的做法，並在程式中註明：
  - `track_local_map`：沒有上一影格的先驗（`mpcpi`）時用 LastKeyFrame 版本；沒有預積分時用 `PoseOptimization`；
  - `need_new_keyframe`：IMU 初始化前沒有上一個 keyframe 時不插入；
  - `apply_mapper_update`：上一影格沒有 keyframe 時不更新它的狀態。
- **`UpdateFrameIMU`**（`apply_mapper_update`）：原版由 local mapping 的執行緒呼叫，時機不定。這裡在影格開始時套用，相當於落在兩個影格之間；這時新影格尚未預積分，由 `begin_frame` 以新 keyframe 的 bias 處理，所以只更新上一影格。
- **`CreateNewKeyFrame` 的 `SetNotStop`**：local mapping 停止時不建立 keyframe（`create_new_keyframe` 回傳 FAILURE），因此它也讀 `mapper`。
- **沒有 local mapping 時**：tracking 可以初始化，但下一個影格就會 LOST。新 keyframe 的 BoW、觀測與 covisibility 都由 local mapping 的 `ProcessNewKeyFrame` 建立，沒有它，`SearchByBoW` 對 reference keyframe 找不到配對，原版也一樣。以臨時加入的 `ProcessNewKeyFrame`（只做這一步，沒有 local BA，IMU 不初始化）跑 V1_02_medium，從初始化到結束都是 OK，324 個 keyframe，每影格 80–500 個 inlier。

### 與原版一致的驗證

重寫的部分（pose-inertial 最佳化、MLPnP、BoW、預積分）要能證明與 ORB-SLAM3 一致。reference 刪除之後就沒有比對的對象，所以**刪除之前**先以原版產生標準答案，放在 `tests/fixtures/orb_slam3/`：

| 對象 | 內容 |
|---|---|
| 預積分 | IMU 樣本、bias → ΔR、ΔV、ΔP、共變異數、Jacobian；`SetNewBias` 後的值 |
| `PoseOptimization` | Frame 的觀測與初始位姿 → 位姿、outlier 判定、inlier 數 |
| `PoseInertialOptimizationLastFrame／LastKeyFrame` | 兩個狀態、預積分、觀測 → 狀態、bias、inlier 數 |
| MLPnP | 2D-3D 對應 → RANSAC 的位姿與 inlier（取樣不同，容差比對）；最小解 `computePose` 的位姿 |
| BoW | 描述子 → BowVector、FeatureVector；兩個向量的分數 |

資料都來自合成場景（KB8 雙目、解析軌跡與 IMU，加上雜訊與外點），格式與欄位見 `tests/fixtures/orb_slam3/README.md`。各段的新實作以這些資料做單元測試。整條軌跡不跑原版，直接以 bag 的 ground truth 算 ATE。

## 新的型別

| 型別 ID | 位置 | 內容 |
|---|---|---|
| `orb/StereoFrame` | `types/orb/stereo_frame.hpp` | 訊息，`orb_matcher` 送出。兩眼的 keypoint、描述子、`scale_factors`、配對結果；`imu`（`std::shared_ptr<const imu::Preintegrated>`），由 `imu_preintegration` 填入，stereo 時為空 |
| `imu/Preintegrated` | `types/imu/preintegrated.hpp` | 線性化點 b̄、ΔR、ΔV、ΔP、共變異數、Jacobian、內插後的樣本；積分、修正、串接、`Reintegrate` |
| `map/Atlas` | `types/map/atlas.hpp` | 資源。所有地圖、keyframe、map point、相機，與 BoW 反向索引 |
| `map/Vocabulary` | `types/map/vocabulary.hpp` | 資源。ORB 詞彙樹；`fw_load` 讀 `.bin`，`fw_save` 不做事 |
| `map/KeyFrameRef` | `types/map/keyframe_ref.hpp` | 訊息。`map_id`、`keyframe_id` |
| `tracking/Tracker` | `types/tracking/tracker.hpp` | 資源。見[資料放在哪裡](#資料放在哪裡) |
| `tracking/State` | `types/tracking/state.hpp` | `FW_ENUM`：`NO_IMAGES_YET`、`NOT_INITIALIZED`、`OK`、`RECENTLY_LOST`、`LOST`（`SYSTEM_NOT_READY` 與 `OK_KLT` 在 ORB-SLAM3 中沒有使用，不列入） |
| `tracking/Status` | `types/tracking/status.hpp` | 訊息。狀態、inlier 數、地圖 id、keyframe 與 map point 數、各階段時間 |
| `tracking/MapperStatus` | `types/tracking/mapper_status.hpp` | 訊息，local mapping 送出。`accept_keyframes`、`keyframes_in_queue`、`initializing`、`stopped`、`bad_imu`、`far_points`／`th_far_points`，以及 IMU 初始化的結果（序號、scale、bias） |
| `tracking/MapperCommand` | `types/tracking/mapper_command.hpp` | 訊息。`reset_active_map`（map id）、`reset_all`、`interrupt_ba` |
| `OrbTrackingParams` | `params/orb_tracking_params.hpp` | 參數。`sensor`（`stereo`、`stereo_inertial`）、`fps`、`th_depth`、`insert_kfs_when_lost`。IMU 雜訊取自 `calib`；`time_recently_lost`（5 s）與 stereo 的 3 s 寫在樹檔中（`lost_time_within` 的 `max_s`） |

## 已決定的事項

| 事項 | 決定 |
|---|---|
| 特徵的輸入 | `orb_matcher` 多送一個 `orb/StereoFrame`；tracking 只有一個 `trigger` 輸入 |
| IMU | 上游的 `imu_preintegration` 以 b̄ = 0 預積分，tracking 以 Jacobian 修正並串接；bias 回饋之後再決定 |
| 程式碼 | 不整包移植；邏輯與數學一致、實作重寫，依[原則](#原則)分散放置；不引入 g2o、Sophus、DBoW2 |
| 驗證 | reference 刪除前先產生標準答案（`tests/fixtures/orb_slam3/`） |
| 詞彙檔 | 壓縮的 `.bin` 進 git，解壓後的不進 |
| localization only 模式 | 這一步不做 |
| `pose` | body 在 world 中的位姿 |
| 軌跡存檔 | 之後另做 block；這一步只送逐影格的 `pose` |
