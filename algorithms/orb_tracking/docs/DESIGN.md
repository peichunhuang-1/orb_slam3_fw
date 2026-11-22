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
| local mapping、loop closing | 不做；介面先定好（`atlas_edits`、`mapper_cmd` 輸出，`mapper` 輸入） | 各自成為一個 block，接上這些埠與 `atlas` 資源 |
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
 │         │ vocabulary (latest) ◄── orb_vocabulary://ORBvoc.bin                     │    │
 │         │                                                                         │    │
 │         │ ──► pose   tracking   atlas_edits ──► local_mapping   mapper_cmd ──►     │    │
 │         └─────────────────────────────────────────────────────────────────────────┘    │
 │                 ▲ read               ▲ write                                           │
 │               atlas               tracker                                （資源）      │
 │                 └──► local_mapping、loop_closing（之後）                               │
 └────────────────────────────────────────────────────────────────────────────────────────┘
```

- **tracking** 是 behavior block，樹是 [`tracking.tree.yaml`](../../../graphs/trees/tracking/tracking.tree.yaml)。只有一個 tick 的來源：`frame`（`trigger`），每個 stereo 影格 tick 一次，對應 `GrabStereoFeatures` → `Track()`。`calib`、`params`、`mapper`、`vocabulary` 是 `latest`。
- **感測器由 `params.sensor` 決定**（`stereo` 或 `stereo_inertial`）。`make_frame` 依它寫入鍵 `inertial`，並檢查 `frame.imu` 是否存在：stereo-inertial 卻沒有預積分時回報錯誤。只做 stereo 時，圖上把 `orb_matcher.stereo` 直接接到 `tracking.frame`，拿掉 `imu_preintegration`。
- **`orb_matcher` 多一個輸出 `stereo`**（`orb/StereoFrame`）：兩眼的 keypoint、描述子、`scale_factors` 與配對結果（`orb::Match` 的內容），**不含金字塔**。`Frame` 需要左右兩眼的特徵（KB8 stereo 的 `N = Nleft + Nright`），由 matcher 一次送出，tracking 就不必 sync 三路資料，也不會因為排隊而持有 extractor 的 GPU 金字塔 buffer。原本的 `match` 輸出保留。
- **`imu_preintegration`**（新的 algorithm，class 形式）。詳見[IMU 預積分](#imu-預積分)。
- **輸出**：
  - `pose`：每個追蹤成功的影格送出 body 在 **`odom`** 中的位姿。`odom` 是連續的座標系，不隨地圖的重新定錨而跳動，見[輸出的座標系](#輸出的座標系odom-與-map)。
  - `map_to_odom`（`fw/TransformStamped`，`map` → `odom`）：與 `pose` 同時送出；body 在地圖（atlas 目前的世界座標系）中的位姿 = `map_to_odom · pose`。
  - `tracking`：每個影格一筆，含狀態、inlier 數、地圖與 keyframe 數。
  - `atlas_edits`：tracking 對地圖的修改（新 keyframe 取代 `InsertKeyFrame`、新地圖、相機、重設、點的 visible／found 計數），交給 local mapping 寫入 `atlas`。
  - `mapper_cmd`：要求 local mapping 中斷 BA。
  - behavior block 另有內建的 `status`。

## 輸出的座標系（odom 與 map）

**與原版不同之處。** 原版的世界座標系會被整個換掉：雙目初始化時是第一個相機的座標系（`Rwb0 = Tcb`），IMU 初始化與 VIBA 1、2 以 `ApplyScaledRotation(Twg)` 把整張地圖轉到重力對齊的座標系，local BA 每次都移動 keyframe 與點，之後的 merge 與 loop closing 也會移動地圖。原版每個影格回傳的 `Tcw` 就在當下的座標系裡，所以即時輸出會跳（V1_02：IMU 初始化時一個影格跳 1.26 m，ground truth 只移動 3.6 cm；之後還有 17 步超過 2 cm，最大 6 cm，來自 BA 的修正）。原版只在結束時以 `SaveTrajectoryEuRoC` 把軌跡換算到最終的座標系。即時位姿給下游（控制、導航）用時不能跳，所以這裡照 ROS REP-105 分成兩個座標系：

- **`map`**：atlas 目前的世界座標系，就是原版的座標系，會被重新定錨。地圖、最佳化、`relative_poses` 軌跡清單都照原版，完全不受影響。
- **`odom`**：連續的座標系，起點是第一個送出的影格當下的 `map`（雙目初始化時：第一個相機的座標系，z 軸不對齊重力）。之後只累積影格之間的移動，地圖怎麼被搬都不會跳。需要重力對齊或全域一致的位置時，用 `map_to_odom` 換回 `map`：body 在 `map` 中 = `map_to_odom · pose`。

**作法：在新的地圖上重新定位上一個影格**（[`publish_frame/odom`](../publish_frame/odom.hpp)，每個送出 `pose` 的影格）。不需要 local mapping 送任何新訊息：地圖的每一種移動都反映在點的位置上。

```
tracker.odom 存：T_map_odom（初值 I）、上一個送出的影格（位姿、特徵、點、outlier）、它的 inlier 點在送出當時的位置、
                它的 reference keyframe（錨點）與錨點當時的 Twc、所在的地圖

每個送出 pose 的影格 f（Twc：f 在目前的 map 中）：
  1. 地圖換了（LOST 後的新地圖、重設）：T_map_odom = Twc · (上一個影格在 odom 中)⁻¹      odom 從上一個位姿接下去
  2. 否則，上一個影格的 inlier 點有沒有移動：
       沒有：D = I
       有：  T_then = PoseOptimization(上一個影格, 點在當時的位置)，從它原本的位姿開始
             T_now  = PoseOptimization(上一個影格, 點現在的位置)，從「隨錨點移動」的位姿開始：D0 · Twc_last
             兩者 inlier 都 ≥ 30：D = T_now · T_then⁻¹                                    （Twc，map_now ← map_then）
             否則：D = D0 = 錨點的 Twc(現在) · Twc(當時)⁻¹；沒有錨點：D = I
       T_map_odom = D · T_map_odom
  3. pose = T_map_odom⁻¹ · Twc · T_body_left⁻¹，map_to_odom = T_map_odom
  4. 記下 f、它的 inlier 點現在的位置、錨點
```

**數學依據。**

1. *位姿是對地圖點的最大概似估計。* tracking 的位姿是 motion-only BA：給定地圖點 X_j，`T = argmin Σ ρ(‖u_j − π(T⁻¹ X_j)‖²_Σ)`（ORB-SLAM 的 PoseOptimization：Mur-Artal et al., "ORB-SLAM: a Versatile and Accurate Monocular SLAM System", IEEE T-RO 2015；ORB-SLAM3：Campos et al., IEEE T-RO 2021）。
2. *規範自由度與等變性。* 重投影誤差只取決於相機與點的相對位置，所以整張地圖做剛體變換 G（X_j → G X_j）時，最佳解正好變成 G·T（BA 的 gauge freedom：Triggs et al., "Bundle Adjustment — A Modern Synthesis", 2000，第 9 節；VIO 裡的 4 自由度：Zhang, Gallego, Scaramuzza, "On the Comparison of Gauge Freedom Handling in Optimization-Based Visual-Inertial State Estimation", IEEE RA-L 2018）。因此 `D = T_now · T_then⁻¹` 對剛體的地圖變化是精確的：IMU 初始化與 VIBA 對每個 keyframe 與點做同一個剛體變換（原版的 `Twg = (Rwg^T, 0)`；stereo 的尺度固定為 1），D 就是它。
3. *非剛體的變化。* BA 讓各處移動的量不同。D 是「同一組觀測、同一個估計器」看到的地圖變化的最佳剛體解釋，而目前的影格正是對這些點（大部分相同）定位的。所以上一個影格與目前影格之間的移動 `Twc_last(now)⁻¹ · Twc(now)` 兩端都在同一個地圖版本裡，就是 tracking 自己估出的移動；地圖的修正全部進 `map_to_odom`。剩下的誤差只來自兩個影格看到的點不完全相同。
4. *差分消掉估計器的偏差。* 「當時」也重新解一次，而不是直接用 tracking 原本的估計（它可能來自 pose-inertial 最佳化、另一組點）；兩次都是同一個估計器，各自的偏差相減後消掉。
5. *map / odom 的分工* 照 ROS REP-105（Coordinate Frames for Mobile Platforms）：odom 連續、可漂移；map 全域一致、可跳；兩者之間的變換承擔修正。VINS-Mono（Qin, Li, Shen, IEEE T-RO 2018）的 relocalization 也是這樣：VIO 的輸出保持連續，另外估出漂移量發布修正後的位姿。

**細節。**
- 「現在」的那次從隨錨點移動的位姿開始：整張地圖的旋轉（IMU 初始化，可達 100° 以上）遠超過 LM 從舊位姿能收斂的範圍；錨點的移動對剛體變化是精確的，所以起點就在解的附近。
- 只在上一個影格的點真的移動時才解（約每個 keyframe 一次，local BA 之後），每次兩個 PoseOptimization。
- inlier 太少（LOST 剛恢復、RECENTLY_LOST 時以 IMU 預測的位姿）時退回錨點的移動；錨點也不在時 D = I。
- LOST 期間不送 pose；同一張地圖上重新定位之後，odom 的跳動就是重新定位修正的量（LOST 期間本來就不知道位姿）。
- keyframe 的 id 在整個 atlas 重設後會重新開始，所以先比較地圖（第 1 步）。map 的 id 不會重用。
- 單元測試 `orb_tracking_odom_test`（pose_optimization fixture 的影格，同一個影格在變化後的地圖上重新追蹤，odom 應該不動）：100° 的剛體變化 1 µm；幾公分的非剛體變化 0.7 mm（只用錨點時 137 mm）；點太少時退回錨點；換地圖時接續。

`tools/ate.py` 預設用連續的 `pose`（`odom`）算 ATE，`--frame map` 換回原版的即時輸出比較；兩者都印出逐影格的步長誤差（與 ground truth 的移動比，就是跳動）。

**實跑**（V1_02_medium 整段，同一次執行的兩種輸出；map 的數字與加入 odom 之前完全相同，地圖與追蹤沒有變）：

| | 步長誤差 p99 | 最大 | > 10 mm | > 20 mm | ATE（SE3）|
|---|---|---|---|---|---|
| map，整段 | 43.1 mm | 1,246 mm（5.90 s，IMU 初始化） | 94 | 55 | 0.138 m |
| odom，整段 | 8.8 mm | 84.7 mm（4.10 s） | 15 | 7 | 0.092 m |
| map，10 s 起 | 20.4 mm | 45.2 mm | 46 | 16 | 0.045 m |
| odom，10 s 起 | 7.4 mm | 13.3 mm | 5 | 0 | 0.051 m |

- `map_to_odom` 變了 351 次（約每個 keyframe 一次）。
- odom 剩下超過 10 mm 的步：11 個在開始追蹤後的第一秒（3.65–4.6 s，地圖剛建立、只有幾個 keyframe；map 的輸出在同一段也一樣大，是追蹤本身的抖動，原因未查），之後 4 個 11–13 mm（28.5、41.3、42.95、77.85 s；map 的輸出在同一時刻也有 11–28 mm）。
- odom 的 ATE 比 map 略大（20 s 起 0.045 對 0.027 m）：這是設計上的取捨。map 的修正（VIBA 對重力方向的修正、BA）會回頭改正整張地圖，odom 不會回頭改，修正都進 `map_to_odom`，所以 odom 會漂移，要全域一致時用 map。

## IMU 預積分

ORB-SLAM3 在 tracking 的執行緒內做 `PreintegrateIMU`：從佇列取出上一影格到這一影格之間的樣本，在兩端內插，然後積分兩次。一次是以 `mLastFrame` 的 bias 為線性化點、從上一影格起算；另一次累加到「從上一個 keyframe 起算」的預積分。這裡把積分移到上游的 block，tracking 只用 bias 的 Jacobian 修正：

**`imu_preintegration`**：`ON(imu)` 累積樣本；`ON(frame)` 收到 `orb/StereoFrame` 後，等到有一筆時間 ≥ 影格時間的樣本，就取跨越上一影格到這一影格的樣本，兩端內插到影格時間後積分（只看時間戳記；`PreintegrateIMU` 用的是固定 1 ms 的 `mImuPer`，影格落在樣本上時兩者相同）。積分的**線性化點固定在 b̄ = 0**，得到 ΔR、ΔV、ΔP、共變異數與 Jacobian（`JRg`、`JVg`、`JVa`、`JPg`、`JPa`）。結果放進影格的 `imu` 欄位，以 `fw::make_msg(影格, 影格的時間)` 送出，時間戳記與影格相同。

- IMU 雜訊與 `T_b_c1` 取自 `calib`。雜訊是連續時間密度，以每一步的 dt（取自樣本時間戳記）離散化，不用設定的 IMU 頻率；每步都是 1/f 時與 ORB-SLAM3 的 `IMU::Calib` 相同（[imu_preintegration/README.md](../../imu_preintegration/README.md)）。
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
| **輸入 `vocabulary`**（`map/Vocabulary`，latest） | ORB 詞彙樹（與 DBoW2 的 ORBvoc 同一份資料） | 很大、唯讀、之後 loop closing 也要用。由 `orb_vocabulary://` plugin（[`filesystem/plugins/orb_vocabulary.cpp`](../../../filesystem/plugins/orb_vocabulary.cpp)）在啟動時載入一次，以 `GUARANTEED` 送出；同一個 box 內各讀取者共用同一份，不複製。不用資源的 `store:`，因為它在結束時一定會存回檔案 |
| **鍵**（blackboard） | `state`（`tracking/State`）、`inertial`、`ok`（`bOK`，只在 `track` 子樹內） | 決定樹的分支，樹檔中看得到，執行時在樹分頁的 Keys 面板上可以觀察 |

**鎖的範圍**：tracking 只在 `sync_atlas` 讀取 `atlas`（O(1) 複製 tile 共用的副本），其餘時間不持有它；`atlas` 是 `policy: versioned`，所以這一次也不必等寫入者。ORB-SLAM3 是在 `Track()` 中持有 `mMutexMapUpdate`。之後的 local BA 與 loop closing 依[鎖外計算](../../../docs/concepts/resources.md#長時間運算鎖外計算)的方式，只在複製與寫回時短暫持有 `atlas`，與 ORB-SLAM3 相同。

**以 id 引用，不用指標**：ORB-SLAM3 的物件以指標互相引用（`Frame::mvpMapPoints`、`KeyFrame` 的 covisibility、`MapPoint` 的觀測）。這裡一律改用 id：`tracker`、`Frame`、`KeyFrame`、`MapPoint` 之間只存 keyframe id 與 point id，使用時向 `atlas` 查。因此 `tracker` 不保留任何指向資源內部的指標，符合 fw 的一般規則；重設或刪除後查不到的 id 直接略過，與 ORB-SLAM3 的 `isBad()` 檢查同義。keyframe 與 map point 存放在以 id 為鍵的 `fw::TiledMap`（[types/map/docs/DESIGN.md](../../../types/map/docs/DESIGN.md)），每次查詢約 50 ns（5 萬點時）。其他 block 對 keyframe 的引用同樣用 id（`map/KeyFrameRef`）。

**重設延後到下一個 tick**：ORB-SLAM3 的 `System::ResetActiveMap()` 只設定旗標，真正的 `Tracking::ResetActiveMap()` 在下一次 `TrackStereo` 的開頭執行。這裡照樣做：葉節點在 `tracker` 記下重設要求，根樹中 `sync_atlas` 之後的 `apply_pending_reset` 在下一個 tick 執行，地圖的部分以 `reset_all`／`reset_active_map` edit 送出，local mapping 依序套用並重設自己，tracking 不等。`CreateMapInAtlas()` 在 ORB-SLAM3 是立即執行的，這裡也立即執行。

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

全部是函式形式的 condition 或 action，不需要 stateful action，因為每個步驟都在一個 tick 內完成，ORB-SLAM3 的 tracking 本來就是同步的。`atlas`、`tracker` 在樹檔中不寫，依同名規則接到樹的資源；`vocabulary` 是樹的輸入，要經由各層 SubTree 傳下去，葉節點寫 `vocabulary: "{vocabulary}"`。只有 `sync_atlas` 使用 `atlas` 資源；其餘葉節點讀寫的是 tracker 裡的副本 `tracker.view`，對地圖的修改以 `atlas_edits` 送給 local mapping。

| 葉節點 | 種類 | 埠（鍵） | 資源 | 對應的 ORB-SLAM3 |
|---|---|---|---|---|
| `sync_atlas` | action | | atlas r、tracker w | 每幀開頭：`tracker.view` 換成資源目前的版本，再套用資源還沒有的 tracking 修改（[types/map/docs/DESIGN.md](../../../types/map/docs/DESIGN.md#tracking-改成只讀)） |
| `apply_pending_reset` | action | `state` inout、`atlas_edits` out | tracker w | `System::TrackStereo` 的 `mbReset`／`mbResetActiveMap` → `Tracking::Reset`／`ResetActiveMap` |
| `make_frame` | action | `frame`、`calib`、`params`、`inertial` out、`atlas_edits` out | tracker w | `newParameterLoader`、`GrabStereoFeatures`（`Frame` 的建構）；`frame.imu` 交給 tracker |
| `apply_mapper_update` | action | `mapper` Optional | tracker w | `UpdateFrameIMU`（由 local mapping 呼叫） |
| `frame_discontinuity` | condition | `state`、`mapper` Optional | tracker r | `Track()` 開頭的 `mbBadImu` 與時間戳記檢查 |
| `handle_discontinuity` | action | `state` inout、`mapper` Optional、`atlas_edits` out | tracker w | 同上的處置：`ResetActiveMap` 要求、`CreateMapInAtlas` |
| `begin_frame` | action | `state` inout | tracker w | `SetNewBias`、`PreintegrateIMU` 的後半（這裡改為修正與串接）、`mbCreatedMap`、`GetMapChangeIndex` |
| `state_is` | condition | `state`、`value` 常值 | | `mState == …` |
| `is_true` | condition | `value` | | 分支條件（`inertial`） |
| `stereo_initialization` | action | `state` inout、`atlas_edits` out | tracker w | `StereoInitialization` |
| `remember_last_frame` | action | | tracker w | 初始化失敗時的 `mLastFrame = Frame(mCurrentFrame)` |
| `start_new_map` | action | `state` inout、`atlas_edits` out | tracker w | `Track()` 的 `else if (mState == LOST)` 分支 |
| `check_replaced_in_last_frame` | action | | tracker w | `CheckReplacedInLastFrame` |
| `use_reference_keyframe` | condition | | tracker r | `(!mbVelocity && !isImuInitialized) \|\| id < reloc + 2` |
| `track_reference_keyframe` | action | `vocabulary` | tracker w | `TrackReferenceKeyFrame`（`ComputeBoW`、`SearchByBoW`、`PoseOptimization`） |
| `track_with_motion_model` | action | | tracker w | `UpdateLastFrame`、`TrackWithMotionModel` |
| `lost_after_pose_failure` | action | `state` inout | tracker w | `if (!bOK)` 之後的 RECENTLY_LOST／LOST 判定 |
| `lost_time_within` | action | `state` inout、`max_s` 常值 | tracker r | `mTimeStamp - mTimeStampLost > …` → LOST |
| `predict_state_imu` | action | | tracker w | `PredictStateIMU`（IMU 未初始化：FAILURE） |
| `relocalize` | action | `vocabulary` | tracker w | `Relocalization`（KeyFrameDatabase、MLPnP） |
| `track_local_map` | action | `state`、`mapper` Optional | tracker w | `TrackLocalMap`（`UpdateLocalMap`、`SearchLocalPoints`、pose (inertial) optimization） |
| `tracking_ok` | action | `state` inout、`ok` out | | `bOK` → `mState = OK` |
| `track_failed` | action | `state` inout、`ok` out | tracker w | `else if (mState == OK)` 的 RECENTLY_LOST 與 inertial 重設要求 |
| `imu_bias_bookkeeping` | action | `ok` | tracker w | `ResetFrameIMU`、`mLastBias` |
| `tracked_or_recently_lost` | condition | `ok`、`state` | | `bOK \|\| mState == RECENTLY_LOST` |
| `update_motion_model` | action | | tracker w | `mVelocity` |
| `clean_vo_matches` | action | | tracker w | Clean VO matches、Delete temporal MapPoints |
| `need_new_keyframe` | action | `ok`、`state`、`params`、`mapper` Optional、`mapper_cmd` out | tracker r | `NeedNewKeyFrame` 加上插入的條件；`InterruptBA` 改為送出 `mapper_cmd` |
| `create_new_keyframe` | action | `mapper` Optional、`atlas_edits` out | tracker w | `CreateNewKeyFrame`；`InsertKeyFrame` 改為 `keyframe` edit |
| `discard_outliers` | action | | tracker w | 丟掉 outlier 的 map point |
| `handle_lost` | action | `state` inout、`atlas_edits` out | tracker w | `Track()` 結尾的 `if (mState == LOST)` |
| `finish_frame` | action | | tracker w | `mpReferenceKF`、`mLastFrame = Frame(mCurrentFrame)` |
| `publish_frame` | action | `state`、`pose` Optional out、`map_to_odom` Optional out、`tracking` out | tracker w | 軌跡清單；`pose` 為 body 在 `odom` 中的位姿，`map_to_odom` 見[輸出的座標系](#輸出的座標系odom-與-map) |

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
  map/vocabulary.hpp/.cpp        詞彙樹、loadBinary、transform、score                                ~350
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
orb_vocabulary/                  orb_vocabulary:// 的資料夾
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
- **詞彙檔**：`orb_vocabulary/ORBvoc.bin.tar.gz` 由 `tools/orb_vocabulary` 從 ORBvoc.txt 轉出，與 reference 的 `ORBvoc.bin` 逐位元組相同。建置時若 `ORBvoc.bin` 不存在，就從壓縮檔解開。

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
| `map/Vocabulary` | `types/map/vocabulary.hpp` | ORB 詞彙樹；`loadBinary` 讀 `.bin`，由 `orb_vocabulary://` plugin 送出 |
| `map/KeyFrameRef` | `types/map/keyframe_ref.hpp` | 訊息。`map_id`、`keyframe_id` |
| `tracking/Tracker` | `types/tracking/tracker.hpp` | 資源。見[資料放在哪裡](#資料放在哪裡) |
| `tracking/State` | `types/tracking/state.hpp` | `FW_ENUM`：`NO_IMAGES_YET`、`NOT_INITIALIZED`、`OK`、`RECENTLY_LOST`、`LOST`（`SYSTEM_NOT_READY` 與 `OK_KLT` 在 ORB-SLAM3 中沒有使用，不列入） |
| `tracking/Status` | `types/tracking/status.hpp` | 訊息。狀態、inlier 數、地圖 id、keyframe 與 map point 數、各階段時間 |
| `tracking/MapperStatus` | `types/tracking/mapper_status.hpp` | 訊息，local mapping 送出。`accept_keyframes`、`keyframes_in_queue`、`initializing`、`stopped`、`bad_imu`、`far_points`／`th_far_points`，以及 IMU 初始化的結果（序號、scale、bias） |
| `tracking/MapperCommand` | `types/tracking/mapper_command.hpp` | 訊息。`interrupt_ba` |
| `tracking/AtlasEdits` | `types/tracking/atlas_edits.hpp` | 訊息。tracking 尚未被套用的地圖修改（`AtlasEdit`，帶 `seq`）；`applyEdit` 套用一筆 |
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
