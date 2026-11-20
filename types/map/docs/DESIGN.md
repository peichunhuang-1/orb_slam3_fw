# map/Atlas 的 tile 化：tracking 無鎖讀取地圖

這份文件定 atlas 怎麼改成 `fw::TiledMap` 組成的 COW 型別，讓 `atlas` 資源能用 `policy: versioned`。改完之後 tracking 永遠讀完整一致的快照，local mapping、loop closing 寫入時都不擋 tracking。

- 型別：[`types/map/`](../)
- 相關設計：[orb_tracking](../../../algorithms/orb_tracking/docs/DESIGN.md)、[orb_local_mapping](../../../algorithms/orb_local_mapping/docs/DESIGN.md)
- fw 文件：`docs/concepts/tiles.md`、`docs/concepts/resources.md`（versioned 一節）

## 為什麼要改

| | tracking 被擋的時機 | tracking 讀到的地圖 |
|---|---|---|
| ORB-SLAM3 | 有人拿 `Map::mMutexMapUpdate` 時：local BA、IMU 初始化、loop correction、merge、GBA 的寫回 | 可能改到一半（物件各自的小鎖，靠 `isBad()`、`GetReplaced()` 容忍） |
| 現在的結構（`writer_pref`） | local mapping 的**每個 tick**（fw 的鎖一個 tick 拿一次） | 完整一致 |
| 本設計（tile + `versioned`） | 不擋 | 完整一致的快照，可能晚一版 |

另一個理由是之後的光達：點雲地圖每幀都要更新。如果每次更新都要複製整張地圖，代價太高；用 tile 的話，只會複製改到的 tile 和它到根的路徑。

## 第 0 步：效能實測

量測環境：Raspberry Pi 5（4 核），`-O2`。tile 用的是接近 `MapPoint` 大小的 struct，裡面帶一個 6 個元素的 observation vector。查詢的 id 刻意打亂，所以這是沒有區域性的最壞情況。

| 點數 | tracking 每幀 12000 次查詢（局部地圖 4000 點 × 3 次）<br>vector / TiledMap | 每個 keyframe 的寫入（複製版本，改 300 點、加 150 點） | 每個點都改一次（GBA、loop 的寫回）<br>TiledMap / vector 原地改 | 保留 3 個舊版本時活著的 tile |
|---|---|---|---|---|
| 10k | 0.04 / 0.34 ms | 1.1 ms | 7.8 / 0.24 ms | 4.5 MB |
| 50k | 0.04 / 0.57 ms | 1.7 ms | 28.7 / 1.2 ms | 9.0 MB |
| 200k | 0.04 / 0.83 ms | 2.0 ms | 95 / 5.0 ms | 25.8 MB |

結論：

- tracking 每幀多花不到 1 ms，相對於特徵擷取和匹配可以忽略。
- local mapping 每個 keyframe 多花 1–2 ms。一個 keyframe 本來就要處理數十 ms，這點可以接受。
- 全圖寫回慢了約 20 倍，但它只在 loop 和 GBA 時發生，而 GBA 本身就要算好幾秒，這個成本攤得掉。而且寫回時 tracking 不會被擋，這點比原版好。
- 舊版本留住的 tile 和寫入量成正比，不會和地圖大小成正比。用 `max_retained_tiles` 監控即可。

## 型別

### Atlas 本身

Atlas 改成 `FW_COW_TYPE`。它的直接成員只能是 `TiledMap`、其他 COW 型別，或 trivially copyable 的值：

```cpp
struct Atlas {
    fw::TiledMap<KeyFrameId, KeyFrame> keyframes;
    fw::TiledMap<MapPointId, MapPoint> points;
    fw::TiledMap<MapId, Map> maps, bad_maps;
    fw::TiledMap<int, camera::StereoRig> rigs;  // 0..n_rigs-1
    BowIndex bow_index;                         // a COW type: fw::TiledMap<WordId, std::vector<KeyFrameId>>
    MapId current; KeyFrameId last_init_kf_id; MapId next_map_id;
    KeyFrameId next_keyframe_id; MapPointId next_point_id; int n_rigs;

    const KeyFrame* keyframe(KeyFrameId) const;  // find
    KeyFrame* editKeyFrame(KeyFrameId);          // edit: copies the tile (and its path) once per version
    // likewise point / editPoint, findMap / editMap, currentMap / editCurrentMap, goodKeyFrame / goodPoint, rig(i)
};
FW_COW_TYPE(map::Atlas, keyframes, points, maps, bad_maps, rigs, bow_index, current, ...)
```

- `edit*` 回傳的是指標，不是 `fw::TileRef`：呼叫端原本就以 `KeyFrame&` 傳遞，而 `TileRef` 沒有 `operator*`。這個指標和 `TileRef` 的有效期間相同：atlas 被複製、指派、清空，或 handler 返回之後就失效，所以不能存起來。對**另一個**鍵 `edit` 不會讓它失效。
- 同一個 handler 裡，先用 `keyframe()` 拿到的 const 指標，在同一個 tile 被 `edit` 之後看到的是舊內容（`versioned` 下寫入者拿到的是副本，第一次 `edit` 才複製 tile）。所以寫入的程式要改一個 tile 時，在 `edit` 之後就只用 `edit` 給的指標。`writer_pref` 下 atlas 不會被複製，`edit` 不複製，這個問題不會出現。
- 唯讀的程式只拿 `const Atlas&`，而 `const` 版只有查詢的函式。所以只要編譯過，就保證它沒有寫入。

### KeyFrame：meta 與 data 分開

每次 `edit` 都會複製整個 tile。features、BoW 這類大東西做完 keyframe 後就不會再變，所以把它們移到一個不可變、共享的區塊：

```cpp
struct KeyFrameData {  // never changes once the keyframe is made
    std::shared_ptr<const orb::FrameFeatures> features;
    BowVector bow; FeatureVector feat;
    int rig; geometry::SE3f Tlr, Trl; float baseline, th_depth;
    long frame_id; double timestamp; bool has_imu_calib; geometry::SE3f T_cb;
};
struct KeyFrame {      // the tile: what changes
    std::shared_ptr<const KeyFrameData> data;
    std::vector<MapPointId> points;          // per feature
    pose, velocity, bias, imu, prev / next, map, origin_map
    std::shared_ptr<const imu::Preintegrated> preintegrated;  // replaced, never changed in place
    covisibility (flat sorted vectors), parent / children / loop / merge edges, flags
};
```

- 複製一個 tile 的代價主要是 `points`（每個 feature 8 bytes，約 1000–2000 個），也就是 8–16 KB。
- `preintegrated` 原本是可變的 `shared_ptr`，`setNewBias` 會原地修改。這在 COW 下是錯的，因為舊版本也會跟著被改。現在改成 `shared_ptr<const>`，`KeyFrame::setNewBias` 會複製一份、改好再換掉。
  - 原版的這個物件也和 tracking 的 frame 共用：tracking 的 `t.last.preintegrated` 可能就是最新 keyframe 的那份。原版 local mapping 改 keyframe 的 bias 時，tracking 的 frame 也會跟著變（跨執行緒，時機不固定）；這裡不會跟著變，要等 tracking 套用 `MapperStatus` 的 IMU 結果時，才設成同一個 bias。
  - `create_new_keyframe` 原本呼叫 `kf.setNewBias(f.bias)`，現在改成建立 keyframe **之前**，由 tracking 直接設定 frame 的預積分。這時物件還沒進 atlas，和原版改的是同一個物件、同一個時間點。
  - 反方向：tracking 的 `t.last.setNewBias` 原本會原地修改一個 atlas 也持有的物件。第 2 步起，`apply_mapper_update` 在 `t.last.preintegrated` 與 keyframe 共用時先複製一份再改；`stereo_initialization` 也複製 keyframe 的預積分。
- `connected`、`children` 這類 `std::map`、`std::set` 改成排序過的 vector（[`flat.hpp`](../flat.hpp) 的 `FlatMap`、`FlatSet`，介面與 `std::map`、`std::set` 相同）。放在 tile 裡也能用，但複製成本比較高。走訪順序同樣依 id，所以結果不變。
- BoW 由 local mapping 的 `ProcessNewKeyFrame` 計算，算好才寫進 data。所以 data 建好之後就不會再變。

### MapPoint、Map

- `MapPoint` 本身就是 meta，直接當 tile。`observations` 從 `std::map` 改成依 keyframe id 排序的 vector，順序和現在一樣（依 id）。
- `Map` 拿掉 `points` 集合，改成計數 `n_points`，每個點記自己是否在地圖裡（`MapPoint::in_map`）。需要「這張地圖的全部點」（`GetAllMapPoints`）的地方很少，只有 stereo 初始化、clearMap，以及之後的 merge 和 GBA。它們改用 `mapPoints(a, m)`：走訪全部點、過濾後依 id 排序，順序和原本的集合相同。
- `keyframes` 留下，改成 `FlatSet`（數千個 id，複製代價小），因為 IMU 初始化和 GBA 要依序取用。
- `bad_maps` 保留成另一個 `TiledMap`，和原本一樣，`maps.size()` 仍只算活著的地圖。

## tracking 改成只讀

tracking 的葉節點不再寫入 `atlas` 資源。做法（第 2 步，與原本規劃的覆蓋層不同，見下）：

- **tracking 自己的副本**：`Tracker::view` 是一份 `map::Atlas`。tile 是共用的，所以複製是 O(1)；tracking 在副本上做的修改只會複製被改到的 tile。
- **每幀開頭同步**：根樹的第一個葉節點 `sync_atlas` 是 tracking 唯一讀取資源的地方。它把 `view` 換成資源目前的版本，再把資源還沒有的 tracking 修改重新套用一次。這一幀其餘的葉節點都只看 `view`，所以整幀讀到的是同一份一致的 atlas，裡面也有 tracking 自己的修改。
- **修改以訊息送出**：tracking 每個修改都是一筆 `tracking::AtlasEdit`（[`types/tracking/atlas_edits.hpp`](../../tracking/atlas_edits.hpp)），帶遞增的 `seq`。修改立即套用到 `view`，並放進 `Tracker::pending`。有新修改的 tick 由 `atlas_edits` 輸出送出 `pending` 的全部內容；local mapping 的 `apply_tracking_edits` 以 `applyEdit` 依序套用到資源，跳過 `seq <= Atlas::tracking_seq` 的（已套用過）。`sync_atlas` 依 `tracking_seq` 把已套用的從 `pending` 刪掉。訊息只保留最新一筆也不會漏，因為每筆都帶著全部未套用的修改。

| 原本 tracking 寫的 | 改成的 edit |
|---|---|
| `++visible`、`++found`（track_local_map） | 累加在 `Tracker::counts`，建立 keyframe 時以 `counts` edit 送出，只加到資源、不重新套用到 `view`。culling 只在處理 keyframe 時才看這兩個數，所以語意幾乎不變 |
| `createKeyFrame`、`createStereoPoint`（create_new_keyframe、stereo_initialization） | `keyframe` edit：整個新 keyframe、新 stereo 點的 id 與位置、是否插入 local mapping 的佇列 |
| `addRig`（make_frame） | `add_rig` edit |
| reset：`clearAtlas`、`clearMap`、`createNewMap`（apply_pending_reset） | `reset_all`、`reset_active_map` edit。local mapping 依序套用，同時丟掉排在它之前的 keyframe、在適當的時機重設自己。原版 tracking 會等 local mapping 重設完（`RequestReset`）；這裡由 edit 的順序保證，tracking 不等 |
| `createMapInAtlas`（start_new_map、handle_lost、handle_discontinuity） | `new_map`、`map_inertial` edit，地圖 id 由 tracking 配發 |
| `Map::reference_points`（只供顯示）、`change_notified` | 前者拿掉（沒有顯示）；後者移到 `Tracker::change_notified` |

`mapper_cmd` 只剩 `interrupt_ba`。

### id 的配置

- keyframe id、地圖 id 由 tracking 配發（`view` 裡的 `next_kf_id`、`next_map_id`，隨 edit 帶到資源）。loop closing 不會建 keyframe。
- 點的 id 依來源分開，避免兩邊撞號：tracking 用偶數（`Atlas::next_tracking_point_id`），local mapping 用奇數（`next_point_id`）。原版的點 id 本來就不連續，也沒有邏輯依賴它。

### 為什麼不用覆蓋層

原本規劃 tracker 只保留「已送出、快照裡還沒有」的 keyframe 和點，查詢時先查覆蓋層再查快照。改成「副本加重新套用」之後，所有查詢都不用改（照樣查 `view`），也不會漏掉某個查詢路徑；代價只是每幀重新套用幾筆未確認的 edit，通常是零到一筆。

### 已知問題：job 模式不會結束

fw 0.1.0 的 bug：行為樹 block 讀取某個資源、又在該資源寫入者的 `trigger` 上游時，job 結束時收不到 EOS。tracking（讀 `atlas`）正是 local mapping（寫 `atlas`）的上游，所以 `slam_job` 跑完資料後不會自行結束，要手動停止。已回報 fw。繞過方式是把 `sync_atlas` 宣告為 `write`，但會失去 `versioned` 的好處（第 3 步），所以沒有採用。

## 步驟

| 步 | 內容 | 驗證 |
|---|---|---|
| 0 | TiledMap 與 vector 的效能比較 | **已完成**，見上面的表 |
| 1 | 換掉底層資料結構（Atlas 改成 COW、KeyFrame 拆成 meta 與 data、容器改成 vector），資源仍用 `writer_pref` | tracking 的 fixture 和 V1_02 的軌跡與現在一致。**已完成**，見下 |
| 2 | tracking 改成只讀：tracking 的副本、修改以 edit 送出、id 配置 | 同上，另外確認 tracking 的葉節點不再拿資源。**已完成**，見下 |
| 3 | `atlas` 改成 `policy: versioned`，量 tracking 的延遲和留住的 tile 數 | tracking 每幀的時間分布不再受 local mapping 影響。**已完成**，見下（留住的 tile 數沒有量） |
| 4 | 填 local mapping 的 stub（照 orb_local_mapping 的設計，寫入改用 `edit`） | local mapping 自己的 fixture |

### 第 1 步的驗證

- 14 個測試全部通過（`atlas_test`、`search_by_bow_test` 已改用新的存取方式）。
- V1_02 的 slam job（tracking 加 local mapping）：每一幀印出狀態、地圖的 keyframe 數與點數、相機位置，和改動前逐行比對。
  - 改動前連跑兩次，結果逐行相同，確認可以重現：1710 幀，結束時 324 個 keyframe、15669 個點。
  - 改動後也逐行相同，每次約 90 秒，和改動前一樣。

### 第 2 步的驗證

- 14 個測試全部通過。
- V1_02 的 slam job：與第 1 步的輸出逐行相同（1710 幀，324 個 keyframe、15669 個點）。跑完資料後不會自行結束（見上面的已知問題）；暫時把 `sync_atlas` 宣告為 `write` 時，約 93 秒自行結束。
- tracking 的葉節點都不再宣告 `atlas` 資源，只有 `sync_atlas` 以 `read` 使用。

### 第 3 步的驗證

`slam_job` 的 `atlas` 改成 `policy: versioned`。量的是每幀的「等待」：`imu_preintegration` 送出 frame 到 tracking 的 `make_frame` 開始之間的時間（steady clock，暫時加的輸出，已移除）。

- **`lossless: true`（評估用的設定）量不出差別**：driver 每送一筆都等整個框閒下來，local mapping 也包括在內，所以 tracking 和 local mapping 不會同時執行。兩種策略的等待都是 p99 1.4 ms，軌跡與第 2 步逐行相同（1710 幀，324 個 keyframe、15669 個點）。local mapping 變慢只會讓整段變長，不會讓 tracking 等。
- **`lossless: false`（即時播放）、在 `map_point_culling` 暫時加 50 ms 的 sleep**（模擬之後的三角化、fuse，在 `ProcessNewKeyFrame` 之後、同一個 tick 內）：

| | 等待 mean | p90 | p99 | 結束時 |
|---|---|---|---|---|
| `writer_pref` | 3.9 ms | 18.3 ms | 28.2 ms | 324 個 keyframe、15669 個點 |
| `versioned` | 0.4 ms | 0.8 ms | 1.5 ms | 同上 |

  tracking 本身每幀約 14 ms，兩者相同。max 約 170–180 ms 的是開頭的幾幀，兩者都有。

- **`ProcessNewKeyFrame` 要在下一幀之前完成**：同樣的 50 ms sleep 改放在 `process_new_keyframe` 之前時，`versioned` 下 tracking 在初始化後的下一幀就 LOST，之後不斷重設。新的 keyframe 在 `ProcessNewKeyFrame` 之前沒有 BoW、觀測與 covisibility，`TrackReferenceKeyFrame` 找不到配對。`writer_pref` 下 tracking 會等它做完，所以看不出來。原版也是這樣（tracking 不等 local mapping，只是 `ProcessNewKeyFrame` 很快），所以行為與原版一致。`map_point_culling` 那次不受影響，是因為行為樹 block 寫入 `versioned` 資源時，**每個葉節點回傳就發布一個新版本**（寫入鎖則持有整個 tick）；`process_new_keyframe` 一回傳，新 keyframe 的 BoW 與觀測就看得到了，後面的 sleep 只延後之後的版本。所以第 4 步不必把同一個 tick 裡較慢的步驟拆出去；只要別把耗時的工作放進 `process_new_keyframe` 或它之前。
