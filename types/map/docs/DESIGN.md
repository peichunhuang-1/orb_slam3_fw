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
  - 反方向還沒處理：tracking 的 `t.last.setNewBias` 會原地修改一個 atlas 也持有的物件。值和 local mapping 設的一樣，所以目前沒有影響，但第 2 步 tracking 改成只讀時要改成複製。
- `connected`、`children` 這類 `std::map`、`std::set` 改成排序過的 vector（[`flat.hpp`](../flat.hpp) 的 `FlatMap`、`FlatSet`，介面與 `std::map`、`std::set` 相同）。放在 tile 裡也能用，但複製成本比較高。走訪順序同樣依 id，所以結果不變。
- BoW 由 local mapping 的 `ProcessNewKeyFrame` 計算，算好才寫進 data。所以 data 建好之後就不會再變。

### MapPoint、Map

- `MapPoint` 本身就是 meta，直接當 tile。`observations` 從 `std::map` 改成依 keyframe id 排序的 vector，順序和現在一樣（依 id）。
- `Map` 拿掉 `points` 集合，改成計數 `n_points`，每個點記自己是否在地圖裡（`MapPoint::in_map`）。需要「這張地圖的全部點」（`GetAllMapPoints`）的地方很少，只有 stereo 初始化、clearMap，以及之後的 merge 和 GBA。它們改用 `mapPoints(a, m)`：走訪全部點、過濾後依 id 排序，順序和原本的集合相同。
- `keyframes` 留下，改成 `FlatSet`（數千個 id，複製代價小），因為 IMU 初始化和 GBA 要依序取用。
- `bad_maps` 保留成另一個 `TiledMap`，和原本一樣，`maps.size()` 仍只算活著的地圖。

## tracking 改成只讀

現在 tracking 會寫 atlas 的地方，全部改成由 local mapping 代寫：

| 現在 tracking 寫的 | 改成 |
|---|---|
| `++visible`、`++found`（track_local_map） | tracker 自己累加，隨下一個 keyframe 訊息送出，由 local mapping 加進去。culling 只在處理 keyframe 時才看這兩個數，所以語意幾乎不變 |
| `createKeyFrame`、`createStereoPoint`（create_new_keyframe、stereo_initialization） | keyframe 訊息直接帶上整個新 keyframe 和新的 stereo 點，由 local mapping 插入 |
| `addRig`（make_frame） | 隨第一個 keyframe 送出 |
| reset：`clearAtlas`、`clearMap`、`createNewMap` | 改用 `mapper_cmd` 送出，tracking 等 status 的 seq 確認完成後再繼續（原版的 `RequestReset` 本來就會等） |
| `createMapInAtlas`（start_new_map、handle_lost、handle_discontinuity） | tracking 自己配新的地圖 id，放進 keyframe 訊息 |

### id 的配置

- keyframe id 由 tracker 配發，它是唯一會建立 keyframe 的地方。loop closing 不會建 keyframe。
- 點的 id 依來源分開，避免兩邊撞號：tracking 用偶數，local mapping 用奇數。原版的點 id 本來就不連續，也沒有邏輯依賴它。
- 地圖 id 由 tracker 配發。

### tracking 自己的覆蓋層（overlay）

tracking 送出新 keyframe 之後，到 local mapping 把它插入、快照更新之前，有一段時間快照裡還沒有它。但下一幀馬上就要把它當成參考 keyframe，也要用到它的點。

所以 tracker 自己保留「已送出、快照裡還沒有」的 keyframe 和點。查詢時先查這份覆蓋層，再查快照，等快照裡出現了就從覆蓋層刪掉。這是這次改動裡最複雜的地方，在第 2 步實作，用 tracking 的 fixture 驗證結果一致。

另一個做法是 tracking 只在建立 keyframe 的那一幀等 local mapping 插入完成。這樣比較簡單，但那幀會被擋到 local mapping 的 tick 結束，等於又回到會擋的情況。所以不建議。

## 步驟

| 步 | 內容 | 驗證 |
|---|---|---|
| 0 | TiledMap 與 vector 的效能比較 | **已完成**，見上面的表 |
| 1 | 換掉底層資料結構（Atlas 改成 COW、KeyFrame 拆成 meta 與 data、容器改成 vector），資源仍用 `writer_pref` | tracking 的 fixture 和 V1_02 的軌跡與現在一致。**已完成**，見下 |
| 2 | tracking 改成只讀：keyframe 訊息、id 配置、覆蓋層、reset 走 `mapper_cmd` | 同上，另外確認 tracking 的程式只拿 `const Atlas&` |
| 3 | `atlas` 改成 `policy: versioned`，量 tracking 的延遲和留住的 tile 數 | tracking 每幀的時間分布不再受 local mapping 影響 |
| 4 | 填 local mapping 的 stub（照 orb_local_mapping 的設計，寫入改用 `edit`） | local mapping 自己的 fixture |

### 第 1 步的驗證

- 14 個測試全部通過（`atlas_test`、`search_by_bow_test` 已改用新的存取方式）。
- V1_02 的 slam job（tracking 加 local mapping）：每一幀印出狀態、地圖的 keyframe 數與點數、相機位置，和改動前逐行比對。
  - 改動前連跑兩次，結果逐行相同，確認可以重現：1710 幀，結束時 324 個 keyframe、15669 個點。
  - 改動後也逐行相同，每次約 90 秒，和改動前一樣。
