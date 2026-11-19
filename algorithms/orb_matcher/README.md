# orb_matcher

雙目特徵配對（KB8 魚眼模型 + 極線約束），接在 `orb_extractor` 後面。演算法見 [docs/ALGORITHM.md](docs/ALGORITHM.md)。

## Block `OrbMatcher`

| 埠 | 型別 | 說明 |
|---|---|---|
| `left`, `right` | `orb::Features`（sync，`SYNC_SLOP(1ms)`） | 兩眼的 `orb_extractor` 輸出。次像素修正讀它們的金字塔（extractor `keep_pyramid: true`，預設開，見下） |
| `calib` | `calib::StereoImuRig`（latest） | `euroc_mcap_driver.calib`。KB8（equidistant）直接用；radtan（plumb_bob）擬合成 KB8（[rig_from_calib.hpp](../../types/camera/rig_from_calib.hpp)） |
| `params` | `OrbMatcherParams`（latest，可不接） | [params/orb_matcher_params.hpp](../../params/orb_matcher_params.hpp)、[config/orb_matcher.yaml](../../config/orb_matcher.yaml) |
| `gpu` | `gpu::Device`（資源，write） | 框的 GPU（[types/gpu/device.hpp](../../types/gpu/device.hpp)），跟兩個 extractor 接同一個 |
| `match` | `orb::Match` | [types/orb/match.hpp](../../types/orb/match.hpp)，時間戳記同左眼 |
| `stereo` | `orb::StereoFrame` | [types/orb/stereo_frame.hpp](../../types/orb/stereo_frame.hpp)：兩眼的 keypoint、描述子、bearing 與配對結果，給 tracking（stereo-inertial 時先經過 `imu_preintegration`）。時間戳記同左眼 |

```yaml
edges:
  - orb_extractor_left.features -> orb_matcher.left
  - orb_extractor_right.features -> orb_matcher.right
  - euroc_mcap_driver.calib -> orb_matcher.calib
  - config://orb_matcher.yaml -> orb_matcher.params
  - gpu -> [orb_extractor_left.gpu, orb_extractor_right.gpu, orb_matcher.gpu]   # boxes.main.resources: gpu: { type: gpu/Device }
```

`orb::Match` 的欄位對應 ORB-SLAM3 fisheye 雙目 `Frame`：`left_to_right` / `right_to_left`（-1 = 沒配到）、
`points`（左相機座標）、`depth`（z，-1 = 沒有）、`right_uv`（次像素修正後的右點）、兩眼的 `bearing`，
另有 `matches`（`cv::DMatch`：query = 左、train = 右、distance = Hamming）與 `stats`。

健康狀態：

- ok：訊息是相機模型（擬合誤差、基線）
- degraded：特徵沒有金字塔（這時配對照做，但不做次像素修正）、GPU 次像素修正失敗（改用 CPU），或 KB8 擬合 p99 > `max_fit_error_px`
- error：標定無法使用（未知畸變模型、基線為 0 等），這一組會被跳過

特徵的影像大小（`orb::Features::width/height`）跟標定不同時，rig 會自動依比例縮放（長寬比不同就丟例外）。
參數、標定或影像大小改變時，下一組才重建配對器。資料結束時印出統計（組數、每組配對數、時間）。

## 次像素修正在哪裡跑（不經 CPU 拷貝）

| extractor | `orb::Features` 帶的金字塔 | `subpixel_device: auto` |
|---|---|---|
| GPU | `gpu_pyramid`：GPU 上複製到 buffer 池（`vkCmdCopyBuffer`，每眼約 0.36 ms GPU，CPU 不碰） | **GPU**（`gpu/subpixel_vk`），每對一次提交 |
| CPU | `pyramid`：直接共用 extractor 的 `cv::Mat`（下一幀有人還拿著就改寫進新的影像） | CPU |

- GPU 是框的共享資源 `gpu/Device`（[types/gpu/device.hpp](../../types/gpu/device.hpp)），圖上接到兩個 extractor 和 matcher：
  同一個 Vulkan 裝置，matcher 的 shader 才讀得到 extractor 的 buffer。三個 block 都用 **write**（獨占）取用，
  Vulkan 的 queue 一次只能一條執行緒用，由框架的鎖保證，型別裡不放鎖。buffer 在最後一個持有它的 message 釋放時回到池裡。
- 代價：三個 block 用 GPU 的 handler 不會同時執行（擷取 CPU 部分每眼約 1.5 ms 不能重疊）。`lossless: true` 時 driver 本來就逐筆送，
  實測沒有差別；照錄影時間送（`lossless: false`）時的影響還沒量。
- 金字塔不在 matcher 那個 `gpu/Device` 上（圖接了兩個不同的資源）時，改用 CPU 讀，健康狀態訊息會寫原因。
- Pi 5 的 GPU 記憶體對 CPU 不經 cache（約 1 GB/s），所以 GPU 金字塔不讀回 CPU；`subpixel_device: cpu` 時 CPU 直接讀它的 mapping，
  結果相同但次像素修正慢約 3.5 倍（6.8 vs 1.95 ms／對），只當後備用。
- GPU 與 CPU 的結果差在 float 捨入：同一批 20,000 次搜尋，接受／拒絕完全一致，位移差 p99 0.0003 px（`orb_matcher_subpixel_gpu_test`）。
- 視窗 `subpixel_half_window` ≤ 7、`subpixel_range` ≤ 8 才能上 GPU（shader 的 shared memory），超過時 `auto` 用 CPU。

## 檔案

| 路徑 | 內容 |
|---|---|
| `orb_matcher.cpp` | fw block |
| `stereo_matcher.hpp/.cpp` | `StereoMatcher`、`StereoMatcherParams`、`StereoMatches`（不依賴 fw） |
| `core/` | bearing（θ, φ, σ）、依 θ 排序的右點索引、閉式三角化、沿極線次像素修正 |
| `../../types/camera/rig_from_calib.hpp/.cpp` | `camera::stereoRigFromCalib`：`calib::StereoImuRig` → `StereoRig`，radtan → KB8 擬合、`T_c1_c2`（tracking 也用） |
| `../../types/camera/kannala_brandt.hpp/.cpp` | `camera::KannalaBrandt8`：project / unproject（單位向量）/ Jacobian / `pixels_per_radian` / `resized` |
| `../../types/camera/stereo_rig.hpp/.cpp` | `camera::StereoRig`：兩個 KB8 + `T_c1_c2` |
| `../../types/orb/hamming.hpp` | `orb::hamming256`（NEON） |
| `gpu/subpixel_vk.hpp/.cpp`、`gpu/shaders/subpixel.comp` | GPU 次像素修正（`SubpixelBackend`），讀 extractor 留在 GPU 上的金字塔 |
| `../../types/gpu/device.hpp/.cpp` | 資源型別 `gpu/Device`：框的 GPU，第一次用時建立 |
| `../../types/gpu/context.hpp/.cpp` | Vulkan 裝置（`gpu::Context`）、GPU 金字塔與 buffer 池 |
| `../../tests/orb_matcher/` | `kb8_test`、`epipolar_synth_test`（對照 ORB-SLAM3 fisheye）、`subpixel_synth_test`、`subpixel_gpu_test`（GPU 對 CPU、金字塔交接） |

## 測試

```bash
colcon build
build/orb_slam3_fw/orb_matcher_kb8_test
build/orb_slam3_fw/orb_matcher_epipolar_test --trials 50
build/orb_slam3_fw/orb_matcher_subpixel_test --trials 10
build/orb_slam3_fw/orb_matcher_subpixel_gpu_test             # 有 Vulkan 時
fw launch graphs/example_job.graph.yaml    # EuRoC V1_02：driver → 兩個 extractor → matcher
```

框架內實測（`graphs/example_job.graph.yaml`，Pi 5，每眼 1000 點，GPU 擷取，V1_02，`lossless: true`）：1710 組全部配對，平均 337 對／組。

| | 金字塔 CPU 拷貝 + CPU 次像素（舊） | GPU 交接 + GPU 次像素（現在） |
|---|---|---|
| 整個 process CPU | 33.5 s（一核的 34.4%） | **22.3–23.2 s（24.7–25.7%）** |
| 配對時間／組（wall，含等 GPU） | 3.6 ms | 3.8 ms |
| 延遲（擷取開始 → 配對完成）平均 | 50.0 ms | **46.0 ms** |
| 輸出頻率 | 17.7 Hz | 19.3 Hz |

## 量頻率與延遲

- **輸出頻率、handler 時間**：圖在跑的時候另開一個終端機
  `python3 tools/fw_stats.py --block orb_matcher --interval 10`（讀 fw_agent 的統計，跟編輯器面板同一份；
  不給 `--block` 就列全部 block）。每段時間印出 `match` 的 Hz、`on_pair` 平均時間，有丟資料或 sync 沒配成時也會列出；結束時印整段平均。
- **端到端延遲**：extractor 在開始擷取時記下 steady clock（`orb::Features::extract_start_ns`），matcher 算
  「兩眼中較早開始擷取的那張 → 配對完成」，放在 `orb::Match::stats.latency_ms`，資料結束時印平均 / 中位 / p95 / 最大值與輸出頻率（wall clock）。
  不含 driver 解碼 PNG 和 extractor 前的排隊（後者 fw 統計裡約幾 µs）。
- fw 統計裡輸出的 `age`（ROS 時鐘 − 時間戳記）在這張圖沒有值（`use_sim_time` + bag 時間戳記），所以延遲用上面這個。
- 同一時間只跑一張圖：兩張 EuRoC 圖都會發 `/clock`，會互相干擾（sync 大量配不成）。

延遲大部分是兩眼在 GPU 上的擷取；數字見上一節的表。

## 參數（實測依據）

以下數字是移植前的版本在 Pi 5 上量的（單核，每眼 500 點，EuRoC V1_02_medium 全部 1710 組，
KB8 擬合 radtan，基線 11 cm，深度真值為 Leica 掃描）。

| | 次像素開（預設） | 次像素關 | ORB-SLAM3 fisheye |
|---|---|---|---|
| 配對數 / 幀 | **186** | 187 | 159 |
| 時間 mean / p99 | 1.43 / 2.23 ms | **0.52 / 0.78 ms** | 0.99 / 1.49 ms |
| 深度中位相對誤差 | **2.77%** | 4.49% | 4.24% |
| 深度誤差 > 20% 的比例 | **4.5%** | 8.7% | 6.8% |

- `ratio` 0.7：合成場景裡 0.9 沒有多配到正確的點，只多了重複紋理的錯配；EuRoC 上 0.7 → 0.9 配對數 +24%，
  誤差 > 20% 的比例 8.7% → 10.2%。
- `min_parallax_deg` 1.146°（同 ORB-SLAM3 `TriangulateMatches` 的 cos > 0.9998）：視差角更小的配對不當雙目點。
  最遠深度約「基線 / 視差角」：5 cm 基線約 2.5 m、11 cm 約 5.5 m。沒有這個限制時會有點被三角化到上百公尺外。設 0 關閉。
- `subpixel` 預設開、`subpixel_fit: ssd`：合成平面上視差誤差中位 0.397 → 0.174 px（EuRoC rig），比 ORB-SLAM3 的
  SAD + 拋物線（0.183）、SAD + V 形（0.178）都略好。
- 沿極線的重複紋理（同一極線上描述子一樣像的點）只靠描述子 + 幾何分不出來（precision 約 89–91%），要靠深度先驗或時序。
