# orb_matcher

雙目特徵配對（KB8 魚眼模型 + 極線約束），接在 `orb_extractor` 後面。演算法見 [docs/ALGORITHM.md](docs/ALGORITHM.md)。

## Block `OrbMatcher`

| 埠 | 型別 | 說明 |
|---|---|---|
| `left`, `right` | `orb::Features`（sync，`SYNC_SLOP(5ms)`） | 兩眼的 `orb_extractor` 輸出。次像素修正讀它們的 `pyramid`（extractor `keep_pyramid: true`，預設開） |
| `calib` | `calib::StereoImuRig`（latest） | `euroc_mcap_driver.calib`。KB8（equidistant）直接用；radtan（plumb_bob）擬合成 KB8（[rig_from_calib.hpp](rig_from_calib.hpp)） |
| `params` | `OrbMatcherParams`（latest，可不接） | [params/orb_matcher_params.hpp](../../params/orb_matcher_params.hpp)、[config/orb_matcher.yaml](../../config/orb_matcher.yaml) |
| `match` | `orb::Match` | [types/orb/match.hpp](../../types/orb/match.hpp)，時間戳記同左眼 |

```yaml
edges:
  - orb_extractor_left.features -> orb_matcher.left
  - orb_extractor_right.features -> orb_matcher.right
  - euroc_mcap_driver.calib -> orb_matcher.calib
  - config://orb_matcher.yaml -> orb_matcher.params
```

`orb::Match` 的欄位對應 ORB-SLAM3 fisheye 雙目 `Frame`：`left_to_right` / `right_to_left`（-1 = 沒配到）、
`points`（左相機座標）、`depth`（z，-1 = 沒有）、`right_uv`（次像素修正後的右點）、兩眼的 `bearing`，
另有 `matches`（`cv::DMatch`：query = 左、train = 右、distance = Hamming）與 `stats`。

健康狀態：

- ok：訊息是相機模型（擬合誤差、基線）
- degraded：特徵沒有金字塔（這時配對照做，但不做次像素修正），或 KB8 擬合 p99 > `max_fit_error_px`
- error：標定無法使用（未知畸變模型、基線為 0 等），這一組會被跳過

特徵的影像大小（`orb::Features::width/height`）跟標定不同時，rig 會自動依比例縮放（長寬比不同就丟例外）。
參數、標定或影像大小改變時，下一組才重建配對器。資料結束時印出統計（組數、每組配對數、時間）。

## 檔案

| 路徑 | 內容 |
|---|---|
| `orb_matcher.cpp` | fw block |
| `rig_from_calib.hpp/.cpp` | `calib::StereoImuRig` → `StereoRig`：radtan → KB8 擬合、`T_c1_c2` |
| `kannala_brandt.hpp/.cpp` | `KannalaBrandt8`：project / unproject（單位向量）/ Jacobian / `pixels_per_radian` / `resized` |
| `stereo_rig.hpp/.cpp` | `StereoRig`：兩個 KB8 + `T_c1_c2` |
| `stereo_matcher.hpp/.cpp` | `StereoMatcher`、`StereoMatcherParams`、`StereoMatches`（不依賴 fw） |
| `core/` | bearing（θ, φ, σ）、依 θ 排序的右點索引、NEON Hamming、閉式三角化、沿極線次像素修正 |
| `../../tests/orb_matcher/` | `kb8_test`、`epipolar_synth_test`（對照 ORB-SLAM3 fisheye）、`subpixel_synth_test` |

## 測試

```bash
colcon build
build/orb_slam3_fw/orb_matcher_kb8_test
build/orb_slam3_fw/orb_matcher_epipolar_test --trials 50
build/orb_slam3_fw/orb_matcher_subpixel_test --trials 10
fw launch graphs/example_job.graph.yaml    # EuRoC V1_02：driver → 兩個 extractor → matcher
```

框架內實測（`graphs/example_job.graph.yaml`，Pi 5，每眼 1000 點，GPU 擷取，V1_02 照錄影速度）：1710 組全部配對，
平均 337 對 / 組，3.6 ms / 組（次像素 2.0 ms），最長 9.2 ms；跟兩個 extractor 同時跑。

## 量頻率與延遲

- **輸出頻率、handler 時間**：圖在跑的時候另開一個終端機
  `python3 tools/fw_stats.py --block orb_matcher --interval 10`（讀 fw_agent 的統計，跟編輯器面板同一份；
  不給 `--block` 就列全部 block）。每段時間印出 `match` 的 Hz、`on_pair` 平均時間，有丟資料或 sync 沒配成時也會列出；結束時印整段平均。
- **端到端延遲**：extractor 在開始擷取時記下 steady clock（`orb::Features::extract_start_ns`），matcher 算
  「兩眼中較早開始擷取的那張 → 配對完成」，放在 `orb::Match::stats.latency_ms`，資料結束時印平均 / 中位 / p95 / 最大值與輸出頻率（wall clock）。
  不含 driver 解碼 PNG 和 extractor 前的排隊（後者 fw 統計裡約幾 µs）。
- fw 統計裡輸出的 `age`（ROS 時鐘 − 時間戳記）在這張圖沒有值（`use_sim_time` + bag 時間戳記），所以延遲用上面這個。
- 同一時間只跑一張圖：兩張 EuRoC 圖都會發 `/clock`，會互相干擾（sync 大量配不成）。

EuRoC V1_02（`lossless: true`，每眼 1000 點，GPU 擷取）：輸出 17.8 Hz，延遲平均 49.8 ms、中位 49.1、p95 57.2；
其中配對只佔約 3.6 ms，其餘是兩眼在 GPU 上的擷取。

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
