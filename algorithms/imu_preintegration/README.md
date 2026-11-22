# imu_preintegration

stereo-inertial tracking 用的 IMU 預積分。ORB-SLAM3 在 `Tracking::PreintegrateIMU` 內做這件事；這裡把它移到上游，線性化點固定在 0，tracking 再以 Jacobian 修正到自己估計的 bias（[DESIGN.md](../orb_tracking/docs/DESIGN.md#imu-預積分)）。

## 埠

| 埠 | 型別 | 說明 |
|---|---|---|
| `imu` | `fw::Imu`（trigger，`QUEUE(2000)`） | `euroc_mcap_driver.imu`，IMU 座標系的原始樣本 |
| `frame` | `orb::StereoFrame`（trigger，`QUEUE(16)`） | `orb_matcher.stereo` |
| `calib` | `calib::StereoImuRig`（latest） | IMU 雜訊（連續時間密度）。不需要取樣率，見下 |
| `frame_imu` | `orb::StereoFrame` | 同一個影格，填上 `imu`（`imu::Preintegrated`，從上一影格起算）。時間戳記與影格相同 |

## 行為

- 影格會等到有一筆時間 ≥ 影格時間的樣本才送出。取「上一影格時間或之前的最後一筆」到「這一影格時間或之後的第一筆」之間的樣本，兩端線性內插到影格時間（[interval.hpp](interval.hpp)），積分在 b̄ = 0（[types/imu/preintegrated.hpp](../../types/imu/preintegrated.hpp)）。只看時間戳記，不假設 IMU 頻率或週期。`PreintegrateIMU` 則以固定的 1 ms（`mImuPer`）為容許值選取；影格落在樣本上時（EuRoC 的相機由 IMU 時脈觸發）兩者相同。
- 每一步的 dt 取自樣本的時間戳記，雜訊也以這一步的 dt 離散化：量測 σ²/dt、bias random walk σ_w²·dt（[preintegrated.hpp](../../types/imu/preintegrated.hpp) 的 `Noise`）。ORB-SLAM3 則以設定的 IMU 頻率 f 離散化一次（`IMU::Calib`：σ·√f、σ_w/√f），每一步都用同一組值，連兩端內插出來較短的步也一樣。每步都是 1/f 時兩者相同；EuRoC 的時間戳記剛好每 5 ms 一筆，影格也落在 IMU 樣本上，所以結果與原版一致。
- 第一個影格不帶 `imu`。區間的某一側沒有樣本時也不帶，並回報 degraded。
- 影格時間比上一個早時，丟掉累積的樣本，從這個影格重新起算。
- 資料結束時，還在等待的影格以已有的樣本積分後送出。
- 一次呼叫最多送出一個影格；同時就緒的影格在下一筆樣本到達時送出。資料結束時只送得出第一個還在等待的影格，其餘的計入結束時的統計。

## 健康狀態

狀態改變時回報，同時有多種情況時取最嚴重的：

| 狀態 | 情況 |
|---|---|
| error | IMU 停了：等待中的影格超過 16 個（與 `frame` 的佇列相同）。丟掉最舊的影格（不會不帶 `imu` 送出，那樣會讓 tracking 從上一個 keyframe 起算的預積分少一段） |
| error | IMU 有缺口：相鄰兩筆樣本的間隔超過最近 64 個間隔中位數的 3 倍（樣本遺失，跨過缺口的那一步是內插出來的，預積分是錯的） |
| degraded | 時間倒退（當時等待中的影格被丟掉），或有影格不帶 `imu` 送出 |
| ok | 第一個完全用上述情況之後的樣本積分出來的影格 |

結束時印出的統計包含：缺口數、重來次數與因此丟掉的影格數、等待 IMU 時丟掉的影格數。

硬體正常時不會出現 error，出現就代表感測器或 driver 有問題，之後的位姿不可信。所以 `graphs/slam_job.graph.yaml` 和 `graphs/tracking_job.graph.yaml` 都設了 `on_error: stop_box`：框會把已經進框的資料處理完再停下，job 以結束碼 4 結束。degraded 不會觸發停止。

## 檔案

| 路徑 | 內容 |
|---|---|
| `imu_preintegration.cpp` | fw block |
| `interval.hpp` | 兩影格之間的樣本選取與內插（`PreintegrateIMU`） |
| `../../types/imu/preintegrated.hpp/.cpp` | 預積分：積分、bias 修正、`reintegrate`、`mergePrevious`、`append` |
| `../../tests/types/preintegration_test.cpp` | 與 ORB-SLAM3 的標準答案比對（`tests/fixtures/orb_slam3/preintegration`） |

V1_02_medium 整段：1710 個影格，1709 個帶預積分，每段 10 步，17100 筆樣本，沒有亂序。
