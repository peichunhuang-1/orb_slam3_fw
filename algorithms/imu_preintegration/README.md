# imu_preintegration

stereo-inertial tracking 用的 IMU 預積分。ORB-SLAM3 在 `Tracking::PreintegrateIMU` 內做這件事；這裡把它移到上游，線性化點固定在 0，tracking 再以 Jacobian 修正到自己估計的 bias（[DESIGN.md](../orb_tracking/docs/DESIGN.md#imu-預積分)）。

## 埠

| 埠 | 型別 | 說明 |
|---|---|---|
| `imu` | `fw::Imu`（trigger，`QUEUE(2000)`） | `euroc_mcap_driver.imu`，IMU 座標系的原始樣本 |
| `frame` | `orb::StereoFrame`（trigger，`QUEUE(16)`） | `orb_matcher.stereo` |
| `calib` | `calib::StereoImuRig`（latest） | IMU 雜訊（連續時間密度與取樣率），換算方式與 ORB-SLAM3 的 `IMU::Calib` 相同 |
| `frame_imu` | `orb::StereoFrame` | 同一個影格，填上 `imu`（`imu::Preintegrated`，從上一影格起算）。時間戳記與影格相同 |

## 行為

- 影格會等到有一筆時間 ≥ 影格時間 − 1 ms 的樣本才送出。樣本的選取與兩端內插與 `PreintegrateIMU` 相同（[interval.hpp](interval.hpp)），積分在 b̄ = 0（[types/imu/preintegrated.hpp](../../types/imu/preintegrated.hpp)）。
- 第一個影格不帶 `imu`。兩個影格之間沒有足夠的樣本時也不帶，並回報 degraded。
- 影格時間比上一個早時，丟掉累積的樣本，從這個影格重新起算。
- 資料結束時，還在等待的影格以已有的樣本積分後送出。
- 一次呼叫最多送出一個影格；同時就緒的影格在下一筆樣本到達時送出。

## 檔案

| 路徑 | 內容 |
|---|---|
| `imu_preintegration.cpp` | fw block |
| `interval.hpp` | 兩影格之間的樣本選取與內插（`PreintegrateIMU`） |
| `../../types/imu/preintegrated.hpp/.cpp` | 預積分：積分、bias 修正、`reintegrate`、`mergePrevious`、`append` |
| `../../tests/types/preintegration_test.cpp` | 與 ORB-SLAM3 的標準答案比對（`tests/fixtures/orb_slam3/preintegration`） |

V1_02_medium 整段：1710 個影格，1709 個帶預積分，每段 10 步，17100 筆樣本，沒有亂序。
