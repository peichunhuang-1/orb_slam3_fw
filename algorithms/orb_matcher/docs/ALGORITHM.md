# orb_matcher 演算法

雙目特徵配對，統一用 Kannala-Brandt（KB8）相機模型，用極線約束找候選點，取代 ORB-SLAM3 原本的做法
（pinhole 用 rectified 掃描列；fisheye 在重疊區用 `BFMatcher` 暴力配對）。目標平台 Pi 5：單執行緒、每幀不配置記憶體、熱路徑 NEON。

原始碼對照：`core/bearing`（第 1 節的 θ, φ, σ）、`core/epipolar_index`（依 θ 排序）、`types/orb/hamming`、
`core/triangulate`（第 2 節）、`core/subpixel`（第 3 節）、`stereo_matcher.cpp`（串起來）、`types/camera/rig_from_calib`（標定 → KB8，第 4 節）。

## 1. 核心想法：在單位球上做 rectification，影像不用 warp

KB 鏡頭的極線在影像上是曲線，但在單位球上是**大圓**，而且所有極平面都通過基線。
所以每個特徵點只要算兩個角度，就等於做好了 rectification：

```
座標：左相機 C1，右相機 C2，T_12 = Stereo.T_c1_c2（C2 在 C1 中的位姿）
      b = t_12（C1 座標下的基線），B = |b|
rect 座標 R_r：e_x = b / B，e_z = 最接近左相機光軸的方向，e_y = e_z × e_x

左點：p_L = R_r · unproject_L(u_L)
右點：p_R = R_r · R_12 · unproject_R(u_R)      // 只旋轉：兩個相機中心都在 e_x 軸上

θ = atan2(p.y, p.z)   極平面繞基線的角度（「掃描列」）
φ = acos(p.x)         跟基線的夾角（「列上的位置」）
```

正確的一對滿足：

| 條件 | 意義 |
|---|---|
| θ_L = θ_R | 在同一個極平面（= rectified 影像的同一列） |
| γ = φ_R − φ_L > 0 | 視差角（= disparity），C2 在 +e_x 方向 |
| r_L = B · sin φ_R / sin γ | 正弦定理，**閉式三角化**，r_L 是 C1 到點的距離 |

深度範圍直接換算成 γ 的範圍：γ_max ≈ B / d_min，γ_min ≈ B · sin φ / d_max（d_max = ∞ 時是 0）。

### 搜尋

1. 右眼所有點算 (θ, φ)，依 θ 排序；描述子照排序後的順序複製成連續 buffer（掃描時 cache 友善）
2. 左眼每個點：二分搜尋 θ ∈ [θ_L − Δ, θ_L + Δ]，在這段裡依序檢查
   - octave：|o_L − o_R| ≤ 1
   - 視差：γ ∈ [γ_min − ε, γ_max + ε]
   - 精確的極線距離：|θ_L − θ_R| · sin φ ≤ k · σ_ang(octave)
   - 以上都過才算 Hamming 距離
3. 留最好和次好：best ≤ `th_dist`（ORB-SLAM3 雙目用 (TH_HIGH + TH_LOW)/2 = 75）
   且 best < `ratio` · second（沿極線的重複紋理靠這個擋；實測後預設 0.7，見 README）
4. 一對一：每個右點只保留 Hamming 最小的左點，其餘的左點變單目點

`σ_ang(octave) = σ_px · scale_factor[octave] / f_local`，`f_local` 用 KB 投影在該點的 Jacobian
（unproject 時順便算），所以影像邊緣（KB 壓縮最大的地方）門檻會自動放寬。

複雜度：排序 O(N log N)，每個左點平均只剩幾個到十幾個候選（估計，要實測），
原版 fisheye 是重疊區 N_L × N_R 全算。

### 極點（epipole）附近

φ 接近 0 或 π（看向基線方向）時 θ 沒有定義、γ 也趨近 0，深度不可靠。
sin φ < sin φ_min 的點直接當單目點，不參與配對。本專案 FOV < 180°，左右並排的雙目極點在
光軸 ±90° 的方向，不會出現在畫面裡，這個檢查只是保險。

---

## 2. 三角化與驗證

1. 閉式三角化 r_L = B · sin φ_R / sin γ，3D 點 = r_L · f_L（C1 座標）
2. r_L > 0、γ ≥ γ_min（視差太小 → 單目點）
3. 3D 點用 KB 投影回兩眼，重投影誤差 χ² < 5.991 · σ²(octave)
   （同 ORB-SLAM3 `KannalaBrandt8::TriangulateMatches` 的檢查）
4. `mvDepth` = 3D 點的 z；fisheye 超過 180° 時 z 可能 ≤ 0，所以有效性看 r_L 不看 z

選用：ORB-SLAM3 `ComputeStereoMatches` 的 median 濾波（Hamming > 1.5·1.4·median 的丟掉）。

---

## 3. 次像素修正（基線 5 cm 時一定要開）

ORB-SLAM3 pinhole 雙目會在 rectified 列上做 11×11 SAD 滑窗 + 拋物線擬合，深度精度明顯比
直接用特徵點座標好（特徵點座標在高層金字塔只有 scale_factor^octave 像素精度）。
KB 版本沿極線曲線做：

1. 在右點所在的大圓上取 ±δ 兩點，KB 投影回右影像、縮到該層 → 局部極線方向 `u`（曲線在 ±5 px 內近似直線）
2. 在 `view[1].pyramid[octave]` 上沿 `x_R + k·u`（k = −L..L）雙線性取樣 11×11 patch，跟左點 patch 算 SAD
3. 拋物線擬合 → 新的右點座標 → 重新 unproject → 新的 φ_R → 重新三角化

估計每對 < 1 ms（約 300 對 × 11 個位置 × 121 像素，NEON）。
有 LIO 之後用光達深度當真值量化精度。

**實作跟上面的差異**，都是 `subpixel_synth_test` / EuRoC 實測決定的：

- 先把右點移到左點的極線上（θ_R := θ_L，φ_R 不變）再沿線搜尋。沒有這一步時，垂直極線的偏差會被斜向紋理
  換成沿線的偏差（孔徑問題），合成資料視差誤差 0.28 → 0.20 px。
- 重投影檢查用偵測到的原始右點，在修正**之前**做；修正後的點已經在極線上，檢查會失效。
- patch 不對齊影像軸，而是對齊各自影像的局部極線方向（局部 rectification）；右眼只取樣一條
  11×21 的 strip，11 個候選位置共用。
- 代價函數用 SSD + 拋物線（`SubpixelFit::SsdParabola`），比 ORB-SLAM3 的 SAD + 拋物線、SAD + V 形擬合都略好。
- 實測每對約 4.8 µs（EuRoC 每幀 0.89 ms），在 CPU 預算表的 0.5–1 ms 內；加速見 TODO。

選項：在**原始 1440×1080** 影像上做修正（等效焦距加倍 → 深度誤差減半）。代價是結果要多帶兩張原圖
（每眼 1.5 MB，複製約 0.3 ms），取樣成本不變（patch 大小相同，座標 ×2）。先在 720 上做，
量過精度之後再決定要不要做這個選項。

---

## 4. 標定與 KB8

框架的標定是 `calib::StereoImuRig`（euroc_mcap_driver 由 CameraInfo + /tf_static 組成）。`rig_from_calib.hpp` 轉成 `StereoRig`：

- `equidistant` / `kannala_brandt`（d = k1..k4）：直接用。
- `plumb_bob` / `radtan`（d = k1 k2 p1 p2 [k3]）或無畸變：保留 fx fy cx cy，在整張影像 4 px 格點上以線性最小平方擬合 k1..k4。
  EuRoC 擬合 rms 約 0.1 px，跟 reference 的 EuRoC 實測（`EuRoC_KB8.yaml`）是同一個擬合。p99 超過 `max_fit_error_px` 時 block 回報 degraded。
- `T_c1_c2 = T_body_left⁻¹ · T_body_right`（右相機在左相機的位姿）。

## 5. 解析度、縮放、短基線

### extractor 不限解析度

`ORBextractor` 依 `CameraPlugin::width()/height()` 建 GPU extractor，任何大小都可以
（GPU 端只要求 row stride 是 4 的倍數，720 沒問題）。720×540 只是 `euroc_bench` 的預設值。
但時間大致跟像素數成正比，1440×1080 直接跑約是 720×540 的 4 倍，所以要先縮放。

### 2× 縮放放在哪裡（依 CPU 成本排序）

| 位置 | CPU 成本 | 備註 |
|---|---|---|
| 感測器 binning / 相機 ISP 直接輸出 720×540 | 0 | 首選。Hikrobot 有 2×2 binning；libcamera 的 ISP 可以縮放，但 GStreamer plugin 註明 CSI 寬度要是 32 的倍數，720 不是（要實測，不行就用 736 或下一列） |
| GPU：extractor 加一個輸入縮放步驟 | 上傳 1.5 MB / 眼（~0.3 ms memcpy） | 相機不能 binning 時用。`resize.comp` 已經有，等於多算一層金字塔；需要在 extractor 參數加 `input_downscale`（1 / 2），`orb::Features` 的座標維持在縮放後的解析度 |
| CPU `cv::resize` | ~1–2 ms / 眼 | 不考慮 |

這一步是 extractor 的事，跟 orb_matcher 無關；換相機的時候再決定，先記在這裡。

### 內參換算（KB8，整數縮放 s = 2）

KB8 的 k1–k4 作用在角度 θ 上、跟像素無關，**不用改**。只有 fx、fy、cx、cy 要換算：

```
fx' = fx / s,   fy' = fy / s
cx' = (cx + 0.5) / s − 0.5,   cy' = (cy + 0.5) / s − 0.5     // 像素中心對齊（2×2 binning / 平均）
```

`cx / s` 跟正確值差 0.25 px，在 5 cm 基線下會變成系統性的深度偏差，所以要用上面的公式。
`StereoRig::resized()` 用這個公式；orb_matcher block 依 `orb::Features` 的 width / height 自動縮放 rig（長寬比不同就丟例外），避免拿 1440 的標定去跑 720 的特徵。

### 基線 5 cm 的深度精度

視差 d_px ≈ f·B / Z，深度誤差 σ_Z ≈ Z² / (f·B) · σ_px。f 在 720 寬度下是多少要看鏡頭，
以 f ≈ 400 px（約 84° HFOV）估：

| Z | 視差 | σ_Z（σ_px = 1，特徵點座標） | σ_Z（σ_px = 0.2，次像素） |
|---|---|---|---|
| 0.5 m | 40 px | 1.3 cm | 0.3 cm |
| 1 m | 20 px | 5 cm | 1 cm |
| 2 m | 10 px | 20 cm | 4 cm |
| 3 m | 6.7 px | 45 cm | 9 cm |
| 5 m | 4 px | 1.25 m | 25 cm |

（第 1 層以上的特徵點 σ_px 還要再乘 1.2^octave。）結論：

- 次像素修正（第 3 節）是必要的，3 m 的上限才站得住；在原圖上做還能再減半。
- 近 / 遠分界 `th_depth` 預設 3 m：近點直接當有深度的雙目點；遠點保留配對，但追蹤時當單目點，
  跟 ORB-SLAM3 `mThDepth` 的處理方式一樣。
- `min_depth` 0.15 m（視差 ~130 px），再近的點兩眼重疊少、外觀差異大，通常配不到。
- 3–5 m 本來就該交給光達：`set_depth_prior()` 用光達深度把視差搜尋範圍縮小，雙目只負責對齊外觀。



