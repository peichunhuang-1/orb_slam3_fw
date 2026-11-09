# GPU 量測：提取器的時間花在哪裡（Pi 5，2026-10-27）

## 結論

- **GPU 有 44% 的時間閒置，主因是驅動。** Mesa（v3dv 與 gallium v3d，到 26.2.2 為止）從來沒有設定
  compute dispatch 暫存器 CSD CFG3 的 `MAX_SG_ID`（最大 supergroup ID，bit 20–25）。欄位為 0 時，V3D 7.1
  一次只跑一個 supergroup：workgroup 寬度是 16 的倍數時，一個 supergroup 就是一個 workgroup。
  所以 128 寬的 kernel 最多只用到 12 個 QPU 中的 8 個，64 寬的只用到 4 個。
- 把 `MAX_SG_ID` 設成 15（`tools/mesa/`，只影響設了 `VK_DRIVER_FILES` 的程式）之後，每張影像的 GPU 時間：

  | | 原版驅動 平均 / p99 | MAX_SG_ID 15 平均 / p99 | 關鍵點 |
  |---|---|---|---|
  | EuRoC V1_02（684 張） | 19.9 / 26.3 ms | 12.3 / 14.8 ms（1.62×） | 逐位元相同 |
  | TUM-VI outdoors4（1120 張） | 22.8 / 33.0 ms | 12.7 / 16.2 ms（1.80×） | 逐位元相同 |

- 記憶體不是瓶頸：等記憶體的時間只占 12%，L2 命中率 85–99%。
- 每次 dispatch 的固定開銷約 25–28 µs，每次都是一個 compute 工作加一個 cache 清除工作。每張影像約 42 次 dispatch，
  合計約 1.1 ms（占 6%）。
- 執行緒數：resize、blur、fast_full、fast_cells、harris、nms 每個 QPU 只有 2 個執行緒（上限 4）。
  fast_full 和 harris 用滿 64 個暫存器，harris 有 spill。
- 不用修改過的驅動時，把 list kernel（fast_full、harris、nms、describe、fast_cells）從 64 寬改成 128 寬，
  同樣的工作量約快 2 倍（microbenchmark 的結果，正式程式還沒改）。

### 為什麼 Mesa 沒有設 `MAX_SG_ID`

沒有找到任何說明是刻意的。Mesa 的 commit 訊息裡從來沒出現過 `MAX_SG_ID`。2021 年的 `afc33a743`（Iago Toral，
"v3dv: limit supergroup size in presence of TSY barriers"）刻意把 supergroup 限制在一半的 QPU 執行緒，理由是
「一個 supergroup 卡在 barrier 時，至少還有另一個可以跑」。也就是說，設計時假設多個 supergroup 會同時執行，但這個欄位
從來沒有被設定。Linux 核心（`drivers/gpu/drm/v3d`）定義了這個欄位，但會把 Mesa 給的值原封不動寫進硬體。
Broadcom 的硬體文件沒有公開，所以不能完全排除硬體上的理由。
2020 年 Igalia 的部落格留言串裡，開發者提過「每個 supergroup 只放一個 workgroup 是寫死的，應該可以增加並行度」，
但那說的是 supergroup 的大小，不是同時執行的數量。

## 怎麼量

### 硬體計數器（除錯用，預設不編譯）

library 不讀環境變數（`OrbParams::perfCounters` / `perfEach` / `listPipelines`）；下面的環境變數由
`extract_profile`、`orb_demo` 這兩個工具讀進去。

```bash
cmake -B build -DORB_PERF_COUNTERS=ON && cmake --build build -j2 --target extract_profile
# 每組 kernel 的計數器（一次最多 8 個，必須一個 pass 就量得完；可用名稱用 tools/gpu_probe/list_counters.c 列出）
ORBVK_PERF=cycle-count,compute-active-cycles,QPU-active,QPU-stalls,QPU-stalls-TMU,QPU-wait-bubble \
  build/extract_profile SEQ out.csv --every 10 --kernels k.csv --counters c.csv
# 加上 ORBVK_PERF_EACH=1：改成每一個 dispatch（標籤含 kernel、層級、workgroup 數）
```

沒有 `-DORB_PERF_COUNTERS=ON` 時，計數器相關的程式不會編譯進去，也不會向驅動要求 `VK_KHR_performance_query`。

**判讀方式**（用 `tools/gpu_probe/calib.c` 校正過）：`QPU-active + QPU-stalls + QPU-wait-bubble = 2 × cycle-count`，
2 是**一個 128 寬 supergroup** 的上限，不是整顆 GPU 的上限。

| 項目 | 算法 | 代表 |
|---|---|---|
| 運算 | QPU-active / (2 × cycles) | |
| 等待 | QPU-wait-bubble / (2 × cycles) | 所有執行緒都在等，大多是等 TMU / 記憶體 |
| 停頓 | QPU-stalls / (2 × cycles) | |
| 閒置 | 剩下的部分 | 沒有分到工作 |

用 `tools/gpu_probe/analyze.py` 彙整。

### Shader 的執行緒數與暫存器

```bash
MESA_SHADER_CACHE_DISABLE=true V3D_DEBUG=shaderdb ORB_LIST_PIPELINES=1 build/extract_profile SEQ x.csv --count 1
```

### Microbenchmark（`tools/gpu_probe/`）

| 程式 | 量什麼 |
|---|---|
| `list_counters.c` | 驅動提供的計數器 |
| `calib.c` | 純運算、循序讀、隨機讀三種 shader 的計數器，用來判讀 |
| `e1_dispatch.c` | 每次 dispatch 的固定開銷、workgroup 數與時間的關係 |
| `e1b_parallel.c` / `e1c_wgsize.c` | 同一次 dispatch 的 workgroup 會不會並行（`V3DV_CSD_MAX_SG_ID` 可比較） |
| `e1d_listwidth.c` | list kernel 用 64 / 128 / 256 寬的時間 |
| `compare_sg.py` | 不同驅動下提取器的時間與關鍵點是否相同 |

編譯：`gcc -O2 X.c -o X -lvulkan`（需要 `glslangValidator`）。

## 驅動測試版（`tools/mesa/`）

`build_v3dv.sh` 會抓 Mesa 26.2.2（跟系統版本一樣），套上 `v3dv-csd-max-sg-id.patch`，編譯到
`/home/pi/opt/mesa-v3dv-test`。系統的 Mesa 完全不動。

```bash
VK_DRIVER_FILES=/home/pi/opt/mesa-v3dv-test/share/vulkan/icd.d/broadcom_icd.aarch64.json V3DV_CSD_MAX_SG_ID=15 <程式>
```

不設 `V3DV_CSD_MAX_SG_ID` 時，這個驅動跟原版行為相同。設了之後，程式會在 stderr 印出 `v3dv test patch: CSD MAX_SG_ID = 15`。
