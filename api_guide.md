# fw API 教學：每個巨集、每個設定怎麼用

這份教學假設你**完全沒看過 fw 的設計**，只會寫一般的 C++。每個東西都會講四件事：它是做什麼的、程式怎麼寫、圖檔怎麼接、寫錯會看到什麼。

三份文件的分工：

| 文件 | 內容 |
|---|---|
| 這份 | 每個 API 的用法，從零開始，也可以當字典查 |
| [`usage.md`](usage.md) | 工具操作：裝環境、VS Code 編輯器、除錯、gdb |
| [`orbslam3_tutorial.md`](orbslam3_tutorial.md) | 怎麼把一個真實系統拆成 block |

**巨集的正確名稱：** 沒有 `FW_REGIST` 和 `FW_PARAM` 這兩個名字。註冊 block 用 `FW_REGISTER_ALGORITHM`、`FW_REGISTER_ALGORITHM_CLASS`、`FW_REGISTER_DRIVER`，宣告參數用 `FW_PARAMS`（有 S）。

**內容可信度：** 除非段落明確標示「未實跑」，這裡的程式都在 `fw-humble:0.1.0` 映像裡實際建置、執行過，輸出和錯誤訊息都是實際跑出來的。文件與程式不一致時請回報。

## 我想做的事在哪一節

| 我想… | 看 |
|---|---|
| 先搞懂 block、埠、框是什麼 | [第 0 節](#0-開始之前先搞懂六個名詞) |
| 寫一個「輸入進來、算完、輸出」的函式 | [第 3 節](#3-fw_register_algorithm用函式寫-block) |
| block 要記住上一次的狀態，或有多個輸入各自處理 | [第 4 節](#4-fw_register_algorithm_class用-class-寫-block) |
| 接感測器、自己產生資料 | [第 5 節](#5-fw_register_driver產生資料的-block) |
| 從 YAML 讀參數、執行中調參數 | [第 6 節](#6-fw_params參數) |
| **多個 block 共用同一張地圖、同一份資料** | [第 7 節](#7-共享資源多個-block-共用同一份資料) |
| 讓第三方函式庫自己讀寫一個檔案 | [第 7.11 節](#711-fwfile讓函式庫自己讀寫檔案) |
| 自訂資料型別 | [第 8 節](#8-fw_register_type資料型別) |
| 問別的 block 問題，等它回答 | [第 9 節](#9-calls--serves呼叫另一個-block) |
| 讀檔案當輸入、把結果寫到檔案、記住上次的狀態 | [第 10 節](#10-uri-與-filesystem-plugin) |
| 接現有的 ROS topic、service、TF、重播 bag | [第 11 節](#11-和-ros-接起來) |
| 用內建的開關、比較、合併 block | [第 12 節](#12-內建-block) |
| 查圖檔裡某個鍵的意思和預設值 | [第 13 節](#13-圖檔的每一個鍵) |
| 看到一則錯誤訊息，想知道怎麼改 | [第 14 節](#14-錯誤訊息對照表) |
| 快速查語法 | [第 15 節](#15-速查) |

---

## 0. 開始之前：先搞懂六個名詞

在一般的 ROS 程式裡，你會自己建 node、自己訂閱 topic、自己開執行緒。**fw 把這些全部拿走了。** 你只寫「資料進來要做什麼」，由框架決定什麼時候呼叫你的函式。

| 名詞 | 白話意思 | 比喻 |
|---|---|---|
| **block** | 一個處理單元，通常就是一個 C++ 函式或 class | 工廠裡的一台機器 |
| **埠（port）** | block 的輸入口與輸出口，每個都有名字和型別 | 機器的進料口、出料口 |
| **連線（edge）** | 把一個輸出埠接到一個輸入埠 | 機器之間的傳送帶 |
| **框（box）** | 一個作業系統行程，裡面放一群 block | 一間廠房 |
| **圖（graph）** | 一個 YAML 檔，寫有哪些框、框裡有哪些 block、誰接誰 | 廠房配置圖 |
| **handler（處理函式）** | block 裡被框架呼叫的那個函式 | 機器動作一次 |

一定要記住的三件事：

1. **你不會自己呼叫 handler。** 你只用巨集「登記」它，資料到了框架才會呼叫它。
2. **程式碼只宣告「有什麼埠」，不決定「接到誰」。** 接線全部寫在圖檔。同一個 block 不用改程式就能接到不同地方。
3. **同一個框內，傳資料是傳指標，不複製也不序列化。** 所以同框內幾百 MB 的影像也沒有傳輸成本。跨框才會走 ROS topic。

每次開發都是同樣的四步：

```text
寫或改 .cpp  →  colcon build  →  fw check <圖>  →  fw launch <圖>
                source install/setup.bash
```

三種東西分別放在哪：

| 問題 | 寫在哪 | 改了之後要做什麼 |
|---|---|---|
| block 有什麼埠、怎麼計算 | `.cpp` | 重新 `colcon build` |
| 誰接誰、用幾條執行緒 | `graphs/*.graph.yaml` | 重新 `fw launch` |
| 參數值是多少 | `config/*.yaml` | 不用重啟，存檔就生效 |

---

## 1. 準備練習專案

環境的安裝請先照 [`usage.md` 第 2 節](usage.md#2-準備環境)做完。在專案的上一層資料夾執行：

```bash
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD:/work" -w /work fw-humble:0.1.0 fw new learn
docker run --rm -it --network=host --ipc=host --cap-add=SYS_NICE --cap-add=SYS_PTRACE --ulimit rtprio=99 \
  -u fw -v "$PWD/learn:/workspaces/learn" -w /workspaces/learn fw-humble:0.1.0 bash
```

第二行會進入容器，之後的指令都在容器裡的 `/workspaces/learn` 執行。如果你用 VS Code，用 **Reopen in Container** 開 `learn` 資料夾，效果相同。

刪掉產生的範例，從空白開始：

```bash
rm -rf algorithms/example drivers/example_* graphs/*
```

專案裡的固定資料夾：

| 資料夾 | 放什麼 | 會不會被編譯 |
|---|---|---|
| `algorithms/` | algorithm block | 會，子資料夾自由 |
| `drivers/` | driver block | 會 |
| `types/` | 自訂型別 | 會 |
| `params/` | 參數結構 | 會 |
| `filesystem/plugins/` | 自訂檔案 plugin | 會 |
| `libs/` 或其他 | 其他程式碼 | **不會**，要自己寫 CMake |
| `graphs/` | `*.graph.yaml` | 不編譯，執行時讀 |
| `config/` | 參數 YAML（`config://`） | 不編譯 |

幾條規則，現在先知道就好：

- **新增 `.cpp` 不用改 CMake。** `fw_add_blocks()` 會自動收集上表前五個資料夾裡的 `.cpp`。
- 所有 `.cpp` 編成同一個動態函式庫，所以**函式要放在匿名 `namespace { }` 裡**，不然不同檔案的同名函式會互相衝突。
- **block 的名字在整個專案裡不能重複**，匿名 `namespace` 也救不了，因為名字是註冊時用的字串。重複時建置最後一步會失敗：`fw manifest: ...: duplicate block name: 'double_it'`。
- `#include` 從專案根目錄算，所以寫 `#include "types/reading.hpp"`。
- 每個檔案都要 `#include "fw/fw.hpp"`。

---

## 2. 第一個能跑的例子

目標：一個 driver 產生 1 到 5，一個 algorithm 把數字乘以 2，另一個 algorithm 印出來。

```text
counter.n ──► double_it.x   double_it.y ──► print_int.v
```

建立下面四個檔案。

**`drivers/counter/counter.cpp`**

```cpp
#include <chrono>

#include "fw/fw.hpp"

namespace {
void counter(fw::DriverContext& ctx) {
    for (int k = 1; k <= 5; ++k) {
        if (!ctx.sleep_for(std::chrono::milliseconds(50))) return;  // asked to stop
        ctx.emit("n", k, fw::Time{k * 1000000000LL});               // value k, stamped k seconds
    }
    ctx.eos();  // no more data
}
}  // namespace

FW_REGISTER_DRIVER(counter, OUTPUTS(("n", int)));
```

**`algorithms/double_it/double_it.cpp`**

```cpp
#include "fw/fw.hpp"

namespace {
void double_it(const int& x, int& y) { y = x * 2; }
}  // namespace

FW_REGISTER_ALGORITHM(double_it,
    INPUTS(("x", int, trigger, QUEUE(16, drop_oldest))),
    OUTPUTS(("y", int)));
```

**`algorithms/print/print_int.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"

namespace {
void print_int(const int& v) {
    std::printf("print_int: %d\n", v);
    std::fflush(stdout);
}
}  // namespace

FW_REGISTER_ALGORITHM(print_int, INPUTS(("v", int, trigger, QUEUE(16, drop_oldest))));
```

**`graphs/hello.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      src:    { driver: counter }
      twice:  { algorithm: double_it }
      out:    { algorithm: print_int }
edges:
  - src.n -> twice.x
  - twice.y -> out.v
```

建置並執行：

```bash
colcon build && source install/setup.bash
fw check graphs/hello.graph.yaml
fw launch graphs/hello.graph.yaml
```

你會看到：

```text
graphs/hello.graph.yaml: ok (1 box(es), 3 block(s))
[fw_container_main-1] print_int: 2
[fw_container_main-1] print_int: 4
[fw_container_main-1] print_int: 6
[fw_container_main-1] print_int: 8
[fw_container_main-1] print_int: 10
```

程式會自己結束，因為 `mode: job` 表示資料跑完就結束。

`colcon build` 如果顯示 `1 package had stderr output`，而內容只有 `GLOB mismatch!`，那只是它發現你加了新檔案，不用理它。

### 逐行看懂註冊巨集

```cpp
FW_REGISTER_ALGORITHM(double_it,
    INPUTS(("x", int, trigger, QUEUE(16, drop_oldest))),
    OUTPUTS(("y", int)));
```

| 寫法 | 意思 |
|---|---|
| `FW_REGISTER_ALGORITHM(double_it, ...)` | 把函式 `double_it` 登記成 block，**block 型別的名字就是 `double_it`** |
| `INPUTS(...)` | 這個 block 的輸入埠清單 |
| `("x", int, trigger, ...)` | 一個埠：名字 `"x"`、型別 `int`、`trigger` 表示有資料就執行 |
| `QUEUE(16, drop_oldest)` | 排隊最多 16 筆，滿了丟最舊的 |
| `OUTPUTS(("y", int))` | 一個輸出埠：名字 `"y"`、型別 `int` |

每個埠都用**兩層括號**包住，外層是 `INPUTS(...)`，內層是一個埠 `("x", ...)`。漏一層會編譯失敗。

### 逐行看懂圖檔

```yaml
version: 1            # format version, always 1
ros_distro: humble    # must match the installed ROS
mode: job             # job: stop when the data ends; service (default): run until Ctrl-C
boxes:                # one or more processes
  main:               # the name of this box, your choice
    blocks:
      src:   { driver: counter }       # src is THIS instance's name; counter is the registered name
      twice: { algorithm: double_it }  # the key (algorithm/driver) must match how it was registered
      out:   { algorithm: print_int }
edges:
  - src.n -> twice.x  # <instance>.<output port> -> <instance>.<input port>
  - twice.y -> out.v
```

重點是**實例名稱和型別名稱不一樣**。你可以寫 `left: { algorithm: double_it }` 和 `right: { algorithm: double_it }` 放兩個同型別的 block，它們各自獨立。連線永遠寫實例名稱。

一個輸出要接多個輸入時，用中括號：

```yaml
edges:
  - src.n -> [twice.x, other.x, third.x]
```

反過來不行。**一個輸入埠只能接一條連線**，要合併多個來源請看 [`fw/Merge`](#12-內建-block)。

---

## 3. `FW_REGISTER_ALGORITHM`：用函式寫 block

適用情況：block **不用記憶任何東西**，或者只有**一個觸發來源**。函式形式是最簡單的 block，能用就先用它。

完整格式：

```cpp
FW_REGISTER_ALGORITHM(函式名,
    INPUTS(("埠名", 型別, trigger|latest|sync [, 埠選項...]), ...),   // optional
    RESOURCES(("埠名", 型別, read|write [, NONBLOCKING]), ...),      // optional, section 7
    OUTPUTS(("埠名", 型別 [, STREAM(n)]), ...),                       // optional
    PERIOD(時間 [, catchup]),                                         // optional
    OPTIONS(...));                                                    // optional
```

### 3.1 最重要的一條規則：函式參數的順序

框架**依位置**把埠對應到函式參數，不看你取的參數名稱。順序固定是：

```text
[fw::Context& ctx（選用）]  →  INPUTS 依序  →  RESOURCES 依序  →  OUTPUTS 依序
```

例子：

```cpp
void f(const Reading& in, const double& limit, fw::Write<TempLog> log, double& out);
//     INPUTS 第 1 個      INPUTS 第 2 個        RESOURCES 第 1 個      OUTPUTS 第 1 個
```

順序或型別錯誤都會在**編譯時**報錯，不會跑到一半才出問題：

```text
fw: this parameter is bound to an input port but is an output form (non-const reference); check the parameter order (§4.1)
fw: parameter type differs from the type of the input port it is bound to
fw: the handler's parameter count does not match its ports (§4.1 parameter order)
```

### 3.2 輸入參數有四種寫法

先建立接下來會用到的型別。任何自訂 struct 都要加一行 `FW_REGISTER_TYPE`，第 8 節會再解釋。

**`types/reading.hpp`**

```cpp
#pragma once

#include "fw/fw.hpp"

struct Reading {
    double celsius = 0;
};
FW_REGISTER_TYPE(Reading, "learn/Reading")
```

同一個埠，函式參數可以寫成：

| 寫法 | 什麼時候用 | 特性 |
|---|---|---|
| `const T& x` | **預設就用這個** | 不複製，直接借用，函式返回後不能再用 |
| `Msg<T> x` | 要讀時間戳記，或要存起來之後再用 | 不複製，只增加參考計數，`*x`、`x->`、`x.stamp()` |
| `T x` | 要修改收到的內容 | 框架會先複製一份給你 |
| `fw::Optional<const T&> x` | 只限 `latest` 埠，可能還沒有值 | `if (x)` 判斷，`*x`、`x->` 取值 |

收到的資料**永遠是唯讀的**，因為同一筆資料可能同時有好幾個 block 在用。

### 3.3 輸出參數也有四種寫法

| 寫法 | 意思 |
|---|---|
| `T& out` | **每次一定送出**，函式返回時 `out` 的內容就是輸出 |
| `fw::Optional<T>& out` | 可以不送。沒有賦值就不產生訊息，下游也不會被觸發 |
| `Msg<T>& out` | 自己決定時間戳記，或原封不動轉送收到的訊息。**沒賦值會拋例外** |
| `fw::Optional<Msg<T>>& out` | 同上，但可以不送 |

**`algorithms/forms/forms.cpp`**

```cpp
#include "fw/fw.hpp"
#include "types/reading.hpp"

namespace {

// Always sends
void to_fahrenheit(const Reading& in, double& out) { out = in.celsius * 9.0 / 5.0 + 32.0; }

// Sends only when too hot; otherwise nothing downstream runs
void too_hot(const Reading& in, fw::Optional<double>& alarm) {
    if (in.celsius > 25.0) alarm = in.celsius;
}

// Forwards the same message: no copy, and the timestamp stays
void forward_hot(Msg<Reading> in, fw::Optional<Msg<Reading>>& out) {
    if (in->celsius > 25.0) out = in;
}

// Chooses the output's timestamp
void restamp(const Reading& in, Msg<Reading>& out) { out = fw::make_msg(in, fw::Time{0}); }

// Takes its own copy, so it may change it
void add_one(Reading in, double& out) {
    in.celsius += 1.0;
    out = in.celsius;
}

}  // namespace

FW_REGISTER_ALGORITHM(to_fahrenheit, INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest))), OUTPUTS(("out", double)));
FW_REGISTER_ALGORITHM(too_hot, INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest))), OUTPUTS(("alarm", double)));
FW_REGISTER_ALGORITHM(forward_hot, INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest))), OUTPUTS(("out", Reading)));
FW_REGISTER_ALGORITHM(restamp, INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest))), OUTPUTS(("out", Reading)));
FW_REGISTER_ALGORITHM(add_one, INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest))), OUTPUTS(("out", double)));
```

注意：`OUTPUTS` 裡的型別永遠寫 `T`，不管參數是 `T&` 還是 `fw::Optional<T>&`。`Optional` 和 `Msg` 只是參數的「取用方式」，埠的型別不變。

**時間戳記**：每筆資料都帶一個時間戳記（`fw::Time`，內容是奈秒）。框架用它做 `sync` 配對和統計。你沒特別指定時，輸出會自動繼承觸發它的輸入：

| 觸發來源 | 輸出的時間戳記 |
|---|---|
| `trigger` 埠 | 那筆輸入的時間 |
| 一組 `sync` | 第一個 sync 埠的時間 |
| `PERIOD` / `ON_PERIOD` / `ON_EOS` | 當下的 ROS 時鐘 |
| `CALLS` 回應 / `SERVES` 請求 | 原請求的時間 |
| 用 `Msg<T>` 轉送 | 保留原訊息的時間 |
| `SYNC_TIMEOUT` | 已到資料中最早的時間 |

### 3.4 三種觸發策略：`trigger`、`latest`、`sync`

每個輸入埠都要選一種，意思是「這個埠收到資料時，要不要立刻執行」：

| 策略 | 收到資料時 | 典型用途 |
|---|---|---|
| `trigger` | **立刻排隊執行** | 主要資料流 |
| `latest` | 不執行，只記下最新一筆，等別的東西觸發時順便讀 | 參數、地圖、回饋值 |
| `sync` | 等其他 sync 埠有時間對得上的資料，湊齊才執行 | 雙目相機左右影像 |

函式形式的 block **只能有一個觸發來源**：一個 `trigger` 埠、一組 `sync` 埠或一個 `PERIOD`，三選一。要兩個以上請改用 [class](#4-fw_register_algorithm_class用-class-寫-block)，否則編譯時會看到：

```text
fw: a block with two or more independent trigger sources (trigger ports, a set of sync ports, PERIOD) must use FW_REGISTER_ALGORITHM_CLASS with ON / ON_PERIOD giving each source its own handler (§4.1)
```

一個觸發來源都沒有也不行：

```text
fw: block has no trigger source (trigger / sync port or PERIOD) and would never run
```

#### `latest`：還沒收到資料時怎麼辦

`latest` 埠從來沒收到資料時，用到它的 handler **預設不會執行**，觸發資料會一直等著。程式看起來就像「沒反應」。有兩種處理方式：

**`algorithms/latest_demo/latest_demo.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"
#include "types/reading.hpp"

namespace {

// Way 1: INIT gives the port a value before any message arrives
void with_limit(const Reading& in, const double& limit, fw::Optional<double>& over) {
    if (in.celsius > limit) over = in.celsius;
}

// Way 2: fw::Optional lets the handler run without a value and decide for itself
void with_offset(const Reading& in, fw::Optional<const double&> offset, double& out) {
    out = in.celsius + (offset ? *offset : 0.0);
}

// A latest value is read again on every run. To tell a new value from an old one, compare its timestamp.
struct NewValueOnly {
    fw::Time seen{};
    void on_n(const int& n, fw::Optional<Msg<int>> fix) {
        if (fix && fix->stamp() != seen) {
            seen = fix->stamp();
            std::printf("run %d: new value %d\n", n, **fix);
        } else {
            std::printf("run %d: nothing new\n", n);
        }
        std::fflush(stdout);
    }
};

}  // namespace

FW_REGISTER_ALGORITHM(with_limit,
    INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest)), ("limit", double, latest, INIT(25.0))),
    OUTPUTS(("over", double)));
FW_REGISTER_ALGORITHM(with_offset,
    INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest)), ("offset", double, latest)),
    OUTPUTS(("out", double)));
FW_REGISTER_ALGORITHM_CLASS(NewValueOnly,
    INPUTS(("fix", int, latest)),
    ON(("n", int, trigger, QUEUE(16, drop_oldest)), &NewValueOnly::on_n, USES("fix")));
```

`INIT` 只能寫在 `latest` 埠上，寫在 `trigger` 會編譯失敗：`fw: INIT applies to latest ports`。

`NewValueOnly` 用到了 class 形式和 `USES`，下一節會講，先知道 `latest` 的值每次都讀得到，**要判斷「有沒有更新」得自己比時間戳記**。

#### `sync`：等時間戳記對齊

**`algorithms/sync_demo/sync_demo.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"
#include "types/reading.hpp"

namespace {
void pair_up(Msg<Reading> a, Msg<Reading> b, double& diff) {
    diff = a->celsius - b->celsius;
    std::printf("pair_up: %.1f@%.3f %.1f@%.3f\n", a->celsius, a.stamp().ns / 1e9, b->celsius, b.stamp().ns / 1e9);
    std::fflush(stdout);
}
}  // namespace

FW_REGISTER_ALGORITHM(pair_up,
    INPUTS(("a", Reading, sync), ("b", Reading, sync)),
    OUTPUTS(("diff", double)),
    OPTIONS(SYNC_SLOP(50ms)));
```

配對方式由 `OPTIONS` 決定：

| 選項 | 行為 |
|---|---|
| 不寫 | 近似配對，沒有時間差上限 |
| `SYNC_SLOP(50ms)` | 近似配對，時間差超過 50 ms 就不配 |
| `SYNC_EXACT` | 時間戳記要完全相同才配。與 `SYNC_SLOP` 不能同時寫 |

`50ms` 這種時間字面值可以直接寫，巨集裡已經 `using namespace std::chrono_literals`。

什麼時候**不該**用 `sync`：少一路就整個 block 停擺。如果少一路時還能運作，改用 class 的 `ON` 搭配 `latest`。少一路想特別處理時，請看[第 4.4 節](#44-on_syncsync_timeout配對與配對失敗)的 `SYNC_TIMEOUT`。

如果 block 看起來從來沒被呼叫，而且 log 有「還沒配成過就丟資料」的警告，先檢查兩條資料流的時間戳記是不是同一個時鐘。

### 3.5 埠的選項：`QUEUE`、`STREAM`、`MAX_WAIT`

#### `QUEUE(n, 策略)`：處理不及時怎麼辦

block 同一時間只執行一次。還在算的時候新資料到了，就要排隊。

| 寫法 | 意思 |
|---|---|
| 沒寫 | 深度 1，只留最新一筆（`sync` 埠預設深度 10） |
| `QUEUE(16, drop_oldest)` | 最多 16 筆，滿了丟最舊的 |
| `QUEUE(16, drop_newest)` | 最多 16 筆，滿了丟剛來的 |

**預設深度只有 1，資料是會被丟掉的。** 這是初學者最容易嚇到的地方。下面的 driver 一口氣送 5 筆，接到預設佇列的 block 會漏資料：

**`drivers/burst/burst.cpp`**

```cpp
#include "fw/fw.hpp"

namespace {
void burst(fw::DriverContext& ctx) {
    for (int k = 1; k <= 5; ++k) ctx.emit("n", k, fw::Time{k * 1000LL});
    ctx.eos();
}
}  // namespace

FW_REGISTER_DRIVER(burst, OUTPUTS(("n", int)));
```

**`algorithms/print/print_default_queue.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"

namespace {
void print_default_queue(const int& n) {
    std::printf("got %d\n", n);
    std::fflush(stdout);
}
}  // namespace

FW_REGISTER_ALGORITHM(print_default_queue, INPUTS(("n", int, trigger)));
```

**`graphs/queue.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      src:  { driver: burst }
      out:  { algorithm: print_default_queue }
edges:
  - src.n -> out.n
```

跑起來常見的結果是只有 `got 1` 和 `got 5`，中間三筆被取代了。這不是 bug：感測器資料通常只要最新的。

不想丟資料時，用下一個選項。在圖上也能覆寫深度，不用改程式：

```yaml
out: { algorithm: print_default_queue, queue: { n: 16 } }
out: { algorithm: print_default_queue, queue: { n: { depth: 16, policy: drop_newest } } }
```

丟了多少筆可以在 VS Code 的詳情面板看到，也會計入統計。

#### `STREAM(n)`：這條資料一筆都不能丟

寫在**輸出埠**上，告訴下游「我每一筆都重要」（IMU、輪速計這種要積分的資料）：

```cpp
FW_REGISTER_DRIVER(burst, OUTPUTS(("n", int, STREAM(100))));
```

接到這個輸出的輸入埠若沒指定深度，會自動變成 100。如果你明確把下游設小（例如 `queue: { n: 1 }`），`fw check` 會警告：

```text
warning: input 'o.n' has queue depth 1 but its upstream declares STREAM(100); data that must not be dropped may be dropped (§4.3)
```

#### `MAX_WAIT(t)`：等待時間的警告門檻

這個選項只在一個 block 有**多個 handler** 時才有作用。框架會偵測「某個 handler 跑太久，害另一個埠的資料排隊太久」並警告。

```cpp
ON(("imu", Imu, trigger), &F::on_imu, MAX_WAIT(20ms))        // warn beyond 20 ms
ON(("log", Log, trigger), &F::on_log, MAX_WAIT(fw::unbounded)) // never warn on this port
OPTIONS(MAX_WAIT(fw::unbounded))                             // never warn on this block
```

圖上也能覆寫：`max_wait: { imu: 50ms }` 或 `max_wait: { imu: unbounded }`。

### 3.6 `PERIOD`：定時執行

沒有觸發輸入，每隔一段時間執行一次：

**`algorithms/heartbeat/heartbeat.cpp`**

```cpp
#include "fw/fw.hpp"

namespace {
void heartbeat(int& out) {
    static int n = 0;
    out = ++n;
}
}  // namespace

FW_REGISTER_ALGORITHM(heartbeat, PERIOD(500ms), OUTPUTS(("n", int)));
```

**`graphs/period.graph.yaml`**

```yaml
version: 1
ros_distro: humble
boxes:
  main:
    blocks:
      beat: { algorithm: heartbeat }
      out:  { algorithm: print_int }
edges:
  - beat.n -> out.v
```

這張圖沒有 `mode: job`，會跑到你按 Ctrl-C。

- `PERIOD(500ms)` 依 ROS 時鐘計時，也支援重播時的模擬時間。
- 來不及時會合併，只執行一次。寫 `PERIOD(500ms, catchup)` 才會把錯過的逐次補回來。
- 定時 block 的輸入埠通常全部是 `latest`。

### 3.7 `OPTIONS`：block 層級的選項

| 選項 | 意思 |
|---|---|
| `SYNC_EXACT`、`SYNC_SLOP(t)` | sync 配對方式，見第 3.4 節 |
| `MAX_WAIT(t)` | 等待警告門檻，見第 3.5 節 |
| `THREADS(n)` | block 可以用的執行緒總數（含執行它的那條），見下 |
| `FW_REENTRANT` | 允許同一個 block 同時執行多次，只有你確定函式可以並行才用。只能用在單一 handler 的 block |
| `RESTART_PROCESS` | 宣告這個 block 無法在行程內乾淨重啟，要重啟就重啟整個行程（有全域狀態的第三方函式庫才需要） |

### 3.8 `fw::Context`：handler 裡的工具箱

需要時才加，一定放在**第一個參數**。它不對應任何埠。

**`algorithms/ctx_demo/ctx_demo.cpp`**

```cpp
#include <atomic>

#include "fw/fw.hpp"
#include "types/reading.hpp"

namespace {
void ctx_demo(fw::Context& ctx, const Reading& in, double& out) {
    if (in.celsius > 28.0) ctx.report(fw::Health::degraded, "too hot");  // shows up in /diagnostics and the editor

    std::atomic<int> sum{0};
    fw::parallel_for(ctx, 0, 100, [&](std::size_t i) { sum += static_cast<int>(i); });  // uses the THREADS budget
    out = in.celsius + sum.load();
}
}  // namespace

FW_REGISTER_ALGORITHM(ctx_demo,
    INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest))),
    OUTPUTS(("out", double)),
    OPTIONS(THREADS(2)));
```

| 成員 | 用途 |
|---|---|
| `ctx.report(fw::Health::ok / degraded / error, "訊息")` | 回報健康狀態 |
| `ctx.threads()` | 這個 block 的執行緒預算（圖上可用 `threads: 2` 覆寫） |
| `fw::parallel_for(ctx, begin, end, fn)` | 在預算內平行跑 `fn(i)`，**建議用這個，不要自己開執行緒** |
| `ctx.call("埠", 請求)` | 呼叫另一個 block，見第 9 節 |

`parallel_for` 的迴圈本體裡**不能**用 `ctx.report`、`ctx.call`。

### 3.9 handler 拋例外會怎樣

框架會接住例外，**跳過這一筆**，把 block 標成 degraded，然後繼續處理下一筆：

```text
[ERROR] [main]: block 'b' handler 'boom' threw: boom at 2
[WARN] [main]: block 'b' reports degraded (exceptions): boom threw: boom at 2
```

不會讓行程崩潰。但段錯誤（segfault）接不住，那會讓整個框崩潰，處理方式見[第 13.2 節](#132-框boxes名字)的 `on_crash`。

框架**不會**因為例外次數多就判定 block 壞掉。如果某種例外代表 block 真的壞了，請自己 `catch` 之後呼叫 `ctx.report(fw::Health::error, "原因")`，並在圖上設 `on_error`，見[第 13.3 節](#133-block-實例blocks名字)。

---

## 4. `FW_REGISTER_ALGORITHM_CLASS`：用 class 寫 block

需要 class 的兩種情況：

1. block 要**記住東西**（累積平均、濾波器狀態）。
2. block 有**兩個以上觸發來源**，每個要做不同的事。

**不用擔心執行緒安全。** 同一個 block 實例的所有 handler 絕對不會同時執行，成員變數不需要鎖。

### 4.1 只有一個 handler：`operator()`

和函式形式完全一樣，只是多了成員變數：

**`algorithms/averager/averager.cpp`**

```cpp
#include "fw/fw.hpp"
#include "types/reading.hpp"

namespace {
struct Averager {
    double sum = 0;
    int n = 0;
    void operator()(const Reading& in, double& mean) {
        sum += in.celsius;
        ++n;
        mean = sum / n;
    }
};
}  // namespace

FW_REGISTER_ALGORITHM_CLASS(Averager,
    INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest))),
    OUTPUTS(("mean", double)));
```

圖上寫 `avg: { algorithm: Averager }`。block 名字就是 class 名字。

### 4.2 多個 handler：`ON`、`USES`、`EMITS`

有多個 handler 時，每個 handler 都要自己宣告**要用哪些東西**：

| 子句 | 意思 |
|---|---|
| `ON((埠定義), &Cls::fn, ...)` | 這個 trigger 埠的資料交給 `fn` 處理 |
| `USES("a", "b")` | `fn` 要讀的 `latest` 埠、要用的資源、要呼叫的呼叫埠 |
| `EMITS("x", "y")` | `fn` 可能送出的輸出 |

**沒列出來的就不能用。** 這樣框架才知道哪個 handler 依賴什麼，能精準做檢查。

**`algorithms/fusion/fusion.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"
#include "types/reading.hpp"

namespace {
struct Fusion {
    double last_a = 0;
    double last_b = 0;

    // a arrives: read weight (USES), send out (EMITS)
    void on_a(const Reading& a, const double& weight, double& out) {
        last_a = a.celsius;
        out = weight * last_a + (1 - weight) * last_b;
    }
    // b arrives: only remember it
    void on_b(const Reading& b) { last_b = b.celsius; }
    // Every 500 ms: may send
    void on_tick(fw::Optional<double>& out) {
        if (last_a != 0) out = last_a;
    }

    void on_configure() { say("configure"); }
    void on_activate() { say("activate"); }
    void on_deactivate() { say("deactivate"); }
    void on_cleanup() { say("cleanup"); }
    void on_eos(double& out) {
        say("end of data");
        out = last_a;
    }

    static void say(const char* what) {
        std::printf("Fusion: %s\n", what);
        std::fflush(stdout);
    }
};
}  // namespace

FW_REGISTER_ALGORITHM_CLASS(Fusion,
    INPUTS(("weight", double, latest, INIT(0.5))),
    OUTPUTS(("out", double)),
    ON(("a", Reading, trigger, QUEUE(16, drop_oldest)), &Fusion::on_a, USES("weight"), EMITS("out")),
    ON(("b", Reading, trigger, QUEUE(16, drop_oldest)), &Fusion::on_b),
    ON_PERIOD(500ms, &Fusion::on_tick, EMITS("out")),
    ON_CONFIGURE(&Fusion::on_configure),
    ON_ACTIVATE(&Fusion::on_activate),
    ON_DEACTIVATE(&Fusion::on_deactivate),
    ON_CLEANUP(&Fusion::on_cleanup),
    ON_EOS(&Fusion::on_eos, EMITS("out")));
```

關鍵差異，容易寫錯：

- `INPUTS` **只能放 `latest` 埠**。`trigger` 埠寫在 `ON(...)` 裡面，`sync` 埠寫在 `ON_SYNC(...)` 裡面。
- 宣告了卻沒人用的 `latest` 埠、沒人 `EMITS` 的輸出，都是編譯錯誤。

### 4.3 handler 的參數順序表

和函式形式一樣依位置對應。每種 handler 的參數順序如下，第一個參數都可以再加 `fw::Context& ctx`：

| handler | 參數順序 |
|---|---|
| `ON` | 觸發輸入 → `USES` 依序 → `EMITS` 依序 |
| `ON_SYNC` | 各 sync 輸入依序 → `USES` → `EMITS` |
| `SYNC_TIMEOUT` | 各 sync 埠的 `fw::Optional<const T&>` → `USES` → `EMITS` |
| `ON_PERIOD` | `USES` → `EMITS` |
| `ON_EOS` | `USES` → `EMITS` |
| `CALLS` 的回應 | `const fw::Reply<Srv>&` → `USES` → `EMITS` |
| `SERVES` | `const Srv::Request&` → `fw::Responder<Srv>` → `USES` → `EMITS` |
| `ON_CONFIGURE` 等四個生命週期 | `USES` |

`USES` 列的順序就是參數的順序。`USES` 裡的呼叫埠只授權，**不產生參數**。

用 `Fusion::on_a` 對照一次：

```cpp
ON(("a", Reading, trigger, ...), &Fusion::on_a, USES("weight"), EMITS("out"))
void on_a(const Reading& a,   const double& weight,   double& out)
//        觸發輸入              USES 第 1 個            EMITS 第 1 個
```

### 4.4 `ON_SYNC`、`SYNC_TIMEOUT`：配對與配對失敗

**`algorithms/pair_block/pair_block.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"
#include "types/reading.hpp"

namespace {
struct PairBlock {
    void on_pair(const Reading& a, const Reading& b, double& out) { out = a.celsius + b.celsius; }

    // Something waited 200 ms without a partner: one of the two is empty
    void on_lonely(fw::Optional<const Reading&> a, fw::Optional<const Reading&> b, fw::Optional<double>& out) {
        std::printf("lonely: a=%d b=%d\n", a.has_value(), b.has_value());
        std::fflush(stdout);
        if (a) out = a->celsius;
    }
};
}  // namespace

FW_REGISTER_ALGORITHM_CLASS(PairBlock,
    OUTPUTS(("out", double)),
    ON_SYNC((("a", Reading), ("b", Reading)), &PairBlock::on_pair, EMITS("out")),
    SYNC_TIMEOUT(200ms, &PairBlock::on_lonely, EMITS("out")),
    OPTIONS(SYNC_EXACT));
```

注意 `ON_SYNC` 的埠不用寫 `sync`，也注意多一層括號：`ON_SYNC((("a", T), ("b", T)), ...)`。一個 block 最多一組 sync。

### 4.5 `ON_PERIOD`：和資料處理並存的定時器

```cpp
ON_PERIOD(33ms, &Ekf::on_timer, USES("params"), EMITS("pose"))
ON_PERIOD(33ms, &Ekf::on_timer, catchup, EMITS("pose"))         // run every missed tick
```

常見用法：`ON` 只把資料存進成員變數，由 `ON_PERIOD` 定時計算輸出。

### 4.6 生命週期回呼

有寫才會被呼叫，框架**不看函式名稱**，一定要用巨集宣告。

| 回呼 | 什麼時候呼叫 | 通常做什麼 |
|---|---|---|
| 建構子 | 最先 | 盡量不做事，留給 `ON_CONFIGURE` |
| `ON_CONFIGURE` | 開始前，只有一次 | 載入模型、開裝置、配置記憶體 |
| `ON_ACTIVATE` | 資料開始流動前 | 重設狀態 |
| 各 handler | 執行中 | 工作 |
| `ON_EOS` | **上游的資料全部結束**後 | 存檔、輸出最終結果。可以 `EMITS` |
| `ON_DEACTIVATE` | 停止時 | 暫停裝置 |
| `ON_CLEANUP` | 最後 | 釋放資源 |

實際跑 `Fusion` 看到的順序：

```text
Fusion: configure
Fusion: activate
Fusion: end of data
Fusion: deactivate
Fusion: cleanup
```

要注意的地方：

- 生命週期回呼可以 `USES` 資源和 `latest` 埠，但資料還沒開始流動，所以 `latest` 埠必須接「保證有初值」的來源（如 `config://`），或宣告成 `fw::Optional`，否則：

  ```text
  error: block 'l': ON_CONFIGURE (on_configure) uses latest input 'v', which has no value then: data does not flow yet; connect it to a guaranteed source (config://... or a plugin output declared GUARANTEED), or take it as fw::Optional
  ```

- `ON_EOS` **只有在資料真的結束時才會被呼叫**。接到 ROS topic 或永不結束的感測器時，它永遠不會執行，存檔請改放 `ON_CLEANUP`，或用另一張 job 圖。按 Ctrl-C 只會觸發 `ON_DEACTIVATE`、`ON_CLEANUP`。

---

## 5. `FW_REGISTER_DRIVER`：產生資料的 block

algorithm 是被動的，資料到了才被呼叫。driver 相反：**框架給它一條專屬執行緒，它自己跑迴圈、自己決定什麼時候送資料。** 感測器都寫成 driver。

driver 的函式簽名只有一種，寫錯會編譯失敗：

```cpp
void 名字(fw::DriverContext& ctx);
```

先宣告參數（第 6 節會詳細講）：

**`params/thermometer_params.hpp`**

```cpp
#pragma once

#include "fw/fw.hpp"

struct ThermometerParams {
    int period_ms = 100;
    int count = 10;
    double start = 20;
};
FW_PARAMS(ThermometerParams,
    FIELD(period_ms, DEFAULT(100), RANGE(1, 10000), DESC("time between readings")),
    FIELD(count, DEFAULT(10), RANGE(1, 1000000), DESC("readings before the end of the data")),
    FIELD(start, DEFAULT(20), DESC("first temperature")))
```

**`drivers/thermometer/thermometer.cpp`**

```cpp
#include <chrono>

#include "fw/fw.hpp"
#include "params/thermometer_params.hpp"
#include "types/reading.hpp"

namespace {
void thermometer(fw::DriverContext& ctx) {
    fw::Time stamp{1000000000};
    for (int k = 0;; ++k) {
        const auto p = ctx.params<ThermometerParams>();  // the current values; call again for newer ones
        if (k >= p->count) break;
        if (!ctx.sleep_for(std::chrono::milliseconds(p->period_ms))) return;  // false: asked to stop
        ctx.emit("reading", Reading{p->start + k}, stamp);
        stamp.ns += std::int64_t{p->period_ms} * 1000000;
    }
    ctx.eos();
}
}  // namespace

FW_REGISTER_DRIVER(thermometer,
    PARAMS(ThermometerParams),
    OUTPUTS(("reading", Reading)));
```

**`config/thermo.yaml`**

```yaml
period_ms: 50
count: 5
start: 24
```

**`algorithms/print/print_double.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"

namespace {
void print_double(Msg<double> v) {
    std::printf("print_double: %.1f at %.3f s\n", *v, v.stamp().ns / 1e9);
    std::fflush(stdout);
}
}  // namespace

FW_REGISTER_ALGORITHM(print_double, INPUTS(("v", double, trigger, QUEUE(32, drop_oldest))));
```

**`graphs/thermo.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      thermo: { driver: thermometer }
      conv:   { algorithm: to_fahrenheit }
      out:    { algorithm: print_double }
edges:
  - config://thermo.yaml -> thermo.params
  - thermo.reading -> conv.in
  - conv.out -> out.v
```

輸出：

```text
print_double: 75.2 at 1.000 s
print_double: 77.0 at 1.050 s
print_double: 78.8 at 1.100 s
print_double: 80.6 at 1.150 s
print_double: 82.4 at 1.200 s
```

`fw::DriverContext` 的成員：

| 成員 | 用途 |
|---|---|
| `ctx.ok()` | 還能繼續跑嗎。`false` 表示要停了，迴圈必須結束 |
| `ctx.sleep_for(時間)` | 睡覺，被要求停止時會提早醒來。回傳 `ok()`。**不要用 `std::this_thread::sleep_for`**，那樣會卡住停止流程 |
| `ctx.emit("埠", 值, 時間戳記)` | 送出新資料，時間戳記是必填的 |
| `ctx.emit("埠", Msg<T>)` | 送出現成的訊息 |
| `ctx.eos()` | 資料送完了。之後不能再 `emit` |
| `ctx.params<T>()` | 取得參數，回傳 `Msg<T>`，用 `p->欄位` 讀 |

**時間戳記要用真實時間時**（接真感測器）：

```cpp
fw::SystemClock clock;
ctx.emit("reading", Reading{value}, clock.now());
```

有硬體時間就用硬體時間。

**driver 怎麼結束，規則很嚴格：**

| 結束方式 | 結果 |
|---|---|
| 呼叫 `ctx.eos()` 然後 return | 正常結束。下游會收到資料結束的通知，`ON_EOS` 會被呼叫 |
| `ctx.sleep_for` 或 `ctx.ok()` 為 false 之後 return | 正常（被要求停止） |
| 沒呼叫 `ctx.eos()` 就 return | **錯誤**，job 模式結束碼 4 |
| 拋例外 | 錯誤，依 `on_error` 處理 |

忘記 `ctx.eos()` 的訊息：

```text
error: block 'n' ended in error: run() returned while the box is active without ctx.eos(); a finite driver ends with ctx.eos(), otherwise this is an error (§4.7)
```

**不會結束的感測器**長這樣：

**`drivers/ticker/ticker.cpp`**

```cpp
#include <chrono>

#include "fw/fw.hpp"

namespace {
void ticker(fw::DriverContext& ctx) {
    for (int k = 1; ctx.sleep_for(std::chrono::milliseconds(300)); ++k)
        ctx.emit("n", k, fw::Time{k * 300000000LL});
}
}  // namespace

FW_REGISTER_DRIVER(ticker, OUTPUTS(("n", int)));
```

**接硬體時特別注意：** 呼叫 SDK 的 `read()` 一定要有逾時（約 100 ms）。如果永遠卡在 `read()`，driver 看不到 `ctx.ok()` 變 false，整個框就停不下來。

**driver 只有輸出，沒有其他輸入埠**（除了 `PARAMS` 帶來的 `params`）。

---

## 6. `FW_PARAMS`：參數

fw **不用 ROS 參數機制**。參數就是 YAML 檔，透過一條普通的連線傳給 block。完整流程只有四步：

```text
① struct + FW_PARAMS        宣告有哪些參數、合法範圍
② block 開一個 latest 埠    型別就是這個 struct（driver 用 PARAMS(T)）
③ config/xxx.yaml           寫值
④ 圖上連線                   config://xxx.yaml -> block.埠
```

### 6.1 完整範例

**`params/controller_params.hpp`**

```cpp
#pragma once

#include <array>
#include <string>
#include <vector>

#include "fw/fw.hpp"

struct PidGains {
    double kp = 0;
    double ki = 0;
};
FW_PARAMS(PidGains,
    FIELD(kp, RANGE(0, 100)),
    FIELD(ki, DEFAULT(0.0), RANGE(0, 100)))

struct ControllerParams {
    std::string mode;
    int max_speed = 0;
    PidGains left;
    PidGains right;
    bool verbose = false;
    std::array<double, 3> offset{};
    std::vector<double> limits;
    double internal_scale = 1;
};
FW_PARAMS(ControllerParams,
    GROUP("motion", PERSIST(ask),
          FIELD(mode, DESC("slow or fast")),
          FIELD(max_speed, RANGE(0, 500))),
    GROUP("gains", PERSIST(ask),
          FIELD(left),
          FIELD(right)),
    GROUP("debug", PERSIST(never),
          FIELD(verbose, DEFAULT(false), DESC("print every step"))),
    FIELD(offset, DEFAULT(std::array<double, 3>{0, 0, 0})),
    FIELD(limits, DEFAULT(std::vector<double>{1, 2}), RANGE(0, 10)),
    FIELD(internal_scale, KEY("scale"), DEFAULT(1.0)))
```

**`config/ctrl.yaml`**

```yaml
mode: slow       # comments are kept when fw writes the file
max_speed: 100
left: { kp: 1.5 }
right:
  kp: 2.5
  ki: 0.5
offset: [1, 2, 3]
limits: [1, 2, 3]
scale: 2
```

**`algorithms/controller/controller.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"
#include "params/controller_params.hpp"

namespace {
void controller(const int& n, const ControllerParams& p) {
    std::printf("run %d: mode=%s max=%d left.kp=%.1f right.ki=%.1f offset[0]=%.1f limits=%zu scale=%.1f verbose=%d\n",
                n, p.mode.c_str(), p.max_speed, p.left.kp, p.right.ki, p.offset[0], p.limits.size(), p.internal_scale,
                p.verbose);
    std::fflush(stdout);
}
}  // namespace

FW_REGISTER_ALGORITHM(controller,
    INPUTS(("n", int, trigger, QUEUE(32, drop_oldest)), ("params", ControllerParams, latest)));
```

**`graphs/params.graph.yaml`**

```yaml
version: 1
ros_distro: humble
boxes:
  main:
    blocks:
      src:  { driver: ticker }
      ctrl: { algorithm: controller }
edges:
  - src.n -> ctrl.n
  - config://ctrl.yaml -> ctrl.params
```

輸出：

```text
run 1: mode=slow max=100 left.kp=1.5 right.ki=0.5 offset[0]=1.0 limits=3 scale=2.0 verbose=0
```

### 6.2 `FIELD` 的所有選項

```cpp
FIELD(成員名 [, DEFAULT(值)] [, RANGE(下限, 上限)] [, DESC("說明")] [, KEY("yaml_key")])
```

| 選項 | 意思 |
|---|---|
| 不寫任何選項 | **YAML 裡一定要有這個欄位**，沒有就是錯誤 |
| `DEFAULT(v)` | YAML 可以不寫，沒寫就用 `v` |
| `RANGE(a, b)` | 只能是數值，超出範圍是錯誤。用在 `array` 或 `vector` 時每個元素都檢查 |
| `DESC("...")` | 說明文字，給編輯器顯示 |
| `KEY("name")` | YAML 裡的名字和 C++ 成員不同時使用，範例的 `internal_scale` 在 YAML 寫成 `scale` |

**struct 成員的 `= 100` 不算預設值。** 只有 `DEFAULT(100)` 才算。沒寫 `DEFAULT` 時，就算 C++ 有初值，YAML 也必須提供。

可以用的欄位型別：

| 型別 | YAML 寫法 |
|---|---|
| `int`、`double`、`float` 等數值 | `max_speed: 100` |
| `bool` | `verbose: true` |
| `std::string` | `mode: slow` |
| `std::array<T, N>` | `offset: [1, 2, 3]`，長度要對 |
| `std::vector<T>` | `limits: [1, 2, 3]` |
| 另一個 `FW_PARAMS` 結構 | `left: { kp: 1.5 }` |

其他型別（例如 `std::map`）會編譯失敗，把巢狀結構改成 `FW_PARAMS` 結構或清單即可：

```text
fw: a FW_PARAMS field is a number, bool, std::string, std::array, std::vector or another FW_PARAMS structure (§4.8)
```

### 6.3 `GROUP`、`PERSIST`、`APPLY`

`GROUP` 把欄位分組。**組是存檔的最小單位**，一組要一起存或一起放棄。例如外參的旋轉與平移應該放同一組，避免只存到一半。

```cpp
GROUP("組名", PERSIST(ask 或 never) [, APPLY(immediate)], FIELD(...), FIELD(...))
```

| 設定 | 意思 |
|---|---|
| 不在 `GROUP` 裡的欄位 | 屬於預設群組，`fw params diff` 顯示為 `(default)`，等同 `PERSIST(ask)` |
| `PERSIST(ask)` | 執行中的修改要你確認才會寫回檔案 |
| `PERSIST(never)` | 只在這次執行有效，永遠不寫回（除錯開關） |
| `APPLY(immediate)` | 演算法自己提出的新值不等確認就生效（見第 6.7 節） |

巢狀結構本身的 `GROUP` 不生效，它的欄位屬於「持有它的那個欄位」所在的組。

### 6.4 必須遵守的兩個位置規則

**`FW_PARAMS` 只能寫在全域範圍，不能放在 `namespace` 裡面。** struct 本身可以在 namespace 內：

```cpp
namespace my { struct Gains { double k = 1; }; }
FW_PARAMS(my::Gains, FIELD(k, DEFAULT(1.0)))    // correct: outside the namespace, with the full name
```

放進 namespace 的錯誤訊息不直覺，看到這個就是這個問題：

```text
error: explicit specialization of non-template 'n::fw::TypeName'
```

**在 algorithm 裡，參數埠就是普通的 `latest` 埠**，名字自己取。只有 driver 和 plugin 用 `PARAMS(T)`，埠名固定是 `params`。在 algorithm 寫 `PARAMS(T)` 會編譯失敗：

```text
fw: PARAMS is the drivers' parameter port; an algorithm takes parameters through a latest input of the FW_PARAMS type (§4.8)
```

### 6.5 其他接法

**一個 YAML 給多個 block**（每個 block 都拿到同一份值）：

```yaml
- config://ctrl.yaml -> [a.params, b.params]
```

**只接檔案的一部分**：`#/` 後面是 YAML 路徑。下面的 `#/right` 只取 `right:` 這段，所以 block 的參數埠型別是 `PidGains`，不是 `ControllerParams`：

```yaml
- config://ctrl.yaml#/right -> pid.gains
```

**同一個 URI 只能是一種型別。** 要接不同結構時就要用片段，否則：

```text
error: connection 'config://ctrl.yaml -> c.gains': config://ctrl.yaml already feeds ControllerParams, and 'c.gains' takes PidGains; one URI is one value (use a fragment, #/part, for a part of the file)
```

**完全不接**：所有欄位都有 `DEFAULT` 時可以不接，值全部用預設。只要有一個欄位沒有 `DEFAULT` 就是錯誤：

```text
error: parameter port 'c.params' is not connected, and ControllerParams has fields without DEFAULT (mode, max_speed, left, right); connect a source (config://...)
```

**子資料夾**：`config://drivers/cam.yaml` 對應 `config/drivers/cam.yaml`。

### 6.6 在 `ON_CONFIGURE` 用參數，和熱更新的陷阱

下面這個 block 在啟動時讀一次參數，執行中也讀：

**`algorithms/model_loader/model_loader.cpp`**

```cpp
#include <cstdio>
#include <string>

#include "fw/fw.hpp"
#include "params/controller_params.hpp"

namespace {
struct ModelLoader {
    std::string loaded;
    void on_configure(const ControllerParams& p) { loaded = p.mode; }
    void on_n(const int&, const ControllerParams& p) {
        std::printf("loaded at start: %s, now: %s\n", loaded.c_str(), p.mode.c_str());
        std::fflush(stdout);
    }
};
}  // namespace

FW_REGISTER_ALGORITHM_CLASS(ModelLoader,
    INPUTS(("params", ControllerParams, latest)),
    ON(("n", int, trigger), &ModelLoader::on_n, USES("params")),
    ON_CONFIGURE(&ModelLoader::on_configure, USES("params")));
```

改了 YAML 之後，`now` 會變，`loaded at start` **不會變**，因為 `ON_CONFIGURE` 不會重跑。這是熱更新最常見的誤解：**只在啟動時用到的參數（模型路徑、裝置名稱），要重新啟動才生效。**

**handler 不會在執行中途看到參數改變**，新值要等到下一次呼叫。driver 則是下一次呼叫 `ctx.params<T>()` 才拿到。

### 6.7 執行中修改參數

圖在跑時，另開一個終端（也要先 `source install/setup.bash`）。

**方法 1：直接編輯 `config/ctrl.yaml` 存檔。** 立刻生效。檔案內容有錯時保留舊值，並把該來源標成 degraded，修好就恢復。

**方法 2：用 `fw params`，先試再決定要不要存。**

```bash
fw params set 'config://ctrl.yaml#/mode' fast    # takes effect now; the file is unchanged
fw params diff                                   # what differs from the file
fw params commit                                 # write everything to the file
fw params commit --group motion                  # or only one group
fw params discard                                # or throw the changes away
```

實際輸出：

```text
$ fw params set 'config://ctrl.yaml#/mode' fast
config://ctrl.yaml#/mode = fast: applied, pending (group 'motion'); fw params commit writes it to the file

$ fw params set 'config://ctrl.yaml#/max_speed' 900
fw params set: box 'main': /workspaces/learn/config/ctrl.yaml:1:1: /max_speed: 900 is outside RANGE(0, 500)

$ fw params set 'config://ctrl.yaml#/verbose' true
config://ctrl.yaml#/verbose = true: applied, pending (group 'debug'); PERSIST(never): never written to the file

$ fw params diff
config/ctrl.yaml group 'motion'
  /mode: slow -> fast
config/ctrl.yaml group 'debug'
  /verbose: (not in the file) -> true   (PERSIST(never): not saved)

$ fw params commit --group motion
written: config/ctrl.yaml /mode = fast
```

`set` 時 URI 要用單引號包起來，因為 `#` 在 shell 裡有特殊意義。寫回檔案時會保留註解，舊檔備份在 `.fw/history/`，還沒確認的修改在 `.fw/pending/`。重新啟動時只讀 YAML，暫存的修改不會自動套用。

### 6.8 演算法提出新參數（線上標定）

演算法算出新的參數值時，把輸出接到 `config://`。**這個值不會立刻生效，要人確認。**

**`algorithms/auto_tune/auto_tune.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"

struct TuneGain {
    double k = 1;
};
FW_PARAMS(TuneGain, FIELD(k, DEFAULT(1.0), RANGE(0, 100)))

namespace {
void auto_tune(const int& n, const TuneGain& now, fw::Optional<TuneGain>& proposal) {
    std::printf("run %d: k=%.1f\n", n, now.k);
    std::fflush(stdout);
    if (n == 3) {
        TuneGain g;
        g.k = 42;
        proposal = g;
    }
}
}  // namespace

FW_REGISTER_ALGORITHM(auto_tune,
    INPUTS(("n", int, trigger, QUEUE(16, drop_oldest)), ("gain", TuneGain, latest)),
    OUTPUTS(("proposal", TuneGain)));
```

**`config/gain.yaml`**

```yaml
k: 1
```

**`graphs/propose.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      src:  { driver: counter }
      tune: { algorithm: auto_tune }
edges:
  - src.n -> tune.n
  - config://gain.yaml -> tune.gain
  - tune.proposal -> config://gain.yaml
```

執行完，`k` 一直是 `1.0`，提案等你審核：

```text
$ fw params diff
config/gain.yaml group '(default)'
  /k: 1 -> 42   (proposed by an algorithm: takes effect when committed)
$ fw params commit
written: config/gain.yaml /k = 42
```

群組宣告 `APPLY(immediate)` 才會跳過確認。

### 6.9 參數錯誤對照

都是 `fw check` 就能抓到的：

| 訊息 | 意思 |
|---|---|
| `config/x.yaml:2:1: unknown key 'max_spee' in ControllerParams (did you mean 'max_speed'?)` | 打錯字 |
| `config/x.yaml:1:1: missing 'max_speed' (int32, no DEFAULT)` | YAML 少了必填欄位 |
| `config/x.yaml:2:12: /max_speed: 900 is outside RANGE(0, 500)` | 超出範圍 |
| `config/x.yaml:2:12: /max_speed: expected int32, got the string "100" (remove the quotes)` | 數字加了引號 |
| `config/nope.yaml: cannot be read` | 檔案不存在，路徑相對於 `config/` |
| `config/ctrl.yaml: #/nope: 'nope' not found` | 片段路徑不存在 |

---

## 7. 共享資源：多個 block 共用同一份資料

### 7.1 先決定要不要用

大部分情況**用不到共享資源**。先照這張表判斷：

| 情況 | 用什麼 | 哪一節 |
|---|---|---|
| 一個 block 算出來，其他 block 讀 | **普通輸出埠**（資料不可變，多個下游共用一個指標） | 第 3 節 |
| 只有一個 block 會改，資料很大又常常局部更新 | **快照**：輸出埠傳 COW 型別 | 第 7.9 節 |
| **多個 block 都要改同一份資料** | **共享資源 `fw::Shared`** | 第 7.2 節起 |
| 讀的人不能等，資料可切成區塊 | 共享資源加 `policy: versioned` | 第 7.8 節 |
| 第三方函式庫要自己讀寫一個檔案 | **`fw::File`** | 第 7.11 節 |
| 別的框也要讀 | 不能共享資源。由某個 block 輸出 `.msg` 版本 | 第 8 節 |

`fw::Shared` 這個名字只出現在文件裡，**程式碼裡不會寫到它**，你只會看到 `fw::Read<T>` 和 `fw::Write<T>`。

### 7.2 最小例子：兩個寫入者、一個讀取者

目標：室內、室外兩支溫度計都把讀數寫進同一份 `TempLog`。

#### 第 1 步：定義資料型別

就是普通的 struct。**不用繼承任何東西，不要在裡面放 mutex。** 要有預設建構子，因為資源是框架建立的。

**`types/temp_log.hpp`**

```cpp
#pragma once

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "fw/fw.hpp"

namespace learn {

struct TempLog {
    std::vector<double> values;
};

// Only needed for store: (section 7.7). Keep them in the type's namespace so that the framework finds them.
inline void fw_save(const TempLog& log, const std::filesystem::path& path) {
    std::ofstream f(path);
    for (double v : log.values) f << v << "\n";
    if (!f) throw std::runtime_error("cannot write " + path.string());
}
inline void fw_load(TempLog& log, const std::filesystem::path& path) {
    std::ifstream f(path);
    log.values.clear();
    for (double v; f >> v;) log.values.push_back(v);
}

}  // namespace learn

FW_REGISTER_TYPE(learn::TempLog, "learn/TempLog")
```

#### 第 2 步：寫 block，宣告「我要用資源」

**`algorithms/temp_log_blocks/temp_log_blocks.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"
#include "types/reading.hpp"
#include "types/temp_log.hpp"

using learn::TempLog;

namespace {

// Writer: fw::Write<T>, used like a pointer
void record(const Reading& in, fw::Write<TempLog> log) { log->values.push_back(in.celsius); }

// Reader: fw::Read<T>, const access. Triggered by the resource's version port.
void report(const fw::ResourceVersion& v, fw::Read<TempLog> log) {
    std::printf("report: version %llu, %zu value(s)\n", static_cast<unsigned long long>(v.version), log->values.size());
    std::fflush(stdout);
}

// Does not wait: empty when a writer holds the resource
void peek(const int& tick, fw::Optional<fw::Read<TempLog>> log) {
    if (log) std::printf("peek %d: %zu value(s)\n", tick, (*log)->values.size());
    else std::printf("peek %d: busy\n", tick);
    std::fflush(stdout);
}

// Class form: USES lists the resource, and the order of the parameters follows USES
struct LogSummary {
    void on_x(const Reading&) {}
    void on_configure(fw::Read<TempLog> log) {
        std::printf("LogSummary: start with %zu value(s)\n", log->values.size());
        std::fflush(stdout);
    }
    void on_eos(fw::Read<TempLog> log, double& mean) {
        double sum = 0;
        for (double v : log->values) sum += v;
        mean = log->values.empty() ? 0 : sum / log->values.size();
        std::printf("LogSummary: %zu value(s), mean %.2f\n", log->values.size(), mean);
        std::fflush(stdout);
    }
};

}  // namespace

FW_REGISTER_ALGORITHM(record,
    INPUTS(("in", Reading, trigger, QUEUE(32, drop_oldest))),
    RESOURCES(("log", TempLog, write)));
FW_REGISTER_ALGORITHM(report,
    INPUTS(("v", fw::ResourceVersion, trigger, QUEUE(32, drop_oldest))),
    RESOURCES(("log", TempLog, read)));
FW_REGISTER_ALGORITHM(peek,
    INPUTS(("tick", int, trigger, QUEUE(32, drop_oldest))),
    RESOURCES(("log", TempLog, read, NONBLOCKING)));
FW_REGISTER_ALGORITHM_CLASS(LogSummary,
    RESOURCES(("log", TempLog, read)),
    OUTPUTS(("mean", double)),
    ON(("x", Reading, trigger, QUEUE(32, drop_oldest)), &LogSummary::on_x),
    ON_CONFIGURE(&LogSummary::on_configure, USES("log")),
    ON_EOS(&LogSummary::on_eos, USES("log"), EMITS("mean")));
```

`RESOURCES` 裡的 `"log"` 是**這個 block 的資源埠名字**，和圖上資源叫什麼沒關係。參數順序是 `INPUTS → RESOURCES → OUTPUTS`。

#### 第 3 步：在圖上建立資源並接線

**`graphs/shared.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    workers: 4
    resources:
      temps: { type: learn/TempLog }
    blocks:
      indoor:  { driver: thermometer }
      outdoor: { driver: thermometer }
      src:     { driver: counter }
      rec_in:  { algorithm: record }
      rec_out: { algorithm: record }
      rep:     { algorithm: report }
      look:    { algorithm: peek }
      sum:     { algorithm: LogSummary }
edges:
  - config://thermo.yaml -> [indoor.params, outdoor.params]
  - indoor.reading -> [rec_in.in, sum.x]
  - outdoor.reading -> rec_out.in
  - src.n -> look.tick
  - temps -> [rec_in.log, rec_out.log, rep.log, look.log, sum.log]
  - temps.version -> rep.v
```

三個容易寫錯的地方：

| 位置 | 正確寫法 | 常見錯誤 |
|---|---|---|
| `resources:` 在哪 | 在**框底下**，和 `blocks:` 同層 | 寫在最外層：`resources are declared in their box` |
| `type:` | **`FW_REGISTER_TYPE` 的字串** `learn/TempLog` | 寫 C++ 名稱 `TempLog`：`resource 'log' is TempLog, the port takes learn/TempLog` |
| 接線 | `資源名 -> block.埠`，方向固定 | 沒接：`resource port 'w.log' (learn/TempLog) is not connected to a resource of box 'main'` |

「讀或寫」**不寫在圖上**，由 block 的 `RESOURCES` 決定。

#### 第 4 步：執行

```bash
colcon build && source install/setup.bash
fw launch graphs/shared.graph.yaml
```

輸出的一段（順序每次可能不同）：

```text
LogSummary: start with 0 value(s)
report: version 1, 2 value(s)
report: version 2, 2 value(s)
peek 1: 2 value(s)
report: version 3, 4 value(s)
report: version 4, 4 value(s)
peek 3: busy
LogSummary: 10 value(s), mean 26.00
```

### 7.3 鎖怎麼運作：你完全不用碰鎖

框架幫你做這些事：

1. **handler 執行前取鎖，返回後放鎖。** 你的程式裡沒有任何 lock。
2. `write` 獨占；`read` 可以多個同時進行。
3. **一個 handler 用多個資源時，一次全部取得，或一個都不拿。** 所以不會死結，不用煩惱取鎖順序。
4. **等鎖的 block 不占用 worker 執行緒**，worker 會去做別的事，不會把整個框卡死。
5. 按到達順序排隊：後到的讀取者不會插隊到正在等的寫入者前面，雙方都不會餓死。
6. handler 拋例外也會放鎖。
7. **輸出在放鎖之後才送出**，所以下游被觸發時鎖已經放開了。

你要負責的只有幾件事：

| 要 | 不要 |
|---|---|
| 把 handler 寫短。鎖在**整個函式期間**都拿著 | 在 handler 裡做耗時運算，會卡住所有其他使用者 |
| 需要的資料在 handler 內複製出來 | **存下資源內的指標或參考，留到 handler 返回之後。** 鎖已經沒了 |
| 只讀就宣告 `read` | 在資料 struct 裡放 mutex |
| | 把 `fw::Read` / `fw::Write` 交給另一條執行緒 |

### 7.4 三種存取方式對照

| `RESOURCES` 寫法 | 函式參數 | 行為 |
|---|---|---|
| `("m", T, write)` | `fw::Write<T> m` | 獨占，可改，也可以讀 |
| `("m", T, read)` | `fw::Read<T> m` | 共用，唯讀 |
| `("m", T, read, NONBLOCKING)` | `fw::Optional<fw::Read<T>> m` | **不等待**。有人在寫時 `m` 是空的 |

用法：

```cpp
log->values.size();            // like a pointer: fw::Read and fw::Write
(*log).values.size();
if (log) (*log)->values.size();  // NONBLOCKING: check first, then (*log)-> gives the data
```

參數要**傳值**，不要寫成 `fw::Read<T>&`。寫錯會看到：

```text
fw: a resource parameter is fw::Read<T> (read), fw::Write<T> (write) or fw::Optional<fw::Read<T>> (read, NONBLOCKING), by value, with the resource's type (§4.4)
```

`NONBLOCKING` 只用在讀取。寫入者一定要排隊：`fw: NONBLOCKING applies to read resources: a writer always waits for its turn (§4.4)`。

一個 block **對同一個資源只能有一個埠**。接兩個埠到同一資源是錯誤。同時要讀和寫時，宣告 `write` 就好，`fw::Write<T>` 也能讀。

### 7.5 `.version`：資源更新時通知別人

每個資源都自動有一個輸出埠 `<資源名>.version`，型別 `fw::ResourceVersion`，每次寫入後送出遞增的版本號。範例裡的 `report` 就是這樣觸發的：

```yaml
- temps.version -> rep.v
```

要注意的是，看上面的輸出，`version 1` 那行讀到了 2 筆資料。**版本事件只告訴你「有更新」，你讀到的永遠是當下最新的內容**，不是那個版本的內容。需要每個版本完整內容時請用快照（第 7.9 節）。

不能讓寫入者被自己資源的 `.version` 觸發，會形成環，載入時會報錯。

### 7.6 在生命週期回呼使用資源、class 形式的寫法

範例的 `LogSummary` 示範了：

- `ON_CONFIGURE(&LogSummary::on_configure, USES("log"))`：開始前先看資源（可以當作初始化時機，用 `write` 埠就能填初值）。
- `ON_EOS(&LogSummary::on_eos, USES("log"), EMITS("mean"))`：資料結束後讀最終結果。框架保證**所有寫入者都結束了，讀取者的 `ON_EOS` 才會執行**。

class 形式有多個 handler 時，每個用到資源的 handler 都要 `USES`：

```cpp
ON(("ba",   BaResult,   trigger), &Apply::on_ba,   USES("atlas"), EMITS("version")),
ON(("loop", Correction, trigger), &Apply::on_loop, USES("atlas"), EMITS("version")),
RESOURCES(("atlas", Atlas, write)),
```

只有一個 handler 的簡寫形式（`operator()` 或函式）會自動用到全部資源。

### 7.7 `store:`：資源自動存檔與載入

要讓資源開機時讀檔、結束時存檔，型別要提供兩個函式，**放在型別的 namespace 裡**（就是前面 `temp_log.hpp` 寫的那兩個）：

```cpp
void fw_save(const T&, const std::filesystem::path&);
void fw_load(T&, const std::filesystem::path&);
```

圖上加 `store:`：

**`graphs/store.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    resources:
      temps: { type: learn/TempLog, store: "data://log/temps.txt", create_if_missing: true, backup_on_open: true, keep: 3 }
    blocks:
      indoor: { driver: thermometer }
      rec:    { algorithm: record }
      sum:    { algorithm: LogSummary }
edges:
  - config://thermo.yaml -> indoor.params
  - indoor.reading -> [rec.in, sum.x]
  - temps -> [rec.log, sum.log]
```

連跑兩次，第二次會接著第一次的資料：

```text
LogSummary: start with 0 value(s)
LogSummary: 5 value(s), mean 26.00
```
```text
LogSummary: start with 5 value(s)
LogSummary: 10 value(s), mean 26.00
```

檔案在 `data/log/temps.txt`，備份在 `data/log/temps.txt.backups/`。

| 鍵 | 意思 |
|---|---|
| `store: "data://x"` | 存檔位置 |
| `create_if_missing: true` | 檔案不存在時用預設建構開始。**不寫的話第一次會啟動失敗** |
| `backup_on_open: true` | 啟動時先備份舊檔 |
| `keep: 3` | 保留幾份備份。`backup_on_open` 時**必填**，框架不替你決定 |

相關錯誤：

```text
resource 'log': keep applies with backup_on_open
resource 'grid': store: needs fw_save(const learn/GridMap&, const std::filesystem::path&) and fw_load(learn/GridMap&, const std::filesystem::path&), found by ADL (§4.4)
/workspaces/learn/data/none/x.txt does not exist (set create_if_missing: true to start with the type's default value)
```

**存檔只在正常結束（cleanup）時發生，執行中不會自動存。** 崩潰時保留的是上次的檔案，存檔本身是原子寫入，不會留下寫到一半的檔案。

列出與還原備份（框要先停止）：

```bash
fw data backups data://log/temps.txt      # lists: number, file name, clean or after_crash
fw data restore data://log/temps.txt --backup 0
```

### 7.8 `policy: versioned`：讀取者永遠不用等

預設的鎖策略叫 `writer_pref`，寫入時讀取者要等。如果**讀取者不能等**（例如控制迴圈要讀地圖），資料也能切塊，可以用 `versioned`：

| | `writer_pref`（預設） | `versioned` |
|---|---|---|
| 讀取者 | 可能要等寫入者 | **永遠不等**，拿到最近一個完整版本（可能稍舊） |
| 寫入者 | 原地改 | 改副本，只複製改到的區塊，完成後才發布 |
| 寫入中拋例外 | **改到一半的內容會留下來** | 整次寫入作廢 |
| 型別要求 | 任何型別 | 必須是 COW 型別 |
| 適合 | 寫入都很短，或一次改很大範圍 | 每次只改一小部分（voxel、格點地圖） |

實測這個差異：`n == 3` 時改到一半就拋例外。

| 寫入 3 之後，讀取者看到 | 結果 |
|---|---|
| `writer_pref` 的 `TempLog` | 多了改到一半的那筆（共 4 筆，而不是 2 筆或 3 筆） |
| `versioned` 的 `GridMap` | 維持第 2 次寫入的內容（`resolution` 還是 2） |

**COW 型別的寫法：** 用 `fw::TiledMap` 存資料，再用 `FW_COW_TYPE` 列出**全部成員**。

**`types/grid_map.hpp`**

```cpp
#pragma once

#include <array>

#include "fw/fw.hpp"

namespace learn {

using Tile = std::array<double, 8>;  // one block of the map; its size is your choice

struct GridMap {
    double resolution = 0.1;
    fw::TiledMap<int, Tile> tiles;   // key -> tile
};

}  // namespace learn

FW_COW_TYPE(learn::GridMap, resolution, tiles);
FW_REGISTER_TYPE(learn::GridMap, "learn/GridMap")
```

**`algorithms/grid_blocks/grid_blocks.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"
#include "types/grid_map.hpp"
#include "types/reading.hpp"

using learn::GridMap;

namespace {

void paint(const Reading& in, fw::Write<GridMap> map) {
    const int key = static_cast<int>(in.celsius) / 8;
    const int cell = static_cast<int>(in.celsius) % 8;
    map->tiles.edit(key)[cell] = in.celsius;   // edit() copies the tile first when needed
}

void look_map(const Reading& in, fw::Read<GridMap> map) {
    double sum = 0;
    map->tiles.for_each([&](const int&, const learn::Tile& t) {
        for (double v : t) sum += v;
    });
    std::printf("look_map: %zu tile(s), sum %.0f at %.0f\n", map->tiles.size(), sum, in.celsius);
    std::fflush(stdout);
}

}  // namespace

FW_REGISTER_ALGORITHM(paint, INPUTS(("in", Reading, trigger, QUEUE(32, drop_oldest))), RESOURCES(("map", GridMap, write)));
FW_REGISTER_ALGORITHM(look_map, INPUTS(("in", Reading, trigger, QUEUE(32, drop_oldest))), RESOURCES(("map", GridMap, read)));
```

**`graphs/versioned.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    resources:
      grid: { type: learn/GridMap, policy: versioned, max_retained_tiles: 1000 }
    blocks:
      thermo: { driver: thermometer }
      draw:   { algorithm: paint }
      see:    { algorithm: look_map }
edges:
  - config://thermo.yaml -> thermo.params
  - thermo.reading -> [draw.in, see.in]
  - grid -> [draw.map, see.map]
```

處理函式完全不用改，`writer_pref` 和 `versioned` 的 `fw::Write<T>`、`fw::Read<T>` 寫法相同，只有圖上的 `policy:` 不同。

`fw::TiledMap<Key, Tile, Hash = std::hash<Key>>` 的成員：

| 成員 | 用途 |
|---|---|
| `find(key)` | 唯讀，回傳 `const Tile*`，沒有就是 `nullptr` |
| `edit(key)` | 要寫時用。回傳 `fw::TileRef<Tile>`，可用 `->`、`[]`、`.set(值)`。不存在時建立 |
| `erase(key)` | 刪除，回傳 `bool` |
| `size()`、`empty()` | 區塊數 |
| `for_each(fn(key, const Tile&))` | 走訪所有區塊 |
| `for_each_changed(舊版, fn(key, const Tile* 現在))` | 只走訪和舊版不同的區塊 |

要注意的地方：

- **`fw::TileRef` 不能留到 handler 結束之後。** 它只在這次呼叫內有效，之後使用會拋例外。
- 不小心的複製（`auto m = map;`、傳值給 helper）不會壞，但之後每個改到的區塊都會多複製一次。helper 請用參考傳遞。
- `FW_COW_TYPE` 的成員只能是 `fw::TiledMap`、另一個 COW 型別，或 trivially copyable 的型別（純數值、`std::array`）。放 `std::vector` 會編譯失敗，因為每次寫入都要整份複製。
- 一次改很大範圍（整張地圖）時，`versioned` 不划算，改用 `writer_pref`。

| 錯誤 | 原因 |
|---|---|
| `resource 'log' is policy: versioned, but its type learn/TempLog is not a COW type ...` | 型別沒用 `FW_COW_TYPE` |
| `fw: FW_COW_TYPE members may only be fw::TiledMap, COW types or trivially copyable types (§4.4)...` | 有 `std::vector` 之類的成員 |
| `fw: FW_COW_TYPE must list every member of the type, in declaration order (§4.4)` | 漏列成員，或順序不對 |

`max_retained_tiles: N` 只用於監控：舊版本留住的區塊超過 N 時警告（表示有讀取者把舊版本留太久）。沒有預設值，不寫就不檢查。

### 7.9 快照：單一寫入者的另一種選擇

如果只有一個 block 會改，就不需要資源。block 把 COW 型別放在**自己的成員**，用輸出埠送出，讀取者用 `latest` 接。送出時只複製索引，成本很低，讀的人也拿到不會變的版本：

**`algorithms/mapper/mapper.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"
#include "types/grid_map.hpp"
#include "types/reading.hpp"

using learn::GridMap;

namespace {

struct Mapper {
    GridMap map;  // this block's own, so no resource and no lock
    void operator()(const Reading& in, GridMap& out) {
        map.tiles.edit(static_cast<int>(in.celsius))[0] = in.celsius;
        out = map;  // cheap: shares every tile that did not change
    }
};

void use_snapshot(const int& n, fw::Optional<const GridMap&> map) {
    std::printf("use_snapshot %d: %zu tile(s)\n", n, map ? map->tiles.size() : 0);
    std::fflush(stdout);
}

}  // namespace

FW_REGISTER_ALGORITHM_CLASS(Mapper, INPUTS(("in", Reading, trigger, QUEUE(32, drop_oldest))), OUTPUTS(("map", GridMap)));
FW_REGISTER_ALGORITHM(use_snapshot, INPUTS(("n", int, trigger, QUEUE(32, drop_oldest)), ("map", GridMap, latest)));
```

**`graphs/snapshot.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      thermo: { driver: thermometer }
      src:    { driver: counter }
      mapper: { algorithm: Mapper }
      reader: { algorithm: use_snapshot }
edges:
  - config://thermo.yaml -> thermo.params
  - thermo.reading -> mapper.in
  - mapper.map -> reader.map
  - src.n -> reader.n
```

快照的版本由誰寫、什麼時候發布都很清楚，能用快照就不要用資源。

### 7.10 資源的限制

| 限制 | 錯誤訊息 |
|---|---|
| 只能在同一個框內用 | `'r' is in box 'b', resource 'log' in 'a'; a resource is used in its own box only (§4.4)` |
| 型別要有預設建構子 | `fw: a resource type is default constructible (the box makes it, §4.4)` |
| 接線的型別要相同 | `resource 'log' is learn/GridMap, the port takes learn/TempLog` |
| 資源和 block 名稱不能重複 | `resource 'x' has the name of a block; they share names` |

要讓別的框讀，在這個框放一個 block，定期讀資源、輸出 `.msg` 型別。想在 ROS 上看資源，用資源 tap（第 11.7 節）。

### 7.11 `fw::File`：讓函式庫自己讀寫檔案

有些函式庫（例如 RTAB-Map 的 sqlite 資料庫）會在運算中**自己開檔案**。這時框架沒辦法經過埠寫入，改由框架管理檔案，只把路徑交給 block：

**`algorithms/file_user/file_user.cpp`**

```cpp
#include <cstdio>
#include <fstream>

#include "fw/fw.hpp"
#include "types/reading.hpp"

namespace {
struct FileUser {
    std::ofstream out;
    void on_configure(fw::File& db) {
        std::printf("FileUser: path=%s uri=%s\n", db.path().c_str(), db.uri().c_str());
        std::fflush(stdout);
        out.open(db.path(), std::ios::app);   // a real block hands db.path() to the library
    }
    void on_x(const Reading& in) {
        out << in.celsius << "\n";
        out.flush();
    }
};

// A check of the content, named in the graph. Empty: fine; otherwise the reason.
std::string not_empty(const std::filesystem::path& p) {
    return std::filesystem::file_size(p) > 0 ? "" : "empty file";
}
}  // namespace

FW_REGISTER_FILE_VALIDATOR("not_empty", &not_empty)

FW_REGISTER_ALGORITHM_CLASS(FileUser,
    RESOURCES(("db", fw::File, read_write)),
    ON(("x", Reading, trigger, QUEUE(32, drop_oldest)), &FileUser::on_x),
    ON_CONFIGURE(&FileUser::on_configure, USES("db")));
```

**`graphs/file.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    resources:
      db: { plugin: "data://db/run.txt", create_if_missing: true, backup_on_open: true, keep: 2 }
    blocks:
      thermo: { driver: thermometer }
      user:   { algorithm: FileUser }
edges:
  - config://thermo.yaml -> thermo.params
  - thermo.reading -> user.x
  - db -> user.db
```

輸出：

```text
FileUser: path=/workspaces/learn/data/db/run.txt uri=data://db/run.txt
```

和一般資源的差別：

| | 一般資源 | `fw::File` |
|---|---|---|
| 圖上寫 | `type: pkg/Name` | `plugin: "data://..."`（兩者不能同寫） |
| block 宣告 | `read` / `write` | `read` / `read_write` |
| 參數 | `fw::Read<T>` / `fw::Write<T>` | `const fw::File&` / `fw::File&` |
| 框架上鎖 | 會 | **不會**，靠「只有一個寫入者」保證 |
| `.version` 埠 | 有 | 沒有 |
| 介面 | 你的型別 | `path()`（真正要用的路徑）、`uri()` |

`fw::File` 的選項：

| 鍵 | 意思 |
|---|---|
| `plugin: "data://x"` | 必填。結尾是 `/` 就表示資料夾 |
| `create_if_missing: true` | 不存在時建立空檔 |
| `backup_on_open: true` | 開檔前備份，備份在 `<檔案>.backups/` |
| `keep: N` | 保留幾份（有 `backup_on_open` 時必填）。最新一份 clean 的備份一定保留，不算在 N 裡 |
| `after_crash: use_as_is` | **預設**。上次沒正常關閉時照用，並回報 degraded |
| `after_crash: restore_last_clean` | 上次沒正常關閉時，自動改用最新一份正常的備份 |
| `validate: 名字` | 挑備份時用這個函式檢查內容（`FW_REGISTER_FILE_VALIDATOR` 註冊） |
| `copy_on_open: true` | 給 block 一份複本，結束時刪除，**原檔不會被改**。重播時很好用。不能和前面的寫入選項並用 |

框架怎麼知道上次有沒有正常關閉：看 `<檔案>.backups/state.json`。開檔時寫 `open`，正常結束時寫 `clean`。

實際看到的訊息：

```text
# after_crash: use_as_is (default)
block 'a' reports degraded: the last run did not close .../f.txt cleanly; it is used as is (after_crash: use_as_is)
# after_crash: restore_last_clean
the last run did not close .../f.txt cleanly: restored the clean backup 20261107-040108-clean.txt; the file the crash left is backup ...
```

崩潰時的檔案一定會先備份（標 `after_crash`），不會被直接蓋掉。

**一個檔案只能有一個 `read_write` 使用者**，多個 `read` 可以：

```text
error: 'resource a' and 'resource b' both write data/f.txt; a file has one writer (write it from one box, through one URI)
```

同一個資源接給兩個 `read_write` 的 block（`db -> [u1.db, u2.db]`）也一樣是錯誤。檔案沒有鎖，兩個 block 會互相蓋掉對方寫的內容。`copy_on_open: true` 時也一樣，因為它們拿到的是同一份複本：

```text
error: 'u1' and 'u2' both use resource 'db' read_write; a file has one writer (make the others read)
```

**跨框：** 每個框各宣告一個資源綁定同一 URI。選項只寫在 `read_write` 那一框，`read` 那一框只能寫 `plugin:`。框之間沒有啟動順序，讀取框可能先看到舊檔。

**能用埠就不要用 `fw::File`。** 如果只是結束時存檔，用 `store:` 或 `out://` 比較簡單。另外，函式庫把參數也存進檔案時（RTAB-Map 可以從資料庫讀回參數），block 要關掉這個行為，參數只該來自 `config://`。

---

## 8. `FW_REGISTER_TYPE`：資料型別

埠上流動的每個型別都需要一個**穩定的名字**。忘記註冊會看到：

```text
fw: type has no stable type ID; add FW_REGISTER_TYPE(T, "pkg/Name") at global scope (§4.5)
```

### 8.1 怎麼選型別

依序問：

1. **ROS 已經有合適的 `.msg` 嗎？** 直接用，**不用註冊**。可以跨框、可以錄。
2. **資料只在一個框內流動嗎？** 寫一般 struct，加一行 `FW_REGISTER_TYPE`。絕大多數內部型別都是這種。
3. **要離開框？** 寫自己的 `.msg`（見 [`orbslam3_tutorial.md` 第 2.3 節](orbslam3_tutorial.md#23-自訂-msg)）。
4. **框內想用原生型別，跨框才轉換？** 用 `TypeAdapter`（第 8.4 節）。
5. **不能序列化，但想在 ROS 上看個大概？** 用 `VIEW`（第 8.3 節）。

### 8.2 一般 struct（行程內型別）

```cpp
struct Reading { double celsius = 0; };
FW_REGISTER_TYPE(Reading, "learn/Reading")
```

- 寫在**全域範圍**，不能在 `namespace` 裡面。型別本身可以在 namespace 內，例如 `FW_REGISTER_TYPE(learn::TempLog, "learn/TempLog")`。
- 字串建議寫成 `專案/型別名`。不能含 `/msg/` 或 `/srv/`。
- 這個字串就是圖上 `type:` 要寫的內容。
- 共用的型別放 `types/xxx.hpp`，並用 `#pragma once`。
- 行程內型別**不能跨框**，也不能 tap（除非有 `VIEW`）。

型別裡有逗號要先取別名，不然巨集會切錯位置：

```cpp
using WordMap = std::map<std::string, int>;
FW_REGISTER_TYPE(WordMap, "learn/WordMap")
// INPUTS(("m", WordMap, latest))    not std::map<std::string, int> directly
```

忘記時的訊息：`fw: invalid trigger policy; expected trigger, latest or sync. If the port type contains a top-level comma (e.g. std::map<K, V>), declare an alias first`。

內建型別已註冊好，型別 ID 如下（也都是行程內型別）：

| C++ | 型別 ID |
|---|---|
| `bool` | `fw/bool` |
| `int` / `std::int32_t` | `fw/int32` |
| `std::int64_t` | `fw/int64` |
| `float` | `fw/float32` |
| `double` | `fw/float64` |
| `std::string` | `fw/string` |

### 8.3 ROS 訊息

**直接用 `.msg`：** 不用任何註冊，型別 ID 就是 `std_msgs/msg/Float64`。專案要加相依，步驟見第 11.1 節。

**用 fw 提供的同名型別**（框內比較好用，跨框仍是官方訊息）：標頭依 ROS 套件分檔：

| 標頭 | 內容 |
|---|---|
| `fw/ros/msgs/std.hpp` | `fw::Int32`、`fw::Float64`、`fw::String`、`fw::Empty`… |
| `fw/ros/msgs/geometry.hpp` | `fw::Pose`、`fw::TransformStamped`、`fw::Twist`… |
| `fw/ros/msgs/nav.hpp` | `fw::Odometry`、`fw::Path`、`fw::OccupancyGrid`… |
| `fw/ros/msgs/sensor.hpp` | `fw::Imu`、`fw::LaserScan`、`fw::PointCloud2`… |
| `fw/ros/msgs/image.hpp` | `fw::Image`（裡面是 `cv::Mat`，要 OpenCV） |

大小寫有差別：`fw/int32` 是 `std::int32_t`，`fw/Int32` 是 `fw::Int32`，兩者是不同型別，**不能互接**。

這些型別的 stamp 在信封裡，要用 `Msg<T>::stamp()` 讀，不存在 struct 內。

**`VIEW`（只給 tap 看）：** 行程內型別加上單向轉換，就能在 ROS 上看，但**不能跨框**：

```cpp
namespace { void log_view(const LogState& s, std_msgs::msg::Float64& m) { m.data = s.last; } }
FW_REGISTER_TYPE(LogState, "learn/LogState", VIEW(std_msgs::msg::Float64, &log_view))
```

完整範例在第 11.7 節。

### 8.4 `TypeAdapter`：框內原生型別，跨框才轉換

框內用自己的型別不用轉換，要跨框時才自動轉成 `.msg`。這是最麻煩的選項，只有型別真的就是那個訊息的另一種表示法才用（`fw::Image` 就是這樣）：

**`types/celsius.hpp`**

```cpp
#pragma once

#include <rclcpp/type_adapter.hpp>
#include <std_msgs/msg/float64.hpp>

#include "fw/fw.hpp"

struct Celsius {
    double v = 0;
};

template <>
struct rclcpp::TypeAdapter<Celsius, std_msgs::msg::Float64> {
    using is_specialized = std::true_type;
    using custom_type = Celsius;
    using ros_message_type = std_msgs::msg::Float64;
    static void convert_to_ros_message(const custom_type& s, ros_message_type& d) { d.data = s.v; }
    static void convert_to_custom(const ros_message_type& s, custom_type& d) { d.v = s.data; }
};
FW_REGISTER_TYPE(Celsius, "learn/Celsius", std_msgs::msg::Float64)
```

不要把不相干的東西硬接到某個訊息上（例如把特徵點硬說是 `sensor_msgs/Image`），`ros2 topic echo` 和 RViz 會解讀錯誤。

---

## 9. `CALLS` / `SERVES`：呼叫另一個 block

適用情況：「發出請求，等回應回來再繼續處理」。**只能在同一個框內**，外部 ROS service 的接法見第 11.4 節。

不會阻塞：`ctx.call()` 立刻返回，回應會觸發另一個 handler。

**`algorithms/calls_demo/calls_demo.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"
#include "types/reading.hpp"

// A service type: any type with nested Request and Response. A ROS .srv qualifies as is.
struct AddSrv {
    struct Request {
        int a = 0;
        int b = 0;
    };
    struct Response {
        int sum = 0;
    };
};
FW_REGISTER_TYPE(AddSrv::Request, "learn/AddRequest")
FW_REGISTER_TYPE(AddSrv::Response, "learn/AddResponse")

namespace {

// The provider
struct Adder {
    void serve(const AddSrv::Request& req, fw::Responder<AddSrv> res) {
        AddSrv::Response r;
        r.sum = req.a + req.b;
        res.reply(r);        // or move res into a member and reply later
    }
};

// The caller
struct Asker {
    int n = 0;
    void on_x(fw::Context& ctx, const Reading&) { ctx.call("add", AddSrv::Request{++n, 100}); }
    void on_reply(const fw::Reply<AddSrv>& r, fw::Optional<int>& out) {
        if (r.ok()) {
            out = r.response().sum;
        } else {
            std::printf("call failed: %s: %s\n", fw::to_string(r.status()), r.error().c_str());
            std::fflush(stdout);
        }
    }
};

}  // namespace

FW_REGISTER_ALGORITHM_CLASS(Adder, SERVES(("add", AddSrv), &Adder::serve));
FW_REGISTER_ALGORITHM_CLASS(Asker,
    OUTPUTS(("sum", int)),
    ON(("x", Reading, trigger, QUEUE(16, drop_oldest)), &Asker::on_x, USES("add")),
    CALLS(("add", AddSrv, MAX_INFLIGHT(4), TIMEOUT(1s)), &Asker::on_reply, EMITS("sum")));
```

**`graphs/calls.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      thermo: { driver: thermometer }
      ask:    { algorithm: Asker }
      add:    { algorithm: Adder }
      out:    { algorithm: print_int }
edges:
  - config://thermo.yaml -> thermo.params
  - thermo.reading -> ask.x
  - ask.add -> add.add
  - ask.sum -> out.v
```

輸出 `print_int: 101`、`102`……。接線方向一律是**呼叫者 → 提供者**。

| 語法 | 意思 |
|---|---|
| `CALLS(("埠", Srv, 選項...), &C::回應處理函式, EMITS(...))` | 我會呼叫別人 |
| `MAX_INFLIGHT(n)` | 同時最多 n 個還沒回應的呼叫，超過就回 `busy` |
| `TIMEOUT(t)` | 多久沒回應就算逾時 |
| `USES("埠")` | **要呼叫的 handler 必須列出**，否則執行時報錯 |
| `SERVES(("埠", Srv), &C::serve)` | 我回答別人 |

每個呼叫**一定會收到一個回應**，`r.status()` 可能是：

| 狀態 | 意思 |
|---|---|
| `ok` | 成功，才能讀 `r.response()`，否則會拋例外 |
| `failed` | 提供者呼叫了 `res.fail("原因")`，用 `r.error()` 看 |
| `timeout` | 超過 `TIMEOUT` |
| `busy` | 超過 `MAX_INFLIGHT` |
| `unavailable` | 沒有人能回答 |
| `abandoned` | 提供者把 `Responder` 丟掉沒回，或框在停止 |

`fw::Responder` 不能複製、可以移動，存進成員就能稍後回應。回應處理函式裡還能用 `r.request()` 看原本的請求。

---

## 10. URI 與 filesystem plugin

圖上所有檔案都用 URI 表示，**前綴就是專案裡的資料夾名稱**，看到 URI 就知道檔案在哪：

```text
config://sensors/lidar.yaml#/extrinsic
  │        │                   └─ fragment: a part of the YAML
  │        └─ the file: <project>/config/sensors/lidar.yaml
  └─ prefix: the folder, and the plugin that handles it
```

路徑不能以 `/` 開頭，不能含 `..` 或 `:`。

### 10.1 內建前綴

| 前綴 | 方向 | 用途 |
|---|---|---|
| `config://` | 讀；寫是提案 | 參數，第 6 節 |
| `data://` | 給資源用 | `fw::File` 和 `store:`，第 7 節 |
| `bags://` | 只給 mcap driver 的 `file:` | 重播的 bag，第 11.6 節 |
| `out://` | 寫 | 輸出結果 |
| `state://` | 讀寫 | 下次啟動要接續的狀態 |

### 10.2 `out://`：把結果寫到檔案

把輸出接到 URI 就好，block 完全不用開檔：

```yaml
edges:
  - m.out -> out://last.yaml               # keeps only the latest value
  - m.out -> out://runs/t-{start}.mcap     # all of them; {start} is the start time
```

| 副檔名 | 行為 |
|---|---|
| `.yaml` | 每筆都把整個檔案換成最新值 |
| `.mcap` | 每筆附加，可重播 |

- 只接 `.msg` 型別。行程內型別要自己寫 plugin。
- 可用 `{start}`（`YYYYmmdd-HHMMSS`）和 `{box}`。
- 預設佇列 1024、滿了丟新的。想改：`- { from: m.out, to: out://x.mcap, queue: 4096 }`。
- 檔案先寫 `.partial`，結束時才改名。崩潰時留下的是 `.partial`。

**不要在 YAML 的連線裡幫 URI 加引號。** `- m.out -> [a.x, "out://x.yaml"]` 會報 `no filesystem plugin for "out://`。整行是一個字串，引號會被當成內容。

### 10.3 `state://`：記住上次的狀態

同一個 URI 既能寫也能讀。讀到的是**上次執行留下的值**，執行中的寫入不會從輸出送回來，所以不會自己餵自己。

`state://` 和 `out://` 只接 `.msg` 型別，所以這裡開始用到 ROS 訊息。**請先做完[第 11.1 節](#111-專案要加什麼相依)的相依設定**，否則會編譯失敗。

**`algorithms/state_demo/state_demo.cpp`**

```cpp
#include <cstdio>

#include <std_msgs/msg/float64.hpp>

#include "fw/fw.hpp"
#include "types/reading.hpp"

using std_msgs::msg::Float64;

namespace {
void to_msg(const Reading& in, Float64& out) { out.data = in.celsius; }

// fw::Optional: the first run has no state file, so no value
void show_last(const int& n, fw::Optional<const Float64&> last) {
    if (last) std::printf("show_last %d: previous run ended at %.1f\n", n, last->data);
    else std::printf("show_last %d: no previous run\n", n);
    std::fflush(stdout);
}
}  // namespace

FW_REGISTER_ALGORITHM(to_msg, INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest))), OUTPUTS(("out", Float64)));
FW_REGISTER_ALGORITHM(show_last, INPUTS(("n", int, trigger, QUEUE(16, drop_oldest)), ("last", Float64, latest)));
```

**`graphs/state.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      thermo: { driver: thermometer }
      src:    { driver: counter }
      conv:   { algorithm: to_msg }
      show:   { algorithm: show_last }
edges:
  - config://thermo.yaml -> thermo.params
  - thermo.reading -> conv.in
  - conv.out -> [state://last_temp, out://last.yaml, out://runs/t-{start}.mcap]
  - state://last_temp -> show.last
  - src.n -> show.n
```

跑兩次，看到：

```text
show_last 1: no previous run
```
```text
show_last 1: previous run ended at 28.0
```

檔案分別在 `state/last_temp`、`out/last.yaml`（內容 `data: 28.0000`）、`out/runs/t-<時間>.mcap`。

**狀態和參數的差別：** `state://` 寫入立刻生效、不用確認。參數要審核。

### 10.4 自己寫 plugin：讀檔

檔案會被多個 block 用、只讀、格式是自訂的（例如詞袋、標定檔）時，寫成 plugin。圖上看起來就是一個資料來源：

**`filesystem/plugins/words.cpp`**

```cpp
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "fw/fw.hpp"

using WordList = std::vector<std::string>;
FW_REGISTER_TYPE(WordList, "learn/WordList")

namespace {

struct WordsPlugin {
    void on_configure(fw::PluginContext& ctx) { load(ctx); }
    void on_changed(fw::PluginContext& ctx) { load(ctx); }
    void load(fw::PluginContext& ctx) {
        std::ifstream f(ctx.path());
        if (!f) throw std::runtime_error("cannot read " + ctx.path());
        WordList words;
        for (std::string w; f >> w;) words.push_back(w);
        ctx.emit("words", std::move(words));
    }
};

void count_words(const int& n, const WordList& words) {
    std::printf("count_words %d: %zu word(s)\n", n, words.size());
    std::fflush(stdout);
}

}  // namespace

FW_REGISTER_FS_PLUGIN(WordsPlugin, PREFIX("vocab"),
    OUTPUTS(("words", WordList, GUARANTEED)),
    ON_CONFIGURE(&WordsPlugin::on_configure),
    ON_FILE_CHANGED(&WordsPlugin::on_changed));

FW_REGISTER_ALGORITHM(count_words,
    INPUTS(("n", int, trigger, QUEUE(16, drop_oldest)), ("words", WordList, latest)));
```

**`vocab/words.txt`**

```text
alpha beta gamma
```

**`graphs/plugin.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      src:  { driver: counter }
      cnt:  { algorithm: count_words }
edges:
  - src.n -> cnt.n
  - vocab://words.txt -> cnt.words
```

輸出 `count_words 1: 3 word(s)`。URI `vocab://words.txt` 固定對應 `<專案>/vocab/words.txt`。

| 子句 | 意思 |
|---|---|
| `PREFIX("x")` | URI 前綴，也就是資料夾名稱。小寫字母、數字、`_` |
| `OUTPUTS(("埠", T, GUARANTEED))` | `GUARANTEED`：`ON_CONFIGURE` 之後一定有值。下游的 `latest` 就不用 `INIT`，生命週期回呼也能用 |
| `ON_CONFIGURE(&C::fn)` 等 | 生命週期，參數固定是 `fw::PluginContext&` |
| `ON_FILE_CHANGED(&C::fn)` | 檔案被改時呼叫。沒寫就不監看 |
| `PARAMS(T)` | plugin 自己的參數，圖上寫 `config://ws.yaml -> maps://site/:params`，`ctx.params<T>()` 取值 |
| `ONE_INSTANCE` | 同一個檔案全圖只能一份（大型資料避免載兩次） |

`fw::PluginContext` 的成員：

| 成員 | 用途 |
|---|---|
| `ctx.path()` | 實際檔案路徑 |
| `ctx.uri()`、`ctx.fragment()`、`ctx.root()` | 原始 URI、`#` 後面的部分、專案根目錄 |
| `ctx.emit("埠", 值)` | 送出值，時間戳記是當下的 ROS 時鐘 |
| `ctx.report(健康狀態, 訊息)`、`ctx.warn(訊息)` | 回報 |
| `ctx.params<T>()` | 參數 |
| `ctx.keep()` | 給 plugin 自己開的執行緒用的 context，有效到 `ON_CLEANUP` 返回。回呼拿到的 `ctx&` 只在回呼內有效 |

多輸出的 plugin，連線要指名埠：`map://site.bin:snapshot -> loc.map`。

**資料比記憶體大時**，有 `fw::WorkingSet`（依位置載入周邊地圖）。本教學沒有涵蓋，請看 `robot-framework-plan.md` §4.8 和 [`include/fw/working_set.hpp`](../include/fw/working_set.hpp) 開頭的說明。

### 10.5 自己寫 plugin：寫檔

要把行程內型別存成自訂格式時使用。寫入 handler 在 plugin 專屬的執行緒跑，**不會占用 worker**：

**`filesystem/plugins/traj_writer.cpp`**

```cpp
#include <cstdio>
#include <fstream>

#include "fw/fw.hpp"

namespace {
struct TrajWriter {
    std::ofstream f;
    void on_configure(fw::PluginContext& ctx) { f.open(ctx.path()); }
    void on_n(fw::PluginContext& ctx, const int& n) {
        f << n << "\n";
        ctx.wrote(sizeof(n));  // write statistics
    }
    void on_eos(fw::PluginContext&) {
        f.close();
        std::printf("TrajWriter: closed\n");
        std::fflush(stdout);
    }
};
}  // namespace

FW_REGISTER_FS_PLUGIN(TrajWriter, PREFIX("traj"),
    ON(("n", int, trigger, QUEUE(256, drop_newest)), &TrajWriter::on_n),
    ON_CONFIGURE(&TrajWriter::on_configure),
    ON_EOS(&TrajWriter::on_eos));
```

**`graphs/writer.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      src: { driver: counter }
edges:
  - src.n -> traj://run.txt
```

要先建資料夾：`mkdir -p traj`。plugin 的寫入埠**一定要寫 `QUEUE`**，否則編譯失敗，因為丟資料等於檔案缺內容。任何丟棄都會讓 plugin 轉 degraded。

**要確認已寫入磁碟時**，用呼叫連線。這個流程本教學未實跑，寫法見 [`include/fw/write_file.hpp`](../include/fw/write_file.hpp)：

```cpp
CALLS(("save", fw::WriteFile<T>, TIMEOUT(30s)), &Saver::on_saved)
ctx.call("save", fw::WriteFile<T>::Request{value});   // in a handler that USES "save"
void on_saved(const fw::Reply<fw::WriteFile<T>>& r) { if (r.ok()) use(r.response().path); }
```

---

## 11. 和 ROS 接起來

框內的連線**不是 ROS topic**，`ros2 topic echo` 看不到。fw 要透過下面幾種方式才會碰到 ROS。

### 11.1 專案要加什麼相依

用到 ROS 訊息就要改兩個檔案。

**`package.xml`**：在 `<buildtool_depend>` 那行下面加：

```xml
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
```

**`CMakeLists.txt`**：把 `fw_add_blocks()` 改成：

```cmake
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
fw_add_blocks(LIBRARIES fw::ros ${std_msgs_TARGETS} ${std_srvs_TARGETS})
```

用 `_TARGETS`，不要用 `_LIBRARIES`，前者才會帶標頭檔。

### 11.2 跨框：不用寫任何程式

跨框連線就是一般連線，只是兩端在不同框。型別要有 `.msg` 形式：

**`algorithms/from_msg/from_msg.cpp`**

```cpp
#include <cstdio>

#include <std_msgs/msg/float64.hpp>

#include "fw/fw.hpp"

namespace {
void from_msg(Msg<std_msgs::msg::Float64> in) {
    std::printf("from_msg: %.1f at %.3f\n", in->data, in.stamp().ns / 1e9);
    std::fflush(stdout);
}
}  // namespace

FW_REGISTER_ALGORITHM(from_msg, INPUTS(("in", std_msgs::msg::Float64, trigger, QUEUE(16, drop_oldest))));
```

**`graphs/two_boxes.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  front:
    blocks:
      thermo: { driver: thermometer }
      conv:   { algorithm: to_msg }
  back:
    blocks:
      show:   { algorithm: from_msg }
edges:
  - config://thermo.yaml -> thermo.params
  - thermo.reading -> conv.in
  - conv.out -> show.in
```

時間戳記會完整保留。行程內型別跨框時：

```text
error: connection 'n.n -> c.v' (box 'a' -> 'b') carries the in-process type 'fw/int32' (no .msg form), which cannot cross boxes; place both blocks in one box, or have the block output a .msg version (§4.5)
```

**什麼時候要分框：** 一個 block 崩潰不想拖累其他 block、要不同優先權、不同的核心綁定時。其他情況放同一個框效能最好。

### 11.3 接現有的 topic：`FW_REGISTER_ROS_INPUT` / `OUTPUT`

只宣告埠的型別，**topic 名稱寫在圖上**。不用寫函式：

**`algorithms/ros_edges/ros_edges.cpp`**

```cpp
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "fw/fw.hpp"
#include "fw/ros/service.hpp"

namespace {

struct TriggerHandler {
    int n = 0;
    void serve(const std_srvs::srv::Trigger::Request&, fw::Responder<std_srvs::srv::Trigger> res) {
        std_srvs::srv::Trigger::Response r;
        r.success = true;
        r.message = "called " + std::to_string(++n);
        res.reply(r);
    }
};

// A ROS service has no error channel: this fills the response when the call does not end ok
void fill_error(fw::CallStatus s, const std::string& error, std_srvs::srv::Trigger::Response& r) {
    r.success = false;
    r.message = std::string(fw::to_string(s)) + ": " + error;
}

}  // namespace

FW_REGISTER_ROS_INPUT(temp_in, OUTPUTS(("temp", std_msgs::msg::Float64)));
FW_REGISTER_ROS_OUTPUT(temp_out, INPUTS(("temp", std_msgs::msg::Float64, trigger)));

FW_REGISTER_ROS_SERVICE_IN(trigger_in, std_srvs::srv::Trigger, &fill_error);
FW_REGISTER_ALGORITHM_CLASS(TriggerHandler, SERVES(("call", std_srvs::srv::Trigger), &TriggerHandler::serve));
```

**`config/long.yaml`**

```yaml
period_ms: 200
count: 200
```

**`graphs/ros.graph.yaml`**

```yaml
version: 1
ros_distro: humble
boxes:
  main:
    blocks:
      thermo: { driver: thermometer }
      conv:   { algorithm: to_msg }
      pub:    { ros: temp_out, topics: { temp: /temperature } }
      sub:    { ros: temp_in, topics: { temp: { topic: /external_temp, qos: { reliability: best_effort, depth: 5 } } } }
      show:   { algorithm: from_msg }
      srv:    { ros: trigger_in, service: { name: /fw_trigger, timeout: 2s } }
      handle: { algorithm: TriggerHandler }
edges:
  - config://long.yaml -> thermo.params
  - thermo.reading -> conv.in
  - conv.out -> pub.temp
  - sub.temp -> show.in
  - srv.call -> handle.call
```

種類鍵是 `ros:`，不是 `algorithm:`。在另一個終端測試：

```bash
ros2 topic echo --once /temperature
ros2 topic pub --once /external_temp std_msgs/msg/Float64 '{data: 3.5}' --qos-reliability best_effort
ros2 service call /fw_trigger std_srvs/srv/Trigger
```

```text
data: 51.0
...
from_msg: 3.5 at 1794023862.675
...
std_srvs.srv.Trigger_Response(success=True, message='called 1')
```

`topics:` 每個埠的寫法：

| 鍵 | 值 | 預設 |
|---|---|---|
| 直接寫字串 | topic 名稱 | |
| `topic` | topic 名稱 | 必填 |
| `qos` | `{ reliability: reliable / best_effort, depth: n }` | |
| `stamp`（輸入） | `header`：取 `header.stamp`，為 0 時**丟棄** | 有 header 的型別 |
| | `header_or_receive`：為 0 時改用接收時間 | |
| | `receive`：接收時間 | 沒 header 的型別 |
| `stamp`（輸出） | `envelope`：把信封時間寫入 `header.stamp` | 有 header 的型別 |
| | `message`：保留你寫入的值 | |

收到的訊息 `header.stamp` 全是 0 時資料會被丟棄並警告，因為框架不替你猜要不要改用接收時間。這時請設 `stamp: header_or_receive`。

### 11.4 ROS service

| 巨集 | 方向 | 接線 |
|---|---|---|
| `FW_REGISTER_ROS_SERVICE_IN(名字, Srv [, &on_error])` | 外部呼叫進來，轉給圖內的 `SERVES` | `srv.call -> handle.call` |
| `FW_REGISTER_ROS_SERVICE_OUT(名字, Srv)` | 圖內的 `CALLS` 轉給外部 service | `caller.x -> out.serve` |

圖上：`{ ros: 名字, service: { name: /foo, timeout: 2s, max_inflight: 4 } }`。`ros_service_in` 的 `timeout` 必填，`ros_service_out` 不能寫 `timeout` 和 `max_inflight`（那些屬於呼叫者的 `CALLS`）。

`on_error` 沒寫時，內部呼叫失敗外部呼叫者就收不到回應，`fw check` 會警告。

`FW_REGISTER_ROS_SERVICE_OUT` 本教學未實跑，有自動測試（`tests/test_ros_services.cpp`）。

### 11.5 TF

```cpp
#include "fw/ros/tf.hpp"
FW_REGISTER_TF_OUTPUT(tf_pub, INPUTS(("odom", geometry_msgs::msg::TransformStamped, trigger)));
```

```yaml
blocks:
  tf:   { ros: tf_in }           # built in: outputs tf and tf_static
  tfb:  { algorithm: tf_buffer } # built in: output buffer (fw::TfBuffer)
  pub:  { ros: tf_pub, topics: { odom: { static: true } } }
edges:
  - tf.tf -> tfb.tf
  - tf.tf_static -> tfb.tf_static
  - tfb.buffer -> loc.tf         # loc.tf: a latest port of type fw::TfBuffer
```

查詢：`const auto r = tf.lookup("map", "base_link", time); if (r) use(r.transform); else log(r.error);`。

本節未實跑，有自動測試（`tests/test_ros_tf.cpp`）。寫法請對照 [`include/fw/ros/tf.hpp`](../include/fw/ros/tf.hpp)。

### 11.6 重播 bag：`FW_REGISTER_MCAP_DRIVER`

**`drivers/temp_bag/temp_bag.cpp`**

```cpp
#include <std_msgs/msg/float64.hpp>

#include "fw/fw.hpp"
#include "fw/ros/mcap_driver.hpp"

FW_REGISTER_MCAP_DRIVER(temp_bag, OUTPUTS(("temp", std_msgs::msg::Float64)));
```

先錄一份：

**`graphs/record.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      thermo: { driver: thermometer }
      conv:   { algorithm: to_msg }
      show:   { algorithm: from_msg }
edges:
  - config://thermo.yaml -> thermo.params
  - thermo.reading -> conv.in
  - conv.out -> [show.in, out://rec.mcap]
```

```bash
fw launch graphs/record.graph.yaml
mkdir -p bags && cp out/rec.mcap bags/
fw launch graphs/replay.graph.yaml
```

**`graphs/replay.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
use_sim_time: true
boxes:
  main:
    blocks:
      bag:  { driver: temp_bag, file: bags://rec.mcap, topics: { temp: conv.out } }
      show: { algorithm: from_msg }
edges:
  - bag.temp -> show.in
```

輸出和錄的時候一樣，時間戳記也相同。`out://` 錄的 channel 名稱是來源埠（`conv.out`），一般 ROS bag 就寫 topic 名稱。

- 一定要 `use_sim_time: true`，一張圖只能有一個 mcap driver。
- `file:` 也可以是清單 `[bags://a.mcap, bags://b.mcap]`，會依時間合併播放。
- 播放設定走參數，`config://play.yaml -> bag.params`，欄位 `rate`（預設 1.0）、`lossless`（預設 false。true 時每筆等框內處理完才播下一筆，**不丟資料，評估演算法時請用它**）。

### 11.7 Tap：看框內的資料

tap 把內部輸出發佈成 ROS topic 或錄成 mcap，**只有開啟時才有序列化成本**。它只有一條規則：型別要有 `.msg` 形式或 `VIEW`。

**`algorithms/tap_demo/tap_demo.cpp`**

```cpp
#include <std_msgs/msg/float64.hpp>

#include "fw/fw.hpp"
#include "types/reading.hpp"

// An in-process type with a one-way view for ROS
struct LogState {
    double last = 0;
};
namespace {
void log_view(const LogState& s, std_msgs::msg::Float64& m) { m.data = s.last; }
}  // namespace
FW_REGISTER_TYPE(LogState, "learn/LogState", VIEW(std_msgs::msg::Float64, &log_view))

namespace {
void make_state(const Reading& in, LogState& out) { out.last = in.celsius; }
}  // namespace

FW_REGISTER_ALGORITHM(make_state, INPUTS(("in", Reading, trigger, QUEUE(16, drop_oldest))), OUTPUTS(("out", LogState)));
```

**`graphs/tap.graph.yaml`**

```yaml
version: 1
ros_distro: humble
boxes:
  main:
    tap_record: out://taps/{box}-{start}.mcap
    blocks:
      thermo: { driver: thermometer }
      conv:   { algorithm: to_msg }
      st:     { algorithm: make_state }
edges:
  - config://long.yaml -> thermo.params
  - thermo.reading -> [conv.in, st.in]
taps:
  - { output: conv.out, publish: true, record: true }
  - { output: st.out, publish: true }
```

```bash
ros2 topic echo --once /fw/tap/main/conv/out
ros2 topic echo --once /fw/tap/main/st/out
```

| 鍵 | 意思 |
|---|---|
| `output: 實例.埠` | 要看的輸出 |
| `resource: 名字` + `period: 2s` | 看資源（型別要有 `VIEW` 或 `.msg` 形式）。**`period` 必填** |
| `publish: true` | 發佈到 `/fw/tap/<框>/<block>/<埠>` |
| `record: true` | 錄進框的 `tap_record` 檔案 |

錄製檔要正常收尾才會改名成 `.mcap`。實測被 `timeout` 強制結束時，留下的是 `.mcap.partial`。

tap 的 topic `/fw/tap/...` 和跨框連線的 topic `/fw/<框>/<block>/<埠>` 不一樣。行程內型別沒有 `VIEW` 時：

```text
error: tap: output 'n.n' carries the in-process type 'fw/int32' (no .msg form); to tap it, declare a view: FW_REGISTER_TYPE(<the type>, "fw/int32", VIEW(SomeMsg, &to_msg))
```

---

## 12. 內建 block

不用寫程式，圖上直接用。**內建 block 的埠是泛型，型別由接線決定**，不做複製，也保留時間戳記。

| 名稱 | 輸入 | 輸出 | 行為 |
|---|---|---|---|
| `fw/Gate` | `in`（trigger）、`when`（bool，latest） | `out` | `when` 為 true 才轉送 |
| `fw/Switch` | `in`（trigger）、`when`（bool，latest） | `on_true`、`on_false` | 依 `when` 選一條送出 |
| `fw/Select` | `when`（bool，trigger）、`a`、`b`（latest） | `out` | `when` 到了就輸出選中的值 |
| `fw/Merge` | `a`、`b`（trigger） | `out` | 依到達順序合併。更多路就串接 |
| `fw/Add`、`Sub`、`Mul`、`Div` | `a`（trigger）、`b`（latest） | `out` | 四則運算 |
| `fw/Eq`、`Ne`、`Lt`、`Le`、`Gt`、`Ge` | `a`（trigger）、`b`（latest） | `out`（bool） | 比較 |
| `fw/And`、`Or` | `a`（trigger）、`b`（latest） | `out`（bool） | 邏輯 |
| `fw/Not` | `a`（trigger） | `out`（bool） | 邏輯反 |
| `fw/Compare` | `a`（trigger）、`k`（參數埠，欄位 `value`） | `out`（bool） | 和參數比較，圖上要寫 `op` |

數值運算只接受 `fw/int32`、`fw/int64`、`fw/float32`、`fw/float64`，`a` 和 `b` 型別必須相同，不做隱式轉換。整數除以零不輸出，回報 degraded。

**`algorithms/builtin_demo/builtin_demo.cpp`**

```cpp
#include <cstdio>

#include "fw/fw.hpp"

namespace {
void is_even(const int& n, bool& b) { b = n % 2 == 0; }
void to_double(const int& n, double& d) { d = n; }
void show_bool(const bool& b) {
    std::printf("over threshold: %d\n", b);
    std::fflush(stdout);
}
}  // namespace

FW_REGISTER_ALGORITHM(is_even, INPUTS(("n", int, trigger, QUEUE(16, drop_oldest))), OUTPUTS(("b", bool)));
FW_REGISTER_ALGORITHM(to_double, INPUTS(("n", int, trigger, QUEUE(16, drop_oldest))), OUTPUTS(("d", double)));
FW_REGISTER_ALGORITHM(show_bool, INPUTS(("b", bool, trigger, QUEUE(16, drop_oldest))));
```

**`config/threshold.yaml`**

```yaml
value: 3
```

**`graphs/builtin.graph.yaml`**

```yaml
version: 1
ros_distro: humble
mode: job
boxes:
  main:
    blocks:
      src:   { driver: counter }
      even:  { algorithm: is_even }
      gate:  { algorithm: fw/Gate, init: { when: false } }
      show:  { algorithm: print_int }
      num:   { algorithm: to_double }
      over:  { algorithm: fw/Compare, op: gt }
      flag:  { algorithm: show_bool }
edges:
  - src.n -> [even.n, gate.in, num.n]
  - even.b -> gate.when
  - gate.out -> show.v
  - num.d -> over.a
  - config://threshold.yaml -> over.k
  - over.out -> flag.b
```

- `init: { when: false }` 給 `when` 埠初值，只能用在內建 block 的 `when` 埠。
- `op:` 可以是 `lt`、`le`、`gt`、`ge`、`eq`、`ne`。
- 上面的 `gate.in` 和 `gate.when` 來自同一筆資料，但 `when` 要多經過 `is_even` 才到，所以**讀到的通常是上一筆的結果**。實跑時印出的是 `3` 和 `5` 這兩個奇數，就是這個原因。這裡只是示範接法，真的需要對齊時請用 `sync`。

環（回饋）的規則：

| 規則 | 錯誤訊息 | 改法 |
|---|---|---|
| 環上至少一條 `latest` | `R1: cycle without a latest connection (would trigger forever)` | 把環上一個埠改成 `latest` |
| 環上的 `latest` 要有初值 | `R2: latest input 'a.fb' is on a cycle and handler 'a.loop_l' needs a value to run, so start-up would deadlock` | 加 `INIT`，或參數改 `fw::Optional` |
| `sync` 不能在環上 | | 改用 `latest` |

---

## 13. 圖檔的每一個鍵

**不認得的鍵一律是錯誤**，會附行列號和 did you mean，不會被默默忽略。時間一律要帶單位（`ns`、`us`、`ms`、`s`、`m`、`h`），只寫數字是錯誤：

```text
graphs/x.graph.yaml:6:19: box 'main' stop_timeout is a duration with a unit (ns, us, ms, s, m, h), e.g. 500ms, not '5'
```

### 13.1 最外層

| 鍵 | 必填 | 意思 |
|---|---|---|
| `version` | 是 | 固定 `1` |
| `ros_distro` | 是 | `humble` |
| `boxes` | 是 | 至少一個框 |
| `edges` | | 連線 |
| `taps` | | 第 11.7 節 |
| `mode` | | `service`（預設，常駐）或 `job`（資料結束就結束） |
| `use_sim_time` | | 用模擬時間。mcap driver 必須開 |
| `cores` | | 目標機器的核心數，用來算執行緒預算警告。沒寫取本機 |

`mode: job` 接了永遠不會結束的來源時會警告。

### 13.2 框（`boxes.<名字>`）

框名只能英數字與底線，字母開頭，不能叫 `tap`。

| 鍵 | 預設 | 意思 |
|---|---|---|
| `blocks` | | 第 13.3 節。實例名稱整張圖唯一 |
| `resources` | | 第 13.4 節 |
| `workers` | 2 | 執行 handler 的執行緒數 |
| `stop_timeout` | 5s | 停止時最多等多久，超過結束碼 3 |
| `eos_timeout` | 5s | 跨框資料結束後等最後一筆多久 |
| `connect_timeout` | 30s | job 模式等其他框連上多久 |
| `stats_window` | 1s | 統計視窗 |
| `detection_min_samples` | 20 | 阻塞偵測最少樣本數 |
| `tap_record` | | tap 錄製的 `out://` 路徑 |
| `tap_queue` | 256 | tap 佇列深度 |
| `on_crash` | `report` | `report`：只回報；`restart`：重啟 |
| `restart_backoff` | | `restart` 時**必填**，重啟前等多久 |
| `max_restarts` | 無上限 | 視窗內最多重啟次數，超過就放棄 |
| `restart_window` | 全程 | 計算次數的時間視窗 |
| `priority` | | 見下 |

`priority`：

```yaml
priority: { nice: -5 }
priority: { realtime: 50, cpus: [2, 3], lock_memory: true }
```

`nice`（-20 到 19）與 `realtime`（1 到 99，軟即時）擇一。權限不足時是載入錯誤，容器要加 `--cap-add=SYS_NICE --ulimit rtprio=99`。macOS 不支援 `realtime`、`cpus`、`lock_memory`。

### 13.3 block 實例（`blocks.<名字>`）

種類鍵三選一，要和註冊方式相同。寫錯會得到 `'numbers' is a driver; write 'driver: numbers'`。

| 鍵 | 適用 | 意思 |
|---|---|---|
| `algorithm` / `driver` / `ros` | 全部 | 註冊名稱 |
| `threads` | algorithm | 覆寫 `THREADS` |
| `queue` | 輸入埠 | `{ 埠: 深度 }` 或 `{ 埠: { depth: n, policy: drop_oldest } }` |
| `max_wait` | 輸入埠 | `{ 埠: 50ms }` 或 `{ 埠: unbounded }` |
| `input_timeout` | trigger / sync 埠 | `{ 埠: 50ms }`，超時沒資料轉 degraded |
| `on_error` | | 見下 |
| `restart_backoff`、`max_restarts`、`restart_window` | `on_error: restart` | 同框的意思，`restart_backoff` 必填 |
| `topics` | `ros` 或 mcap driver | 第 11.3 節、第 11.6 節 |
| `service` | ROS service | 第 11.4 節 |
| `file` | mcap driver | `bags://x.mcap` 或清單 |
| `init` | 內建 block | `{ when: true }` |
| `op` | `fw/Compare` | 比較方式 |

**`on_error`：** block 進入 **error**（不是 degraded）時的動作。handler 拋例外只算 degraded，一個統計視窗內沒有再發生就恢復。下面這些才是 error：`ctx.report(fw::Health::error, ...)`、driver 沒呼叫 `ctx.eos()` 就返回或拋例外、`ON_DEACTIVATE` / `ON_CLEANUP` 拋例外。

| 值 | 行為 |
|---|---|
| `report` | 預設，只回報 |
| `stop_box` | 整框停止，job 模式以非零結束 |
| `restart` | 重建 block（driver 重跑 `run()`）。要設 `restart_backoff`。佇列與 latest 值保留，**成員變數重新開始** |

```text
error: block 'o': on_error: restart needs restart_backoff (how long to wait before each restart)
```

ROS 邊界 block 和宣告 `RESTART_PROCESS` 的 block 不能設 `restart`。

### 13.4 資源（`boxes.<名字>.resources.<名字>`）

| 鍵 | 意思 |
|---|---|
| `type` | `FW_REGISTER_TYPE` 的字串。一般資源必填 |
| `policy` | `writer_pref`（預設）或 `versioned` |
| `max_retained_tiles` | 只限 `versioned`，第 7.8 節 |
| `store` | `data://` 路徑，第 7.7 節 |
| `plugin` | `data://` 路徑，有這個就是 `fw::File`，第 7.11 節 |
| `create_if_missing`、`backup_on_open`、`keep` | 第 7.7 節、第 7.11 節 |
| `after_crash`、`copy_on_open`、`validate` | 只限 `fw::File`，第 7.11 節 |

### 13.5 連線（`edges`）

```yaml
edges:
  - a.out -> b.in                       # plain
  - a.out -> [b.in, c.in]               # one output to many inputs
  - config://x.yaml -> b.params         # parameters
  - config://x.yaml#/part -> b.params   # part of a file
  - vocab://w.txt -> b.words            # a plugin's output
  - map://s.bin:snapshot -> b.map       # a plugin with several outputs
  - a.out -> out://x.mcap               # write to a file
  - a.proposal -> config://x.yaml       # propose parameters
  - temps -> [a.log, b.log]             # a resource
  - temps.version -> c.v                # a resource's version port
  - ask.add -> add.add                  # a call: caller -> provider
  - config://w.yaml -> maps://s/:params # a plugin's parameters
  - { from: a.out, to: b.in, qos: { reliability: reliable, depth: 10 } }   # across boxes
  - { from: a.out, to: out://x.mcap, queue: 4096 }                         # a writer's queue
```

跨框 QoS 預設是合理值，只有要覆寫時才需要字典寫法。

### 13.6 結束碼

| 碼 | 意思 |
|---|---|
| 0 | 正常 |
| 1 | 載入或啟動錯誤 |
| 2 | 命令列參數錯誤 |
| 3 | `stop_timeout` 內停不下來 |
| 4 | job 模式有 block 進入 error |
| 5 | 崩潰且不再重啟 |

`fw launch` 回傳最差的結束碼。

---

## 14. 錯誤訊息對照表

**編譯錯誤很長，請往上找 `fw:` 開頭的那一行**，那才是真正的原因。

### 14.1 編譯時

| 訊息（`fw:` 之後） | 原因 | 改法 |
|---|---|---|
| `this parameter is bound to an input port but is an output form` | 參數順序錯 | 順序是 INPUTS → RESOURCES → OUTPUTS |
| `parameter type differs from the type of the input port` | 參數型別和埠不同 | 兩邊改成一樣 |
| `the handler's parameter count does not match its ports` | 參數數量不對，或多 handler 時 `USES` / `EMITS` 少列 | 對照第 4.3 節 |
| `this parameter is bound to an output port; it must be T&, fw::Optional<T>&...` | 輸出參數沒寫 `&` | 加 `&` |
| `a block with two or more independent trigger sources...` | 函式形式有兩個觸發來源 | 改 class |
| `block has no trigger source...` | 沒有 trigger、sync 或 PERIOD | 至少要一個 |
| `invalid trigger policy; ...` | `trigger` 拼錯，或型別有逗號 | 改拼字，或 `using` 別名 |
| `unknown clause; accepted are INPUTS, OUTPUTS...` | 子句名稱打錯（`INPUT`） | 改名 |
| `unknown port option; ...` | 埠選項打錯，或型別有逗號 | 同上 |
| `unknown handler option; expected USES(...), EMITS(...), MAX_WAIT(t) or catchup` | 打錯 | 改名 |
| `unknown block option; ...` | `OPTIONS` 內打錯 | 改名 |
| `a latest port in INPUTS is not used by any handler` | 宣告了沒人 `USES` | 加 `USES` 或刪埠 |
| `an output is not emitted by any handler` | 沒人 `EMITS` | 加 `EMITS` 或刪埠 |
| `USES names a port that does not exist in this block` | `USES` 拼錯 | 改名 |
| `with two or more data handlers, INPUTS holds only latest ports` | `trigger` 寫在 `INPUTS` | 移到 `ON` |
| `INIT applies to latest ports` | `INIT` 用在 `trigger` | 移除 |
| `FW_REENTRANT is only allowed on blocks with a single data handler` | 多 handler 宣告重入 | 拆 block |
| `type has no stable type ID` | 忘了 `FW_REGISTER_TYPE` | 加上 |
| `a resource parameter is fw::Read<T> (read), fw::Write<T> (write)...` | 參數和 `read` / `write` 不符，或用了 `&` | 第 7.4 節 |
| `NONBLOCKING applies to read resources` | `write` 加 `NONBLOCKING` | 移除 |
| `a fw::File resource is read or read_write` | `fw::File` 用了 `write` | 改 `read_write` |
| `read_write is for fw::File` | 一般資源用 `read_write` | 改 `write` |
| `a resource type is default constructible` | 資源型別沒預設建構子 | 加上 |
| `a FW_PARAMS field is a number, bool, std::string...` | 參數欄位型別不支援 | 第 6.2 節 |
| `a driver is a function void run(fw::DriverContext& ctx)` | driver 簽名錯 | 照著改 |
| `PARAMS(T) needs a structure declared with FW_PARAMS` | `T` 沒有 `FW_PARAMS` | 加上 |
| `PARAMS is the drivers' parameter port; ...` | 在 algorithm 寫 `PARAMS` | 改用 `latest` 輸入埠 |
| `FW_COW_TYPE members may only be...` | 有 `vector` 等成員 | 第 7.8 節 |
| `FW_COW_TYPE must list every member...` | 漏列或順序錯 | 全部列出，依宣告順序 |
| `a plugin's write port declares its QUEUE` | plugin 寫入埠沒寫 `QUEUE` | 加上 |

**沒有 `fw:` 的訊息：**

| 訊息 | 原因 |
|---|---|
| `explicit specialization of non-template 'n::fw::TypeName'` | `FW_PARAMS` 或 `FW_REGISTER_TYPE` 寫在 namespace 裡 |
| `no match for 'operator='`，內容有 `std::function` 和 `add_field` | `FIELD(x)` 的 `x` 不是 struct 的成員 |
| `fw manifest: cannot load ...: undefined symbol: ...` | 有宣告但沒實作，或忘了在 `fw_add_blocks(LIBRARIES ...)` 列函式庫 |
| `fw manifest: ...: duplicate block name: 'x'` | 兩個 block 註冊了同一個名字 |

### 14.2 `fw check` 或啟動時

| 訊息 | 原因 |
|---|---|
| `block type 'x' is in no package's manifest (searched: learn, fw)` | 名字錯、沒 `colcon build`，或沒 `source install/setup.bash` |
| `block 'a': 'x' is a driver; write 'driver: x'` | 種類鍵寫錯 |
| `edge 'a.m -> c.v': block 'a' has no output 'm'` | 埠名錯 |
| `edge ...: type fw/int32 does not match input type learn/Reading` | 兩端型別不同 |
| `input 'c.in' has more than one upstream connection` | 一個輸入接兩條線 |
| `unknown key 'x' in block 'y' (did you mean 'z'?)` | 鍵打錯 |
| `no filesystem plugin for wordz:// (known: ...)` | 前綴錯，或 plugin 沒編進來 |
| `no filesystem plugin for "out://` | URI 加了引號 |
| `carries the in-process type ... which cannot cross boxes` | 行程內型別跨框 |
| `R1: cycle without a latest connection` | 環上全是 trigger |
| `R2: latest input ... is on a cycle` | 環上的 latest 沒初值 |
| `... changed since its launch file was generated; regenerate it with fw launch` | 改圖之後用了舊的 launch 檔，重新 `fw launch` |

警告（不會阻止啟動，但通常代表寫錯）：

| 訊息 | 意思 |
|---|---|
| `input 'a.in' is not connected; its handler is never triggered by it` | 埠沒接 |
| `resource 'x' (box 'y') is used by no block` | 資源沒人用 |
| `R5: trigger input 'b.a' is on a cycle with queue depth 1; feedback events may be dropped` | 環上佇列太小 |
| `input 'o.n' has queue depth 1 but its upstream declares STREAM(100)` | 下游佇列太小 |

### 14.3 程式跑了但沒反應

| 現象 | 先查 |
|---|---|
| block 從未被呼叫 | `latest` 埠從沒收到值。加 `INIT` 或 `fw::Optional` |
| sync block 從未被呼叫 | 時間戳記對不上，或其中一路沒資料 |
| 資料變少 | 佇列深度。預設只有 1 |
| `printf` 沒輸出 | 沒有 `fflush(stdout)` |
| `ON_EOS` 沒被呼叫 | 上游永遠不會結束。Ctrl-C 不算資料結束 |
| job 永遠不結束 | 有 driver 沒呼叫 `ctx.eos()`，或接了 ROS 輸入 |
| 改了 YAML 沒效果 | 該參數只在 `ON_CONFIGURE` 用；或 `fw params diff` 有暫存 |
| 按 Ctrl-C 很久才停 | driver 卡在沒有逾時的 `read()` |

更多除錯方法請看 [`usage.md` 第 9 節](usage.md#9-除錯)。

---

## 15. 速查

### 哪個巨集接受哪些子句

| 子句 | `ALGORITHM` | `ALGORITHM_CLASS` | `DRIVER` | `FS_PLUGIN` |
|---|---|---|---|---|
| `INPUTS` | 全部策略 | 單一 handler：全部；多 handler：只 latest | | |
| `OUTPUTS` | ✓ | ✓ | 必填 | ✓ |
| `RESOURCES` | ✓ | ✓ | | |
| `OPTIONS` | ✓ | ✓ | | |
| `PERIOD` | ✓ | ✓（單一 handler） | | |
| `PARAMS` | | | ✓ | ✓ |
| `ON`、`ON_EOS`、`ON_CONFIGURE` 等 | | ✓ | | ✓（不接受 `USES` / `EMITS`） |
| `ON_SYNC`、`SYNC_TIMEOUT`、`ON_PERIOD` | | ✓ | | |
| `CALLS` | | ✓ | | |
| `SERVES` | | ✓ | | ✓（寫入確認） |
| `PREFIX`、`ONE_INSTANCE`、`ON_FILE_CHANGED` | | | | ✓ |

### 所有註冊巨集

```cpp
FW_REGISTER_ALGORITHM(fn, INPUTS(...), RESOURCES(...), OUTPUTS(...), PERIOD(t), OPTIONS(...));
FW_REGISTER_ALGORITHM_CLASS(Cls, INPUTS(...), RESOURCES(...), OUTPUTS(...), ON(...), ...);
FW_REGISTER_DRIVER(run, PARAMS(T), OUTPUTS(...));
FW_REGISTER_FS_PLUGIN(Cls, PREFIX("x"), OUTPUTS(...), ON_CONFIGURE(&Cls::fn), ...);
FW_REGISTER_MCAP_DRIVER(name, OUTPUTS(...));
FW_REGISTER_ROS_INPUT(name, OUTPUTS(...));
FW_REGISTER_ROS_OUTPUT(name, INPUTS((..., trigger)));
FW_REGISTER_TF_OUTPUT(name, INPUTS((..., trigger)));
FW_REGISTER_ROS_SERVICE_IN(name, Srv, &on_error);
FW_REGISTER_ROS_SERVICE_OUT(name, Srv);
FW_REGISTER_FILE_VALIDATOR("name", &fn)
FW_REGISTER_TYPE(T, "pkg/Name" [, RosMsg | VIEW(RosMsg, &fn)])
FW_PARAMS(T, GROUP("g", PERSIST(ask|never) [, APPLY(immediate)], FIELD(...)), FIELD(m, DEFAULT(v), RANGE(a, b), DESC("..."), KEY("k")))
FW_COW_TYPE(T, member, member, ...);
```

### 埠

```cpp
("名字", T, trigger [, QUEUE(n, drop_oldest|drop_newest)] [, MAX_WAIT(t|fw::unbounded)])
("名字", T, latest  [, INIT(值)])
("名字", T, sync    [, QUEUE(...)])
("名字", T [, STREAM(n)])                       // output
("名字", T [, GUARANTEED])                      // a plugin's output
("名字", T, read [, NONBLOCKING])               // resource
("名字", T, write)
("名字", fw::File, read | read_write)
("名字", Srv [, MAX_INFLIGHT(n)] [, TIMEOUT(t)])  // CALLS
("名字", Srv)                                    // SERVES
```

### handler 參數

| 埠 | 參數形式 |
|---|---|
| trigger | `const T&`、`Msg<T>`、`T` |
| latest | `const T&`、`Msg<T>`、`fw::Optional<const T&>`、`fw::Optional<Msg<T>>` |
| 輸出 | `T&`、`fw::Optional<T>&`、`Msg<T>&`、`fw::Optional<Msg<T>>&` |
| 資源 read | `fw::Read<T>` |
| 資源 write | `fw::Write<T>` |
| 資源 NONBLOCKING | `fw::Optional<fw::Read<T>>` |
| `fw::File` | `const fw::File&`（read）、`fw::File&`（read_write） |

### 指令

```text
fw new <name>                                  new project
fw scaffold <name> --kind algorithm|driver [--input p=T] [--output p=T] [--resource p=T] [--class]
fw check <graph>                               check without running
fw launch <graph>                              run
fw manifest                                    list every block (JSON)
fw params set 'config://<file>#/<key>' <value>
fw params diff | commit | discard [--group <g>]
fw data backups data://<file>
fw data restore data://<file> --backup <n>
```

`fw scaffold` 會產生骨架，看不懂的地方寫成 `TODO_` 開頭的名字，不改就編不過。例如自訂型別會變成 `TODO_learn_Reading`，資源的存取方式會變成 `TODO_Read_or_Write`。這是故意的，把它們換成真正的名字就好，`#include` 要自己補。

---

## 16. 已知問題與未實跑的部分

### 從舊版留下來的備份

macOS 上用 Docker Desktop bind mount 時，舊版 fw 做的備份在容器內顯示 `--w-------`。現在的版本可以直接還原它們，還原後的檔案權限是 `rw-------`（`0600`），因為原本的權限已經查不回來。需要別的權限時自己 `chmod` 一次。

**真正**讀不到的備份（macOS 上也是 `--w-------`，舊版還原失敗時可能留下）會回報 `Permission denied`，不會被略過。在 macOS 終端機 `chmod u+r <備份>` 就能再用。

### 本教學未實跑，只有查過程式與自動測試

| 項目 | 依據 |
|---|---|
| `FW_REGISTER_ROS_SERVICE_OUT` | `tests/test_ros_services.cpp` |
| `FW_REGISTER_TF_OUTPUT`、`tf_in`、`tf_buffer` | `tests/test_ros_tf.cpp` |
| `fw::WriteFile<T>` | `tests/test_plugin_writes.cpp` |
| `ON_PERIOD` 的 `catchup`、`PARAMS` 用在 plugin、`ONE_INSTANCE`、資源 tap 的正常流程 | 對應的 `tests/test_*.cpp` |
| `fw::WorkingSet` | 沒有教，見 plan §4.8 |
| `on_crash: restart` | [`usage.md` 第 9 節](usage.md#崩潰) 有實測輸出 |
| `priority` | `tests/test_priority.cpp` |

規則的完整定義在 `robot-framework-plan.md`，附錄 A 是 API，附錄 B 是圖的鍵，附錄 C 是所有載入檢查。
