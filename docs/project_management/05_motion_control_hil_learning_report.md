# 運動控制 HIL 開發紀錄與學習筆記

更新日期：2026-07-31  
目標平台：STM32F446RE + FreeRTOS + micro-ROS + ROS 2 Humble container

本文件整理本輪從可編譯韌體到輸出關閉 HIL 命令影子流程的實作、
量測結果與問題排除方式。它是閱讀導向的摘要；逐項驗證證據見
[`PLAN_04_AUDIT.md`](../../.agent-cowork/runs/motion-control-hil-20260731/PLAN_04_AUDIT.md)。

## 1. 目前整體進度

| 階段 | 狀態 | 已有證據 | 尚缺條件 |
|---|---|---|---|
| Phase 0：協定與單位基線 | 通過；機械常數、Nidec 13-pin 電氣介面與初始保守限制已確認 | 固定長度 ROS message、SI conversion、host tests、`24H702U030` 規格書 | 帶負載速度誤差與實機調參；現行停止限制由使用者管理 |
| Phase 1：僅遙測 HIL | 通過 | MCU state 約 49.2–49.3 Hz、無順序錯誤、UART 使用率 37.33%、Agent 穩定 | 長時間堆積記憶體壓力測試 |
| Phase 2：command shadow | 通過，輸出關閉 | session/sequence/TTL/版本驗證與真機拒絕矩陣 | 1000 次 reconnect heap soak、NaN MCU direct-injection 可另行擴充 |
| Phase 3：motor velocity | 未授權 | 軟體安全 gate 已具備 | 馬達/driver 規格、E-stop、實體測試條件及另一次 gate=1 授權 |
| Phase 4–7：關節與末端軌跡 | 軟體部分完成 | interpolator、PID/feedforward、FK/IK、tracking analyzer | 校正後實機響應、調參與 performance thresholds |
| Phase 8：系統整合 | 部分完成 | 版本化的 EF 與 Joint-PVA contract | 外部 adapter ownership 與最終 integration path |

目前燒錄在 NUCLEO-F446RE 的韌體仍以硬體輸出關閉模式運行。最後一筆
`McuState` 顯示 `safety_state=0`、`fault_code=0`、`enable_mask=0` 與三軸
`commanded_motor_rpm=0`。

## 2. 系統架構與技術選擇

```text
ROS 2 Humble container
  doghead_driver / doghead_hil
          │ DDS
micro-ROS Agent container
          │ TCP pseudo-TTY (socat)
Windows pyserial bridge ── COM6, 460800 8N1 ── ST-Link VCP
          │
STM32F446RE USART2 custom transport
  FreeRTOS CommTask (micro-ROS, 50 Hz telemetry)
  FreeRTOS ControlTask (1 kHz control and ultimate safety gate)
```

- **ROS 2 Humble**：host message、driver、HIL scenario 與測試都用 Humble
  container 建置與驗證，避免使用主機上不一致的 ROS ABI。
- **micro-ROS static library**：使用 Humble builder 重新產生 ARM library 與
  `doghead_interfaces` typesupport；這是 MCU 能正確建立 XRCE entities 的必要條件。
- **Windows COM bridge**：Windows 可直接穩定寫入 ST-Link VCP，但 WSL2/usbipd
  的 host-to-target path 不穩定。因此 Windows Python bridge 將 COM6 轉為 TCP，
  Agent container 透過 `socat` 建立 pseudo-TTY。
- **QoS 分工**：50 Hz `McuState`/`JointState` 用 Best-Effort，降低 reliable ACK
  對 UART scheduling 的影響；diagnostics、command 與 session control 保持 Reliable。
- **安全模型**：輸出 gate 預設為 0；host 的 arm/shadow/actuator-enabled 三條件與
  MCU 最終 gate 都必須同時允許，才可能碰觸 PWM。

## 3. 關鍵問題、根因與修復

### 3.1 micro-ROS library ABI 與啟動問題

**現象**：早期韌體在 micro-ROS ping 或 entity 建立時停在 `WAIT_AGENT`，或有
HardFault 風險。

**根因**：checked-in static library 與 STM32F4 的編譯 ABI/attribute 組合不相容，
且先前的 type support 產物不足以支援完整 custom interface。

**修復**：以 ROS 2 Humble static-library builder 重建，保留 ARM GCC 的必要 attribute
與 F446 編譯參數，並將自訂 `doghead_interfaces` 放入 builder 的 extra package。
之後韌體可建立完整 publisher/subscriber graph。

### 3.2 USART2 收得到發不回的 WSL2 問題

**現象**：Agent 可讀到 STM32 XRCE ping，也確實寫出 reply；STM32 卻收不到 reply。

**根因**：Windows 原生 COM6 寫入能到達 MCU，但 usbipd/WSL2 的 VCP
host-to-target forwarding 不可靠。這不是 MCU DMA parser 單獨能修好的問題。

**修復**：採用 Windows-native `pyserial` bridge（COM6 <-> TCP:8888），container
以 `socat` 連到 `host.docker.internal:8888`。這讓 Agent 的 serial I/O 最終仍從
Windows 原生 COM port 送出。

### 3.3 RX 路徑與 UART overrun

**現象**：直接逐 byte polling 時出現大量 ORE，ping 不穩定。

**修復歷程**：先做 bounded RXNE fallback，確認問題不是完全沒有 RX；接著改成
bounded burst drain，最後採 RXNE IRQ ring，保留 DMA-TX completion。現在診斷中的
UART ORE/NE/FE、DMA overflow 與 TX timeout 都可被 telemetry 監測。

**學習點**：UART 460800 時每個 byte 約 21.7 µs；若用毫秒級 task polling 逐 byte
收資料，必然漏資料。接收路徑必須在 IRQ/DMA 級別快速排空。

### 3.4 50 Hz telemetry 沒有真正到 50 Hz

**現象**：可靠 telemetry 的 ACK round-trip 延長了 scheduler，實測約 36.5 Hz。

**修復**：高頻 `McuState` 和 `/joint_states` 改為 Best-Effort，heartbeat 與
diagnostics 維持低頻/可靠；scheduler 也改為 deadline-preserving，而非每輪工作後
再 delay。

**結果**：MCU source rate 約 49.2–49.3 Hz；20.15 秒樣本為 17.20 kB/s，約為
460800 8N1 可用頻寬的 37.33%，無 reconnect、sequence loss 或 TX error。

### 3.5 Session binding 原本是 fail-open

**現象**：`RobotInterface` 的 active session 初始為 0，舊邏輯只有在 active session
非零時才檢查 session ID。因此 MCU 尚未 arm 時仍可能接受任意 session command。

**修復**：新增固定長度 `McuSessionControl`：

```text
uint16 protocol_version
uint32 session_id
bool armed
```

host 的每次 Arm 建立單調遞增、非零 epoch；MCU Reliable subscription 收到有效
`armed=true` 才綁定 session。未綁定、disarm、entity destroy、creation failure 或
Agent disconnect 時都清 session 與 command mailbox。相同 session 的重複控制訊息
為 idempotent，避免週期 status publication 不斷清掉 command。

**實機結果**：

- session=2、sequence=2、合法 joint envelope 的 command 被 MCU 接受；
- wrong session、stale sequence、protocol mismatch、TTL=0 都分別增加相對應的
  diagnostics counter；
- driver disarm 後，舊 session 被拒絕；
- stop/restart Agent 後，`agent_reconnect_total` 增加，舊 session 再次被拒絕。

### 3.6 ROS Python float32 與 JointState binding

**現象**：driver 收到正常 MCU telemetry 後崩潰，錯誤發生於 `JointState.position`。

**根因**：ROS generated `float32[]` 在 Python 端是 NumPy scalar；它不是內建
`float`。原本 finite validator 誤判，且 `sensor_msgs` binding 也不接受 NumPy scalar。

**修復**：finite validator 改用 `numbers.Real`，並在發布 JointState 前顯式執行
`float(value)`。若真的收到 NaN/Inf，driver 會 drop telemetry 並 disarm，而不是崩潰。

### 3.7 reconnect counter 被 50 Hz state 覆寫

**現象**：diagnostics 的 reconnect count 為 1 時，driver 每秒都再次 disarm，導致
不能重新 arm。

**根因**：`McuState` 沒有 reconnect counter，但 driver 每 50 Hz 呼叫 safety state
時傳入硬編碼 `0`，覆寫 diagnostics 已記住的值。

**修復**：保留 diagnostics baseline；只在 counter 真正遞增時 disarm 一次。

## 4. 如何重現建置與 HIL 驗證

### ROS 2 Humble clean build

在 `osrf/ros:humble-desktop-full` 中複製 workspace source 後執行：

```bash
source /opt/ros/humble/setup.bash
colcon build
colcon test
colcon test-result --all --verbose
```

本輪最後結果：5 個套件、31 個測試結果、0 個失敗。

### Firmware build 與 flash

使用 STM32CubeIDE bundled GNU toolchain 建置 `Firmware/doghead_arm/Debug`，再以
STM32CubeProgrammer 對已知 ST-Link serial number 進行 SWD write/verify/reset。
flash 成功不代表 micro-ROS 正常；仍須看 `McuState`、`McuDiagnostics`、CFSR 與
transport counters。

### 安全的命令影子測試順序

1. 確認 `enable_mask=0`、driver 為 disarmed。
2. 啟動 driver，等 MCU state/diagnostics 穩定。
3. 呼叫 arm service，讀取 `DriverSessionState` 與 `McuSessionControl`。
4. 用範圍內 joint position 發送合法 command；joint1 下限約為 1.396 rad，所以
   `0.1 rad` 是預期會被 envelope validation 拒絕的測試資料。
5. 分別測 wrong session、stale sequence、wrong protocol、TTL=0，讀取 diagnostics
   counter。
6. disarm，再送舊 session；必須收到 session mismatch。
7. stop/restart Agent，再送舊 session；必須仍收到 session mismatch。

## 5. 安全規則

- 不因為 command 被 MCU 接受，就代表允許馬達動作；本輪驗證所有 command 都是
  `enable_mask=0`。
- 不可在未取得硬體規格和獨立授權下把 `ACTUATOR_HARDWARE_OUTPUT_ENABLED` 改為 1。
- 進入 Phase 3 前，必須確認 E-stop、driver fault output、電機/driver 規格、encoder
  PPR、減速比、方向、零點、soft limit，以及允許的 RPM/速度/加速度/jerk。
- 應先做輸出關閉的 1000 次重連堆積記憶體壓力測試，再評估任何馬達帶電測試。

## 6. 主要檔案導覽

| 用途 | 檔案 |
|---|---|
| ROS message contract | `Software/ros2_ws/src/doghead_interfaces/msg/` |
| Host arm/session/validation | `Software/ros2_ws/src/doghead_driver/doghead_driver/driver_node.py`、`safety_validator.py` |
| HIL scenarios 與 telemetry validation | `Software/ros2_ws/src/doghead_hil/` |
| MCU session binding 與 entity lifecycle | `Firmware/doghead_arm/Middleware/MicroROS/Src/microros_app.c` |
| MCU command validation/mailbox | `Firmware/doghead_arm/Application/Src/robot_interface.cpp` |
| UART custom transport | `Firmware/doghead_arm/Middleware/MicroROS/Src/dma_transport.c` |
| Windows VCP bridge | `scripts/start_vcp_bridge.ps1`、`scripts/win_vcp_bridge.py` |

## 7. 初學者先建立的整體心智模型

這個專案同時跨越三種不同世界：

1. **ROS 2 應用世界**：Linux/container 上的 nodes、topics、services、QoS。
2. **micro-ROS 通訊世界**：把 MCU 上的精簡 ROS client 經 XRCE-DDS 連到 Agent。
3. **RTOS／硬體控制世界**：STM32 上的 task、interrupt、timer、UART、PWM、encoder。

最重要的原則是：三個世界有不同的時間尺度與失敗模式。

| 世界 | 典型時間尺度 | 主要責任 | 不能假設的事情 |
|---|---:|---|---|
| ROS 2 host | 10 ms～秒 | 規劃、驗證、監控、操作 API | Linux callback 一定準時 |
| micro-ROS communication | 1～100 ms | serialization、transport、entity lifecycle | Agent 在線就代表 MCU command 安全 |
| MCU control | µs～1 ms | feedback、local limits、watchdog、PWM gate | host 永遠連線或永遠傳正確資料 |

因此，ROS 2 負責「想做什麼」，MCU 負責「此刻是否仍可安全地做」。Linux 不應直接
承擔 1 kHz motor loop；MCU 也不應解析大型 path 或自己成為第二套 planner。

## 8. ROS 2 入門：Node、Topic、Service 與 Interface

### 8.1 Node 是什麼

ROS 2 node 是一個具有名稱、可建立 publisher/subscriber/service/timer 的程式單元。
本專案的例子：

- `doghead_driver_node`：host safety state、Arm service、command validation、telemetry conversion。
- `hil_scenario_runner_node`：產生 HIL 測試 scenario。
- `doghead_arm_node`：由 micro-ROS MCU 建立，發布 state/diagnostics 並接收 command。

Node 不等於 process。實作上可以一個 process 放一個 node，也能使用 composition
放多個 node；初學階段先採一 process 一 node，較容易看 log 與隔離崩潰。

常用觀察命令：

```bash
ros2 node list
ros2 node info /doghead_driver_node
```

### 8.2 Topic 是資料流，不是函式呼叫

Publisher 將 message 發到 topic，subscriber 非同步收到。Publisher 不知道誰接收，
也不等待對方完成控制動作。這種 decoupling 很適合 telemetry，但代表：

- 「publish 成功」只表示資料交給 ROS middleware，不代表 MCU 已套用；
- subscriber 可能晚加入、漏訊息，或因 QoS 不相容完全收不到；
- command 必須自帶 protocol、session、sequence、TTL，不能只依賴 topic 名稱。

本專案重要 topic：

| Topic | 方向 | Message | 目的 |
|---|---|---|---|
| `/doghead/mcu/state` | MCU → host | `McuState` | 50 Hz control/feedback snapshot |
| `/joint_states` | MCU/driver → ROS | `JointState` | ROS 標準 joint visualization |
| `/doghead/mcu/diagnostics` | MCU → host | `McuDiagnostics` | 低頻 counter 與健康狀態 |
| `/doghead/mcu/session_control` | driver → MCU | `McuSessionControl` | 綁定或撤銷 command epoch |
| `/doghead/mcu/joint_setpoint` | driver → MCU | `McuJointSetpoint` | 固定長度 Joint-PVA command |
| `/doghead/command/joint_setpoint` | app → driver | `McuJointSetpoint` | 經 host safety 驗證的入口 |

查看型別與資料：

```bash
ros2 topic list -t
ros2 interface show doghead_interfaces/msg/McuState
ros2 topic echo --once /doghead/mcu/diagnostics
```

### 8.3 Service 適合低頻交易

Service 有 request/response，適合 Arm、Disarm、Stop 這種操作。它不適合 100 Hz
setpoint，因為每次都等待 response，延遲與 coupling 都較高。

```bash
ros2 service list
ros2 service call /doghead/driver/arm std_srvs/srv/Trigger '{}'
ros2 service call /doghead/driver/disarm std_srvs/srv/Trigger '{}'
```

Service 回覆 `success=true` 仍不是馬達帶電授權；它只改變 driver/session state。

### 8.4 為什麼要自訂固定長度 Message

MCU RAM 小、不能接受不可預測 allocation。`float32[3]` 是固定陣列，序列化大小可預估；
`float32[]` 是動態 sequence，通常需要 capacity/data allocation。對三軸 firmware 而言，
固定陣列更安全、可測，也更容易計算 UART bandwidth。

不要用 `Float32MultiArray` 當正式 command，因為它缺少：

- protocol version；
- session/sequence/TTL；
- axis 與單位語意；
- 明確的 finite/envelope validation contract。

## 9. ROS 2 QoS 入門

QoS 是 publisher 與 subscriber 對「資料如何傳」的契約。兩端不相容時，即使 topic
名稱與 message type 正確，也可能完全收不到。

### 9.1 Reliability

- **Reliable**：middleware 嘗試重送與確認，適合 session、diagnostics、低頻 command。
- **Best-Effort**：不補舊資料，適合高頻 state，因為下一筆新 state 很快就會到。

在 serial micro-ROS 上，Reliable ACK 會消耗額外 UART frame 與 round-trip time。
兩條 50 Hz telemetry 若都 Reliable，callback/publish 可能等 ACK，實際週期被拉長。

### 9.2 History 與 Depth

`KEEP_LAST depth=N` 表示保留最近 N 筆。Control state 通常只需要最新一筆；queue
太深會讓 subscriber 在落後後處理過時資料。安全 command 更應由 MCU overwrite
mailbox 與 TTL 保護，而不是慢慢消化歷史 queue。

### 9.3 Durability

Volatile 表示新 subscriber 不會收到加入前的歷史資料。Session control 目前由 driver
週期發布，因此新建立的 MCU subscription 很快會再次收到 current state。若未來改成
Transient Local，必須重新評估 micro-ROS 支援與 stale arm 資料風險。

### 9.4 本專案 QoS 決策

```text
McuState / JointState        Best-Effort：最新值優先
Diagnostics                 Reliable：低頻 counter
SessionControl              Reliable：Arm epoch 不可隨意丟失
JointSetpoint / MotorTest   Reliable + MCU TTL/session/sequence
```

QoS 只提升傳輸語意，不是 safety mechanism。TTL 與 hardware gate 必須在 MCU 本地。

## 10. FreeRTOS 入門：Task、Priority、Tick 與 Deadline

### 10.1 Task 與 Scheduler

FreeRTOS task 類似可被 scheduler 切換的執行緒。高 priority task ready 時會先執行；
低 priority task 只有在高 priority task blocked/yield 時取得 CPU。

本專案概念上有：

```text
ControlTask  高優先權，1 kHz，做 deterministic local control
CommTask     較低優先權，約 50 Hz，做 micro-ROS spin/publish
Idle/default 最低優先權，背景與 RTOS housekeeping
```

如果 CommTask 在 serial write 卡住，不應阻止 ControlTask；反過來，ControlTask 也不應
在每個 1 ms 週期執行複雜 serialization 或 blocking I/O。

### 10.2 Tick 與 `vTaskDelayUntil`

`vTaskDelay(20 ms)` 是「工作做完後再等 20 ms」。若工作花 7 ms，週期會變 27 ms。
`vTaskDelayUntil()` 是「對齊下一個絕對 deadline」，比較適合固定頻率 task。

```text
delay-after-work:  work 7 ms + sleep 20 ms = 27 ms period
deadline-based:    every 20 ms release; work must fit inside budget
```

本專案 T26 改成 deadline-preserving scheduler，因此 telemetry 不會因單次 publish 時間
而永久漂移。

### 10.3 Deadline miss 與 WCET

- **Deadline miss**：task 在下一個 release 前尚未完成。
- **WCET**：Worst-Case Execution Time，觀察到的最長執行時間。

初學者常只看平均週期，但 real-time safety 要看 worst case。平均 0.2 ms 的 control loop
若偶爾跑 2 ms，仍會破壞 1 kHz deadline。

MCU 可用 cycle counter 或 hardware timer 在 task entry/exit 量測；量測本身也有 overhead，
要先校準，並把 maximum 和 deadline miss counter 放入 diagnostics。

### 10.4 Stack 與 Heap

每個 task 有自己的 stack。stack overflow 可能直接破壞鄰近記憶體，症狀看起來像隨機
HardFault。應監控 stack high-watermark，而不是只看目前使用量。

micro-ROS entity create/destroy 會使用 allocator。反覆 reconnect 若有 leak，free heap
會逐次下降，因此需要 1000 次 reconnect soak，而不是只測一次成功重連。

### 10.5 Queue、Mailbox 與 Overwrite Semantics

此控制應用只需要最新 command，因此 queue length=1 並 overwrite：

```text
seq 10 arrives → mailbox=10
seq 11 arrives before consume → mailbox=11
ControlTask reads → gets newest command
```

這避免堆積過時軌跡。Sequence/TTL 仍需驗證，因為 overwrite 不能阻止網路上較舊的
packet 晚到並覆蓋新 packet。

## 11. Interrupt 與 Task 的邊界

### 11.1 ISR 要短

Interrupt Service Routine 應快速讀/寫 register 或將資料放入 ring buffer，再喚醒 task。
不要在 ISR 內：

- 呼叫會 blocking 的 ROS API；
- 做大範圍浮點計算；
- 使用一般 mutex；
- `printf` 大量文字；
- 等待 UART 傳輸完成。

### 11.2 為什麼 460800 不能用毫秒 polling 收每個 byte

460800 8N1 每 byte 需要 10 bits：

```text
10 / 460800 s ≈ 21.7 µs per byte
```

1 ms 內可到約 46 bytes。如果 task 每 1 ms 才讀一個 byte，UART hardware register
早已被後續 byte 覆蓋，產生 ORE。RXNE IRQ ring 或 circular DMA 能在 byte arrival 時
快速搬走資料，task 再批次解析。

### 11.3 Critical Section 不是萬用鎖

Critical section 會暫時禁止某些 interrupt/scheduling。太長會增加 interrupt latency，
甚至造成 UART loss 或 control jitter。只應保護很短的共享狀態更新，例如：

- 更新 session ID 與 sequence baseline；
- 複製固定大小 telemetry snapshot；
- 更新 diagnostics counter。

不要把 Agent ping、serialization 或 HAL transmit wait 包進 critical section。

## 12. micro-ROS 與 XRCE-DDS 入門

### 12.1 為什麼 MCU 不直接跑完整 DDS

完整 DDS discovery、history、serialization 與 networking 對小型 MCU 太重。micro-ROS
使用 Micro XRCE-DDS client；MCU 保留精簡 client，Agent 在 Linux 端代理成 ROS 2 DDS
entities。

```text
MCU rclc publisher/subscriber
        │ XRCE session and serial frames
micro-ROS Agent
        │ DDS
ROS 2 nodes
```

所以 Agent 是 transport bridge，不是 safety controller。Agent crash 時 MCU 必須自己
timeout、disarm、clear mailbox。

### 12.2 Entity 是什麼

Support、node、publisher、subscription、executor 都是 entity/resource。建立到一半
失敗時，必須只 finalize 已成功建立的項目，並依反向順序清理。

```text
create:  support → node → publishers → subscriptions → executor handles
destroy: executor → subscriptions → publishers → node → support
```

本專案用 initialized flags 與 stage enum 紀錄進度。這讓 SWD 可以回答「失敗在哪一階段」，
也避免對未初始化 handle 呼叫 `fini()`。

### 12.3 Executor 的角色

rclc executor 維護 subscriptions/timers，`spin_some()` 時才執行 callbacks。Callback 應：

1. decode 固定 message；
2. 整包 validation；
3. overwrite mailbox；
4. 立即返回。

Callback 不應直接驅動 motor，否則 transport timing 會進入 control path。

### 12.4 Static library 為什麼容易出 ABI 問題

`libmicroros.a` 已被交叉編譯成 ARM object。以下任一不一致都可能造成 link error、
runtime crash 或序列化錯位：

- ROS distribution；
- `.msg` 定義與 generated type support；
- compiler version/flags；
- `-mfloat-abi`、FPU；
- struct packing/alignment attributes；
- transport profiles（custom transport、stream framing）；
- C/C++ runtime expectations。

因此「同為 Cortex-M4」不足以保證可直接複製 library。

## 13. Command Safety Contract 詳解

### 13.1 Protocol Version

Message layout 或語意改變時，protocol version 必須改版。MCU 若收到不相容 version，
應拒絕整包，而不是猜測欄位。

### 13.2 Session ID

Session 是一次 Arm epoch。它解決「斷線前的舊 command 在重連後復活」：

```text
Arm #1 → session 1 → commands accepted
Disarm / disconnect → active session 0
Old session 1 command arrives → reject
Arm #2 → session 2 → only session 2 accepted
```

Session setter 對相同 ID 必須 idempotent；否則 driver 每秒重發 status 時，MCU 會每秒
清空 sequence/mailbox，合法 command 反而被破壞。

### 13.3 Sequence Number

Sequence 防止 reordering/replay。比較 uint32 sequence 時要考慮 wrap-around，不能單純
`new > old` 永久使用。常用方式是將差值解讀為 signed 32-bit，只有正差才算更新。

### 13.4 TTL

TTL 是 command 在 MCU 本地仍有效的時間。應以 `xTaskGetTickCount()` 計算 receive age，
不能依賴 host timestamp，因為 PC/MCU clock 未必同步。

```text
received_at = local_tick
age = now - received_at
if age > valid_for_ms: clear command and stop output
```

### 13.5 Whole-packet Validation

先檢查全部欄位，再更新任何 state：

1. protocol；
2. active session；
3. sequence；
4. TTL；
5. mode/axis/mask；
6. array size；
7. NaN/Inf；
8. position/velocity/acceleration envelope。

若第八步失敗，前七步的資料也不得 partial apply。這稱為 transactional 或
whole-packet validation。

## 14. Telemetry 與 Diagnostics 應如何閱讀

### 14.1 State 是 snapshot

`McuState` 是某一時間點的整組快照：target、applied、measured、error、RPM、encoder、
PWM、safety、fault、sequence。相互關聯的欄位應先在 MCU 複製成 snapshot，再 publish，
避免一半是 control tick N、一半是 tick N+1。

### 14.2 Counter 是累積證據

Diagnostics counter 通常單調增加：

- `invalid_command_total`：version、finite、envelope 等錯誤；
- `stale_sequence_total`：sequence replay/out-of-order；
- `session_mismatch_total`：未綁定或錯 session；
- `command_timeout_total`：TTL 無效或已逾時；
- UART/DMA/transport counters：底層資料遺失或傳送失敗；
- `agent_reconnect_total`：Agent session 重新建立次數。

不要只看絕對值，要記錄測試前後 delta。例如 wrong-session injection 前後
`session_mismatch_total` 應剛好 +1。

### 14.3 Rate、Drop 與 Order

判斷 50 Hz 不只用 wall-clock message count。應同時檢查：

- `state_sequence` 是否單調；
- sequence gap 是否表示 drop；
- `uptime_ms` delta 是否約 20 ms；
- 是否 duplicate 或 out-of-order；
- reconnect counter 是否增加。

## 15. 分層除錯方法

遇到「ROS 收不到」時，不要直接猜 MCU callback。沿資料路徑逐層建立證據：

### Layer 1：電氣與 peripheral

- board voltage、COM port 是否存在；
- UART baud/8N1 是否兩端一致；
- USART RXNE/ORE、DMA NDTR、IRQ enable；
- SWD CFSR/HardFault state。

### Layer 2：transport bytes

- MCU `tx_bytes/tx_frames` 是否增加；
- Windows bridge 是否讀到 serial bytes；
- Agent 是否 read request / write reply；
- MCU `rx_bytes` 是否增加。

### Layer 3：XRCE session

- ping 是否成功；
- state machine 是 WAIT、CREATE 還是 RUNNING；
- entity stage/last error；
- Agent log 是否顯示 create client/entities。

### Layer 4：ROS graph 與 QoS

- `ros2 node list` 是否看見 MCU node；
- `ros2 topic list -t` type 是否一致；
- publisher/subscriber QoS 是否相容；
- overlay 是否 source 到正確 install prefix。

### Layer 5：application contract

- protocol/session/sequence/TTL；
- finite/limit/mask；
- diagnostics 哪個 rejection counter 增加；
- driver 是否仍 armed、shadow/gate 是否符合預期。

一次只變更一層。若 Agent 已證實寫出 42-byte reply，而 MCU RX counter 不動，就應查
transport/host forwarding，不應先改 ROS message callback。

## 16. Container 與 Overlay 初學注意事項

ROS 2 workspace build 後要 source `install/setup.bash`，否則 CLI 不知道自訂 package：

```bash
source /opt/ros/humble/setup.bash
source /path/to/workspace/install/setup.bash
```

若 install tree 被複製到另一個 path，colcon generated script 可能記住 build-time prefix。
可在受控環境設定 `COLCON_CURRENT_PREFIX`，但正式部署更建議在固定 prefix 建置/install，
避免 runtime overlay 難以追蹤。

Container 的好處：

- 固定 Ubuntu/ROS distribution；
- 減少 host package 汙染；
- CI 與開發機較容易重現。

但 container 不會自動解決 USB/serial ownership、Windows/WSL forwarding 或 real-time
scheduling。這些仍需要明確 bridge 與量測。

## 17. 初學者實作練習

### 練習 1：只觀察 graph

1. 啟動 Agent，不啟動 driver。
2. 列出 node/topic/type。
3. 分辨哪些 publisher 來自 MCU。
4. 記錄 McuState 與 Diagnostics QoS。

### 練習 2：量測 telemetry

1. 收集 10 秒 `state_sequence` 與 `uptime_ms`。
2. 算平均/最小/最大 interval。
3. 數 gap、duplicate、out-of-order。
4. 比較 wall clock rate 與 MCU source rate 的差異。

### 練習 3：安全 rejection matrix

保持 `enable_mask=0`：

1. 未 arm 送 command，預期 session mismatch。
2. arm 後送合法 envelope，預期 accepted sequence 更新。
3. 重送相同 sequence，預期 stale +1。
4. 改錯 session/version/TTL，分別確認 counter delta。
5. disarm 後重送舊 session，預期 mismatch +1。

### 練習 4：估算 UART 頻寬

對每種 message 估算 serialized bytes × Hz，再加 XRCE framing/ACK margin。8N1 可用：

```text
usable_bytes_per_second = baud / 10
utilization = observed_bytes_per_second / usable_bytes_per_second
```

比較 115200 與 460800，解釋為何本專案不能把完整 50 Hz telemetry 放在 115200。

### 練習 5：讀 code 找責任邊界

追蹤一筆 Joint-PVA command：

```text
scenario/app publisher
→ doghead_driver host validation
→ MCU topic
→ micro-ROS callback
→ RobotInterface whole-packet validation
→ overwrite mailbox
→ ControlTask / safety gate
```

對每一層回答：它驗證什麼？失敗時 counter/狀態如何改變？會不會直接碰 hardware？

## 18. 常見名詞表

| 名詞 | 初學解釋 |
|---|---|
| HIL | Hardware-in-the-Loop；真硬體參與，但可把 actuator output 關閉 |
| RTOS | 提供 priority、task、tick、queue 等 deterministic scheduling 工具 |
| ISR | 硬體 interrupt 發生時執行的短函式 |
| WCET | 觀察或分析出的最長執行時間 |
| DDS | ROS 2 底層資料分發 middleware |
| XRCE-DDS | 適合 resource-constrained client 的 DDS 代理協定 |
| Agent | 把 micro-ROS XRCE client 映射到 ROS 2 DDS graph 的 Linux process |
| QoS | Reliability、history、durability 等傳輸契約 |
| ABI | 編譯後 binary 如何傳參數、排 struct、使用 FPU 的規則 |
| TTL | command 從本地接收時刻起可存活多久 |
| Session epoch | 一次 Arm 的身份；disconnect/disarm 後舊 epoch 無效 |
| Fail-closed | 缺資料、未綁定或出錯時預設拒絕，而不是繼續動作 |
| Mailbox | 只保存最新一筆 command 的 queue/storage |
| Soak test | 長時間或大量重複操作，用來找 leak、race 與偶發錯誤 |

## 19. 下一步學習與開發順序

1. 先熟悉 ROS 2 graph、topic/service 與 QoS，不碰 actuator gate。
2. 理解 FreeRTOS task/ISR/queue/tick，能說明 1 kHz control 與 50 Hz comm 的分工。
3. 用 diagnostics 做輸出關閉的故障注入。
4. 完成 1000 次 Agent reconnect heap soak 與 WCET instrumentation。
5. 取得並驗證全部硬體常數與 E-stop path。
6. 另行審查／授權 gate=1 專用映像，再做單軸無負載低速啟動。

在第 6 步以前，所有工作都可以且應該維持 actuator output disabled。
