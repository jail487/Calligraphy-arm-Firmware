# 三軸運動控制與 ROS 2 HIL 平台詳細執行計畫

> [!IMPORTANT]
> 執行狀態更新：Phase 0–2 已以 NUCLEO-F446RE gate=0 HIL 驗證；ROS 2 Humble
> 乾淨建置為 5 個套件／31 個測試結果。三軸機械參數已由使用者依原始韌體設定確認；
> Phase 3 仍等待驅動器／E-Stop 條件與馬達帶電測試授權。

**文件類型**：Execution Plan / Interface Baseline  
**版本**：v1.0  
**日期**：2026-07-31  
**適用平台**：STM32F446RET6、FreeRTOS、micro-ROS、ROS 2 Humble、WSL2  
**上位機暫定環境**：Ubuntu 22.04 + ROS 2 Humble  
**前置基線**：USART2 DMA Transport、micro-ROS Agent 連線狀態機、50 Hz `/joint_states` 遙測已建立  
**安全規範**：必須遵守 `.agents/AGENTS.md`

---

## 1. 文件目的

本文件定義目前個人負責範圍內，三軸書法機器臂之韌體、ROS 2 最小上位機、通訊介面、Hardware-in-the-Loop（HIL）測試與控制品質驗證的詳細執行計畫。

現階段不以整合完整書法字體或筆觸演算法為目標，而是先建立一套：

1. 可接收上位機演算法產生的控制參考。
2. 可將 End-Effector（EF）參考轉換成三軸 Joint setpoint。
3. 可透過 micro-ROS 安全地傳送控制與遙測資料。
4. 可由 STM32 以確定性週期執行 Joint servo。
5. 可量測並量化位置、速度、延遲、jitter 與安全停止表現。
6. 未來可在不重寫 MCU 底層介面的前提下，接入隊友的書法路徑軟體。

本階段的核心成果不是「能畫字」，而是：

> 建立一套可被任意上位機演算法驅動、能安全調校、能量測追蹤誤差、並具備清楚 Interface Contract 的三軸運動控制平台。

---

## 2. 現階段前提與限制

### 2.1 已確認前提

- MCU 為 STM32F446RET6。
- 控制迴圈由 TIM6 觸發，ControlTask 以 1 kHz 執行。
- 通訊使用 USART2 DMA + micro-ROS Agent。
- ROS 2 使用 Humble。
- 三顆馬達目前設定為 Nidec 13-pin frequency-command 類型。
- 馬達驅動板以輸入脈波／PWM frequency 表示 motor-shaft speed reference。
- 三軸具備 Encoder 回授，可取得位置與估算速度。
- Nidec 規格書 `24H702U030`（Drawing `3GSPC22Z013`）已提供 13-pin 介面基線：
  `STMP` 為上升緣頻率輸入，`START` 為高有效運轉許可，`CW/CCW` 為方向輸入，
  `ENC_A/ENC_B` 為 100 pulse/revolution 的正交輸出，電源為 `VM=DC24V±10%`、
  邏輯電源 `5.0V±10%`。
- 使用者已確認原始韌體的三軸機械基線：gear ratio 均為 `114.0`、Encoder PPR 均為
  `100`、quadrature factor 均為 `4`；J1/J2/J3 的 motor direction inversion 為
  `true/false/false`，三軸 `encoder_inverted` 均為 `false`。
- 原始韌體的 home 零點為 J1=`115°`、J2=`180°`、J3=`0°`；這些值由
  `homing_calibration.cpp` 的 `HOME_THETA*_DEG` 定義。
- micro-ROS 已可發布三軸 `/joint_states`。
- Agent 斷線、Entity destroy 與 command invalidation 已有安全基線。

### 2.2 隊友軟體目前的已知輸出

隊友目前的路徑資料只包含：

```text
type = travel | write
x
y
z
```

其意義暫定為筆尖／End-Effector 的 Cartesian waypoint，但尚未完成下列正式定義：

- frame_id
- SI 單位
- End-Effector orientation
- velocity
- acceleration
- jerk
- time_from_start
- Joint trajectory
- micro-ROS 或 MCU command

因此本計畫不得假設隊友已經提供可直接控制馬達的資料。

### 2.3 本階段不納入範圍

下列功能不屬於本階段必要交付：

- 字體解析。
- SVG 或影像轉書法路徑。
- 筆順生成。
- BBSMG 正式整合。
- 完整筆觸／毛筆模型。
- 視覺辨識。
- 自動紙面標定。
- 完整 MoveIt 設定。
- production ros2_control plugin。
- 正式 UI。
- 高速書寫。
- 最終產品級 trajectory execution。

這些功能不得阻擋馬達、通訊、Joint tracking 與 EF tracking 測試平台的建立。

---

## 3. 系統目標與責任邊界

### 3.1 目標資料流

```text
測試軌跡產生器                     未來書法演算法
Step / Ramp / Sine / Line          EF waypoints / path
          │                              │
          └────── EndEffectorReference ──┘
                              │
                              ▼
                    ROS 2 Motion Adapter
               frame / unit / IK / constraints
                  velocity / acceleration / jerk
                              │
                              ▼
                       Joint PVA Setpoint
                              │
                              ▼
                       ROS 2 MCU Driver
                  session / sequence / TTL / Arm
                              │
                              ▼
                    micro-ROS Agent + UART DMA
                              │
                              ▼
                      STM32 Command Decoder
                              │
                              ▼
             Local Interpolator + Safety Rate Guard
                              │
                              ▼
                1 kHz Joint Servo + Ultimate Gate
                              │
                              ▼
              motor RPM → PWM frequency → Motor
                              │
                              ▼
                           Encoder
                              │
                              ▼
        FirmwareState / Diagnostics / `/joint_states`
```

### 3.2 ROS 2 上位機主責

- 接收測試軌跡或未來隊友的 EF reference。
- 定義並處理 paper、base、tool 等 frame。
- 將所有外部單位轉換成 ROS SI。
- 執行 FK／IK。
- 檢查 workspace、Joint limit 與 singularity。
- 產生 velocity／acceleration／jerk 受限軌跡。
- 將 EF reference 轉換成 Joint PVA setpoint。
- 管理 Arm Session、sequence 與 command TTL。
- 記錄 rosbag2／CSV。
- 計算 tracking error、latency、jitter 與控制品質。
- 提供 HIL 測試、故障注入與報告工具。

### 3.3 STM32 Firmware 主責

- 1 kHz Encoder feedback acquisition。
- Joint position／velocity estimation。
- 固定大小 command decode 與 validation。
- Command mailbox、session、sequence 與 TTL。
- 1 kHz setpoint interpolation。
- 最終 velocity／acceleration／jerk safety guard。
- Joint feedback servo。
- motor-shaft RPM 與 PWM frequency 轉換。
- Motor direction、start、stop 與實體隔離。
- Homing、Limit、E-Stop、Watchdog 與 Fault latch。
- Ultimate Safety Gate。
- 固定大小 telemetry snapshot 與 diagnostics counter。
- Agent 斷線後獨立安全停止。

### 3.4 不允許跨越的責任邊界

- MCU 不解析 JSON、字體、SVG 或長 Cartesian path。
- MCU 不直接接收未驗證的 EF `x/y/z` 作為 actuator command。
- ROS 2 不直接控制 PWM register。
- Linux、DDS 或 Agent 不得成為唯一安全控制器。
- ROS 2 傳入的 limit 不得放寬 Firmware hard limit。
- Firmware production command path 不得同時存在兩個 trajectory owner。
- Bring-up motor RPM command 不得借用 Joint position 欄位。

---

## 4. 單位與座標契約

### 4.1 ROS 2 邊界單位

| 物理量 | ROS 2 單位 |
|---|---|
| Cartesian position | m |
| Cartesian velocity | m/s |
| Cartesian acceleration | m/s² |
| Cartesian jerk limit | m/s³ |
| Joint position | rad |
| Joint velocity | rad/s |
| Joint acceleration | rad/s² |
| Joint jerk limit | rad/s³ |
| Time | s、ns 或明確的整數 ms/us |

### 4.2 Firmware 內部單位

Firmware 內部可暫時保留既有 deg、deg/s 與 motor RPM，但必須符合：

1. 每個 API 名稱或註解明確標示單位。
2. 單位轉換集中在單一模組。
3. 不能讓 `rpm` 同時代表 motor-shaft RPM 與 Joint output RPM。
4. 通訊邊界只能使用 ROS SI。
5. 所有換算函式必須有 host-side unit test。

### 4.3 馬達與 Joint 換算

```text
joint_rps       = joint_rad_s / (2π)
joint_rpm       = joint_rad_s × 60 / (2π)
motor_rpm       = joint_rpm × gear_ratio
command_freq_hz = motor_rpm × pulses_per_motor_rev / 60
```

反向回授：

```text
joint_revolutions =
    encoder_count /
    (encoder_ppr × quadrature_factor × gear_ratio)

joint_position_rad = joint_revolutions × 2π
```

### 4.4 座標系暫定規則

第一版至少保留以下 frame 名稱：

```text
base_link
tool0
brush_tip
paper
```

在正式 EF 測試前必須完成：

- `base_link` 原點與軸向定義。
- `brush_tip` 相對於最後一軸的 TCP offset。
- `paper` 相對於 `base_link` 的固定轉換。
- 隊友資料的 `x/y/z` 單位與 frame adapter。

---

## 5. 控制模式

Firmware 使用單一 enum 管理 command semantics：

```c
typedef enum {
    CONTROL_MODE_DISABLED = 0,
    CONTROL_MODE_MOTOR_VELOCITY_TEST = 1,
    CONTROL_MODE_JOINT_PVA = 2,
    CONTROL_MODE_HOLD = 3,
    CONTROL_MODE_STOP = 4
} RobotControlMode_t;
```

### 5.1 `CONTROL_MODE_DISABLED`

- 不允許 actuator output。
- 清除 command mailbox。
- 清除 interpolator target。
- 清除 cached command。
- 重設 `has_command`。
- Motor 實體停止。

### 5.2 `CONTROL_MODE_MOTOR_VELOCITY_TEST`

- 僅供 Bring-up。
- 一次只能使能一軸。
- 使用獨立 `axis` 與 `target_motor_rpm`。
- 未被選擇的軸必須 `Motor_Stop()`。
- 使用 Firmware hardcoded RPM、acceleration 與 jerk limit。
- 方向反轉前必須先降至零。

### 5.3 `CONTROL_MODE_JOINT_PVA`

- 接收三軸 position／velocity／acceleration reference。
- 所有值必須 finite。
- 必須通過 Joint envelope。
- 由 MCU 1 kHz interpolator 產生當前 setpoint。
- velocity／acceleration 為 feedforward reference，不得繞過 position safety。
- local rate guard 必須在 actuator output 前再次驗證。

### 5.4 `CONTROL_MODE_HOLD`

- 將當前 Encoder position 鎖定為 hold target。
- 清除未執行 setpoint。
- 不得恢復舊 trajectory。

### 5.5 `CONTROL_MODE_STOP`

- 明確停止命令。
- 清除舊命令、interpolator 與 Arm lease。
- 根據停止原因選擇 controlled stop 或 immediate disable。

---

## 6. Interface Contract v0.1

### 6.1 Topic namespace

本專案第一版使用：

```text
/doghead/mcu/motor_test_command
/doghead/mcu/joint_setpoint
/doghead/mcu/session_control
/doghead/mcu/state
/doghead/mcu/diagnostics
/doghead/mcu/safety_status
/joint_states
```

未來與隊友整合時，優先使用 ROS remapping 或 adapter，不直接破壞 MCU topic contract。

### 6.2 `McuMotorTestCommand.msg`

```text
uint16 protocol_version
uint32 session_id
uint32 sequence
uint16 valid_for_ms

uint8 axis
float32 target_motor_rpm
float32 requested_accel_rpm_s
float32 requested_jerk_rpm_s2
bool enable
```

驗證規則：

- `axis` 必須為 0、1 或 2。
- 所有 float 必須 finite。
- `valid_for_ms` 不得為 0，且不得高於 Firmware 上限。
- requested acceleration／jerk 只能被 Firmware 進一步縮小。
- `target_motor_rpm` 必須受 Bring-up hard limit 約束。
- `enable == false` 必須清除此模式的 cached command。
- production build 可完全停用此 Topic。

### 6.3 `McuJointSetpoint.msg`

```text
uint16 protocol_version
uint32 session_id
uint32 sequence
uint16 valid_for_ms
uint8 enable_mask

float32[3] position_rad
float32[3] velocity_rad_s
float32[3] acceleration_rad_s2
```

驗證規則：

- `protocol_version` 必須完全相符。
- `session_id` 必須是目前已 Arm 的 session。
- `sequence` 必須符合 wrap-safe 的單調遞增規則。
- `valid_for_ms` 使用 MCU local FreeRTOS tick 計算。
- 任一欄位 invalid，整包拒絕，不得 partial apply。
- 任一 position 超出 hard joint envelope，整包拒絕。
- 任一 velocity／acceleration 超出 hard limit，整包拒絕或 clamp 的策略必須固定，不可混用。
- `enable_mask` 不得使能未 homed 或已 fault 的軸。

第一版不傳逐點 jerk。Jerk 由上位機 motion generator 約束，MCU 再執行 local rate guard。

### 6.4 `McuState.msg`

```text
uint16 protocol_version
uint32 boot_id
uint32 state_sequence
uint32 uptime_ms

uint8 control_mode
uint8 safety_state
uint16 fault_code
uint8 enable_mask
bool agent_connected

float32[3] target_position_rad
float32[3] target_velocity_rad_s
float32[3] target_acceleration_rad_s2

float32[3] applied_position_rad
float32[3] applied_velocity_rad_s

float32[3] measured_position_rad
float32[3] measured_velocity_rad_s
float32[3] position_error_rad

float32[3] commanded_motor_rpm
int64[3] encoder_count
uint32[3] pwm_frequency_hz

uint16 command_age_ms
uint32 last_accepted_command_sequence
uint32 rejected_command_total
```

### 6.4.1 `McuSessionControl.msg`

```text
uint16 protocol_version
uint32 session_id
bool armed
```

- `doghead_driver` publishes this reliable control topic periodically.
- `armed=false` or `session_id=0` clears the MCU mailbox and sequence baseline.
- Each Arm creates a nonzero, monotonic session epoch. MCU commands must match
  that epoch exactly; repeated messages for the same epoch are idempotent.

### 6.5 `McuDiagnostics.msg`

```text
uint32 control_deadline_miss_total
uint32 control_wcet_cycles
uint32 command_timeout_total
uint32 invalid_command_total
uint32 stale_sequence_total
uint32 session_mismatch_total

uint32 uart_ore_total
uint32 uart_ne_total
uint32 uart_fe_total
uint32 dma_rx_overflow_total
uint32 transport_tx_timeout_total
uint32 agent_reconnect_total

uint32 free_heap_bytes
uint32 minimum_ever_free_heap_bytes
uint32 control_stack_high_watermark
uint32 comm_stack_high_watermark
```

### 6.6 `EndEffectorReference.msg`

此訊息只存在 ROS 2 上位機，不傳入 STM32：

```text
std_msgs/Header header

uint8 mode
geometry_msgs/Point position
geometry_msgs/Vector3 linear_velocity
geometry_msgs/Vector3 linear_acceleration
```

第一版允許 orientation 尚未實作，但必須在文件中明確標記為「固定工具姿態」，不得默認忽略。

### 6.7 Protocol versioning

- 第一版 `protocol_version = 1`。
- Message 欄位順序或語意改變時必須增加版本。
- Firmware 與 ROS driver 版本不符時只能進入 telemetry／diagnostic，不得 Arm。
- Firmware build 應回報 git commit 或 firmware version。
- 不允許以註解默認兩端版本相容。

---

## 7. 通訊、更新率與時間策略

### 7.1 第一版更新率

| 資料 | 目標頻率 |
|---|---:|
| STM32 ControlTask | 1000 Hz |
| Joint setpoint | 100 Hz |
| Motor test command | 20–50 Hz |
| McuState | 50 Hz |
| McuDiagnostics | 1–5 Hz |
| Agent ping | 1 Hz |

### 7.2 Command TTL

- 第一版 command timeout：250 ms。
- TTL 一律使用 MCU `xTaskGetTickCount()`。
- PC timestamp 只供記錄與 latency 分析，不作唯一安全判斷。
- TTL 到期後必須清除 command、cached state、interpolator 與 `has_command`。
- 重連後不得自動恢復前一個 session。

### 7.3 第一版 setpoint 策略

第一版採「最新 setpoint + local interpolation」：

- ROS driver 以約 100 Hz 發送 PVA setpoint。
- MCU 以 1 kHz 套用插值與 local guard。
- Command Queue 維持 overwrite semantics。
- 不在第一版實作大型 trajectory queue。

若量測發現 Linux／UART jitter 無法達到 tracking 需求，再升級為固定大小短 segment buffer。不得在沒有量測證據前增加複雜度。

### 7.4 UART bandwidth gate

- 115200 8N1 理論上限約 11,520 bytes/s。
- steady-state 平均使用率目標不超過 50%。
- 短暫 peak 不超過 70%。
- diagnostics 與 state 分頻發布。
- binary micro-ROS UART 禁止混入 `printf`。
- 若頻寬不足，依序採取：
  1. 減少 telemetry 欄位或分頻。
  2. 提升 baud rate 並完成 soak test。
  3. 改用 USB CDC。

---

## 8. ROS 2 最小上位機套件規劃

```text
Software/
└── ros2_ws/
    └── src/
        ├── doghead_interfaces/
        ├── doghead_driver/
        ├── doghead_motion/
        ├── doghead_hil/
        └── doghead_bringup/
```

### 8.1 `doghead_interfaces`

交付內容：

- `.msg` 定義。
- protocol version 常數。
- safety state 與 fault code 定義。
- message validation test vectors。
- micro-ROS static library build input。

完成條件：

- ROS 2 可 clean build。
- micro-ROS 可生成相同 type support。
- fixed array 不在 MCU steady state 配置 heap。
- host serialization round-trip test 通過。

### 8.2 `doghead_driver`

第一版功能：

- 訂閱 `McuState` 與 diagnostics。
- 發布 `/joint_states`。
- 建立 session／sequence／TTL。
- 提供 Arm、Disarm、Stop 的 host-side API。
- 發送 `McuMotorTestCommand` 與 `McuJointSetpoint`。
- 檢查 firmware protocol version。
- 檢查 NaN／Inf、joint name、array size 與 limit。
- Agent reconnect 後強制 Disarmed。
- 提供 command shadow log。

第一版可以是一般 ROS node；安全與 reconnect 穩定後再升級 Lifecycle Node。

### 8.3 `doghead_motion`

第一版功能：

- FK。
- IK。
- Joint limit。
- workspace validation。
- singularity／unreachable detection。
- Cartesian line interpolation。
- Joint trapezoidal profile。
- Joint S-curve profile。
- velocity／acceleration／jerk constraint。
- EF reference → Joint PVA。

第一版只支援固定工具姿態與測試路徑，不實作字體。

### 8.4 `doghead_hil`

測試節點：

- `telemetry_validator`
- `motor_step_test`
- `motor_ramp_test`
- `motor_reverse_test`
- `joint_step_test`
- `joint_sine_test`
- `cartesian_line_test`
- `agent_disconnect_test`
- `command_timeout_test`
- `stale_command_test`
- `tracking_report`

所有會動馬達的節點預設：

- `enable=false`
- 單軸
- 低速
- 必須明確 Arm
- 必須提供 timeout
- 測試結束必須 Stop／Disarm

### 8.5 `doghead_bringup`

交付內容：

- micro-ROS Agent launch／script（含 Linux 直連模式與 Windows ST-Link VCP TCP Bridge 雙向轉接模式 `scripts/start_vcp_bridge.ps1`，詳見 `docs/ros2_integration/02_wsl2_usb_and_agent_setup_guide.md`）。
- 僅遙測啟動檔。
- 單軸啟動檔。
- Joint PVA launch。
- EF HIL launch。
- YAML limit 與 test scenario。
- rosbag2 recording profile。

Firmware hard limit 不得從 YAML 取得或被 YAML 放寬。

---

## 9. Firmware 工作分解

### F0：單位與硬體契約

- [x] 建立 motor-shaft RPM、Joint RPM、deg/s、rad/s 的明確型別或命名。
- [x] 盤點三軸 gear ratio（使用者確認：三軸皆 `114.0`）。
- [x] 盤點 Encoder PPR 與 quadrature factor（使用者確認：`100`、`4`）。
- [x] 盤點三軸方向與 encoder inversion（motor inversion=`true/false/false`；encoder inversion 全為 `false`）。
- [x] 驗證 `Motor_SetSpeed()` 的 RPM 語意。
- [x] 驗證 `Motor_GetVelocity()` 的 RPM 語意。
- [x] 建立 conversion unit test。
- [ ] 確認 100 Hz minimum command frequency 是硬體限制或舊軟體限制。

### F1：Feedback estimator

- [x] 將 Encoder raw count 與 Joint position conversion 分離。
- [x] 記錄原始速度與濾波速度。
- [ ] 評估目前 10 ms velocity update 是否足夠。
- [x] 建立低速 count quantization 測試。
- [ ] 避免直接對含噪 velocity 二次差分計算 jerk。
- [ ] 將 estimator latency 納入 tracking report。

### F2：Command model

- [x] 以 enum 取代模糊 bool 模式組合。
- [x] 分離 Motor Test 與 Joint PVA payload。
- [x] 新增 protocol version。
- [x] 新增 session ID。
- [x] 新增 sequence。
- [x] 新增 `valid_for_ms`。
- [x] 實作 whole-packet validation。
- [x] 實作 invalid command counter。

### F3：Setpoint interpolator

- [x] 1 kHz 保存與更新 PVA setpoint。
- [x] arrival-based 第一版 interpolation。
- [x] velocity limit。
- [x] acceleration limit。
- [x] jerk/rate guard。
- [x] direction reversal zero-crossing。
- [x] timeout 時清除所有 interpolation state。
- [x] session change 時清除所有 interpolation state。

### F4：Joint servo

- [x] 明確定義 position feedback output 單位。
- [x] 實作 velocity feedforward。
- [x] acceleration feedforward 先設為可關閉。
- [ ] 量測驅動板速度 response。
- [ ] 根據量測決定是否增加 external velocity correction。
- [x] 實作 saturation 與 anti-windup。
- [x] 避免 control mode 切換時殘留 integral。

### F5：Actuator output

- [x] 單一模組成為 `Motor_*` production owner。
- [x] finite validation。
- [x] hardcoded motor RPM cap。
- [x] enable mask。
- [x] Safety state check。
- [x] `RobotSafety_CanDrive()` 緊鄰硬體 API。
- [x] fault／timeout 時立即 return，不使用無限 polling。

### F6：micro-ROS Entity

- [x] 新增固定大小 command subscription。
- [x] 新增 state publisher。
- [x] 新增 diagnostics publisher。
- [x] 建立 create order。
- [x] 建立 reverse destroy order。
- [x] callback 不直接操作 Motor。
- [x] callback 只做 decode、validate 與 overwrite mailbox。
- [ ] reconnect 1000 次 heap drift test。

### F7：Telemetry

- [x] target PVA。
- [x] applied PVA。
- [x] measured position／velocity。
- [x] position error。
- [x] commanded motor RPM。
- [x] raw Encoder count。
- [x] PWM frequency。
- [x] command age。
- [x] safety／fault。
- [ ] deadline miss／WCET。
- [x] UART／DMA counters。

---

## 10. 分階段執行與驗收 Gate

### Phase 0：單位、硬體與 Protocol Baseline

**執行狀態**：已通過；機械常數已依原始韌體設定由使用者確認。帶電測試前仍須確認驅動器規格、E-Stop 與負載條件。

**供電狀態**：馬達不得通電。

工作：

- 完成 F0。
- 建立 `doghead_interfaces` skeleton。
- 固化 Joint name、axis order、gear ratio 與方向。
- 固化 protocol version 1。
- 建立 host-side serialization test。

Gate：

- Firmware clean build：0 error、0 warning。
- ROS workspace clean build。
- 所有 unit conversion test 通過。
- malformed、NaN、Inf、version mismatch 被拒絕。
- 不產生任何 actuator output。

### Phase 1：Telemetry-only HIL

**執行狀態**：已通過輸出關閉 HIL；49.2–49.3 Hz、無序號遺失，
短期 reconnect 與 UART counters 正常。1 小時/1000 次 soak 仍列為長期測試。

**供電狀態**：Motor output disabled。

工作：

- 實作 `McuState` 與 diagnostics。
- 實作 `doghead_driver` telemetry path。
- 將 raw state 轉成 `/joint_states`。
- 建立 rosbag2／CSV recorder。
- 手動轉軸驗證方向與單位。

Gate：

- 正常連線時 `/joint_states` 目標 50 Hz。
- 所有 Joint name 與 axis order 正確。
- 手動轉動方向與 ROS 顯示一致。
- Agent restart 不影響 1 kHz ControlTask。
- 1 小時 telemetry soak 無 heap drift、RX overflow。

### Phase 2：Command shadow mode

**執行狀態**：已通過輸出關閉 HIL；合法命令可接受，錯誤 session、
stale sequence、wrong version、TTL=0、disarm 與 Agent reconnect 皆已驗證 fail-closed。

**供電狀態**：Motor output disabled。

工作：

- 建立 command subscriptions。
- 完成 session／sequence／TTL validation。
- callback 寫入 mailbox。
- ControlTask 計算 applied command，但不呼叫 Motor。
- 發布 accepted／rejected command 與原因。

Gate：

- 過期 TTL、舊 sequence、錯 session、錯 version、NaN/Inf 全部拒絕。
- 拒絕不得 partial apply。
- Agent disconnect 必須 annihilate command。
- reconnect 後沒有新 Arm 不得接受運動命令。

### Phase 3：單軸 Motor Velocity Bring-up

**執行狀態**：未授權。不得將已完成的命令影子流程誤認為馬達帶電 GO。

**供電狀態**：Conditional GO。

條件：

- 單軸。
- 無負載。
- 低速 10–50 motor RPM。
- 其他軸 `Motor_Stop()`。
- 實體 E-Stop 可用。
- 人員可立即斷電。

工作：

- Motor step。
- Motor ramp。
- S-curve RPM。
- 正反轉。
- 驗證 minimum frequency。
- 量測 frequency command → measured speed response。

Gate：

- enable mask 隔離正確。
- 無瞬間反向。
- 250 ms timeout 內觸發安全停止流程。
- Agent kill 後停止。
- commanded RPM、measured RPM、PWM frequency 可同步記錄。
- 最壞停止時間有實測數據。

### Phase 4：單軸 Joint Position PVA

工作：

- 完成 interpolator。
- 位置閉迴路。
- velocity feedforward。
- acceleration feedforward 關閉與開啟 A/B 測試。
- 小角度 Step。
- Joint sine。
- 不同速度與負載測試。

Gate：

- position RMSE 可重現量測。
- overshoot、rise time、settling time 有報告。
- 通訊 jitter 不直接造成 PWM frequency step。
- Integral 在 Stop／Fault／mode switch 後清除。
- Joint limit 與 homing 狀態正確阻擋命令。

### Phase 5：三軸 Joint PVA

工作：

- 三軸同步 setpoint。
- 各軸獨立 limit。
- 三軸小幅低速 trajectory。
- 一軸 fault 對全系統停止策略驗證。
- 多軸 UART bandwidth 測量。

Gate：

- 三軸 command／state axis mapping 無誤。
- 任一軸 fault 觸發預定停止策略。
- 30 分鐘低速運行無 control deadline miss。
- UART steady-state 平均使用率符合 gate。
- 三軸 tracking report 可自動產生。

### Phase 6：EF HIL

工作：

- 完成 FK／IK。
- 建立 `EndEffectorReference`。
- Joint limit 與 workspace validation。
- Cartesian straight line。
- 慢速小圓或 sine。
- 由 Encoder Joint state 執行 FK，重建 measured EF。

Gate：

- unreachable target 在 PC 端被拒絕。
- singularity 附近被拒絕或降速。
- reference EF 與 measured EF 可畫在同一時間軸。
- EF position error 可量測。
- MCU 完全不解析 Cartesian command。

### Phase 7：控制品質最佳化

工作：

- 馬達／驅動板動態辨識。
- Position gain tuning。
- velocity correction 評估。
- feedforward identification。
- deadzone／摩擦補償。
- 方向反轉補償。
- 重複定位。
- 不同負載測試。

Gate：

- 完成至少一份三軸 baseline report。
- 對每軸設定可接受的 RMSE、overshoot、settling time。
- 對 EF tracking 設定正式 tolerance。
- 所有正式 threshold 有量測依據，不以主觀觀察決定。

### Phase 8：隊友軟體整合

前置條件：

- 隊友定義 EF frame、unit 與 timing。
- 我方 Joint PVA interface 已穩定。
- 雙方確認 protocol／adapter ownership。

整合方式：

- 隊友輸出接入 `EndEffectorReference`，或
- 隊友直接輸出 ROS 標準 Joint trajectory。

原則：

- 只更換／新增 ROS 2 adapter。
- 不因隊友 JSON 格式修改 MCU command contract。
- 不讓 teammate path bypass IK、limit、Arm Session 或 Safety Gate。

---

## 11. 測試策略

### 11.1 Static／Build

- STM32CubeIDE clean build。
- C/C++ 0 warnings。
- ABI 保持 `-mfloat-abi=softfp -mfpu=fpv4-sp-d16`。
- `colcon build --symlink-install`。
- `colcon test`。
- Message generation 與 micro-ROS type support build。

### 11.2 單元測試

- Unit conversion。
- gear ratio。
- Encoder wrap。
- sequence wrap。
- TTL tick wrap。
- NaN／Inf。
- Joint limit。
- Motor RPM cap。
- Interpolator。
- acceleration limiter。
- jerk limiter。
- IK reachable／unreachable。
- singularity。

### 11.3 Software-in-the-Loop

- 假 MCU state publisher。
- Driver reconnect。
- command sequence loss。
- command reordering。
- malformed state。
- delayed state。
- duplicate state。
- Agent unavailable。
- protocol mismatch。

### 11.4 HIL fault matrix

| 測試 | 預期結果 |
|---|---|
| 停止 Agent process | MCU 清除 command 並安全停止 |
| 拔除 UART／USB | command timeout，舊命令不得復活 |
| 發送 NaN／Inf | 整包拒絕，fault/reject counter 增加 |
| 重複 sequence | 拒絕，不更新 command age |
| 舊 session | 拒絕，不驅動馬達 |
| 超出 RPM | 拒絕或依固定策略處理 |
| 超出 Joint limit | 整包拒絕 |
| 未 homed Arm | 拒絕 |
| E-Stop | 不依賴 ROS，立即進入安全狀態 |
| Limit switch | 禁止繼續向限制方向運動 |
| ControlTask deadline miss | counter／fault 符合規格 |
| Agent reconnect | 保持 Disarmed |

---

## 12. 控制品質量測

### 12.1 必須記錄的 signal

- host reference time。
- host Joint PVA。
- MCU accepted sequence。
- MCU applied PVA。
- measured Joint position／velocity。
- position error。
- motor RPM command。
- PWM frequency。
- safety state。
- fault code。
- command age。
- communication reconnect。

### 12.2 必須計算的指標

- Position RMSE。
- Position maximum absolute error。
- Steady-state error。
- Velocity RMSE。
- Overshoot。
- Rise time。
- Settling time。
- Command-to-accept latency。
- Command-to-motion latency。
- Telemetry jitter。
- Command jitter。
- 反轉死區。
- 重複定位誤差。
- Agent loss-to-stop time。

### 12.3 「精準」的定義流程

第一輪不先猜測 tolerance：

1. 建立可重現的測試腳本。
2. 取得單軸無負載 baseline。
3. 取得單軸負載 baseline。
4. 取得三軸低速 baseline。
5. 依機構、書寫品質與硬體能力設定 acceptance threshold。
6. 將 threshold 寫入測試設定與文件。
7. 後續修改以 regression test 驗證不得退化。

---

## 13. 風險與對策

| 風險 | 嚴重度 | 對策 |
|---|---:|---|
| motor RPM 與 Joint RPM 語意混用 | Critical | Phase 0 先完成單位契約與 unit test |
| 100 Hz minimum frequency 造成低速跳變 | High | 查驅動板規格並做低速 frequency sweep |
| 13-pin drive 內部動態未知 | High | Phase 3 系統辨識，量測 command-to-speed |
| Encoder velocity 10 ms 更新延遲 | High | 評估 estimator rate、filter 與 latency |
| 對 noisy velocity 計算 jerk | High | jerk 控制 reference，不直接閉迴路 noisy jerk |
| 115200 UART 頻寬不足 | Medium | 固定訊息、分頻、bandwidth gate、升 baud/USB |
| ROS jitter 造成 setpoint step | High | MCU 1 kHz interpolation + local rate guard |
| 重連後舊命令復活 | Critical | session、sequence、TTL、command invalidation |
| 多個 command owner | Critical | Driver arbiter + Firmware enum mode |
| Feedforward 未辨識即啟用 | High | 預設關閉，A/B 測試後才核准 |
| 未知 EF frame／unit | High | Motion Adapter 隔離，不進 MCU contract |
| 動態 message allocation | High | MCU fixed arrays、reconnect heap soak |

---

## 14. 尚待確認的硬體資料

下列資料不阻擋 Phase 0～2，但在馬達通電前必須完成：

- [x] 三軸實際 motor part number（Nidec `24H702U030`；Drawing `3GSPC22Z013`）。
- [x] 驅動板 frequency input specification（整合於 13-pin 馬達介面：Pin 1 `STMP`，上升緣偵測，內部 10 kΩ pulldown）。
- [x] 最低／最高有效 frequency（目前軟體命令範圍：`100 Hz` 至 `33,333 Hz`，對應 `15 RPM` 至 `5000 RPM`；實際有效下限仍以後續測試確認）。
- [x] frequency-to-speed 比例（規格書：`STMP[Hz] = Speed[min-1] × 400 / 60`，即每 1 RPM 約 `6.6667 Hz`）。
- [x] Encoder 實際 PPR（原始韌體：三軸皆 `100`，使用者已確認）。
- [x] gear ratio（原始韌體：三軸皆 `114.0`，使用者已確認）。
- [x] Joint 正方向（原始韌體 motor inversion：J1/J2/J3=`true/false/false`）。
- [x] Encoder 正方向（三軸 `encoder_inverted=false`）。
- [x] 機械零點（原始韌體 home：J1=`115°`、J2=`180°`、J3=`0°`）。
- [x] `STMP`／`CW/CCW` 的 STM32 3.3 V 輸出與馬達 5 V 邏輯輸入相容性（使用者實測通過）。
- [x] Encoder 5 V 輸出與 STM32 TIM encoder 腳位相容性（使用者實測通過）。
- [x] Joint soft limit（J1=`80–180°`、J2=`0–180°`、J3=`-180–180°`，使用者確認）。
- [x] Home switch 方向與有效電位（使用者確認目前接線與韌體設定）。
- [x] E-Stop 硬體路徑（使用者確認現行硬體限制：只能以速度命令歸零／`0 Hz` 停止；不宣稱具備獨立硬體 E-Stop）。
- [x] Driver fault output（使用者確認馬達沒有外部 fault 接頭；採內建保護與其他安全機制）。
- [x] 可允許的 motor RPM（馬達軸上限 `5000 RPM`；第一次 bring-up 仍限制 `150 RPM`）。
- [x] 可允許的 Joint velocity／acceleration／jerk（初始保守值：`10°/s`、`20°/s²`；jerk 先由現有 rate／zero-crossing guard 約束，實機後再調整）。
- [x] 供電、負載與機構測試條件（由使用者自行負責現場管理，不屬 Codex 軟體工作範圍）。

### 14.1 已由 Nidec 規格書取得的電氣資料

- `Pin 1 STMP`：頻率命令輸入；上升緣偵測，`VIH >= Vdd×0.8`、`VIL <= 0.9V`，內部 10 kΩ pulldown。
- `Pin 2 START`：`H` 接受 STMP 與方向，`L` 不接受；`VIH >= Vdd×0.7`、`VIL <= 0.9V`，內部 10 kΩ pulldown。
- `Pin 3 5.0V`：`DC5.0V±10%` 邏輯電源。
- `Pin 4 CW/CCW`：由輸出軸側觀看，`H=CCW`，開路或 `L=CW`；內部 10 kΩ pulldown。
- `Pin 7 ENC_B`、`Pin 8 ENC_A`：100 pulse/revolution，A/B 相位差 `90±45°`，duty `50±20%`，內部 2.2 kΩ pull-up。
- `Pin 11 SGND`、`Pin 12 PGND`、`Pin 13 VM`；`VM=DC24V±10%`。
- 規格書列出的無負載速度為 `6000 min⁻¹±1%`（DC24V、無負載），但這不是本機構的允許運轉上限；韌體目前保守限制仍應維持。
- 內建保護包含過電流、過／欠電壓、過溫、過載、位置偏差與振動保護；這些不能取代外部 E-Stop 或 MCU safety gate。

### 14.2 規格書仍未提供的系統資料

規格書沒有定義帶負載的 frequency-to-speed 誤差、外部 E-Stop 或本機構可承受的速度／加速度／jerk；目前以使用者確認的韌體保守值作為初始基線，實機測試後再調整。

另外，規格書揭露了三項必須在接線圖與示波器上確認的整合風險：

- 使用者已實測確認 STM32 3.3 V 輸出可正常驅動 `STMP`／`CW/CCW` 馬達 5 V 邏輯輸入；此項不再列為未解決風險。
- 使用者已實測確認 Encoder 5 V `ENC_A`／`ENC_B` 輸出可安全接入 STM32 TIM encoder 腳位；此項不再列為未解決風險。
- 使用者已確認 Pin 2 `START` 已停用 MCU 控制並由硬體固定為 5 V；目前 `motor_config.c` 的 `enable_port = NULL` 是刻意設定。韌體只控制 `STMP` 速度頻率與 `CW/CCW` 方向。
- 因為 `START` 固定為高，目前硬體限制是不提供獨立 E-Stop；使用者已確認由 `0 Hz` 軟體停止與現場操作程序處理。此限制不應被描述成硬體級緊急斷電能力。

---

## 15. Definition of Done

本計畫完成的定義不是單純「Topic 可以 echo」，而是同時滿足：

1. ROS 2 能產生可重現的 Joint 與 EF 測試 reference。
2. Driver 能以 versioned contract 傳送 command。
3. STM32 能以 1 kHz 安全執行並回報 applied state。
4. 三軸 position／velocity feedback 單位正確。
5. velocity／acceleration／jerk reference 不造成未受控 PWM step。
6. Agent 斷線、TTL 到期與 invalid command 均能安全停止。
7. 重連後舊命令不會復活。
8. 單軸與三軸 tracking test 可自動執行。
9. rosbag／CSV 與控制品質報告可重現。
10. EF reference 與 measured EF error 可量測。
11. 未來隊友輸入只需新增 adapter，不必修改 MCU command contract。
12. 所有實機測試具有可追溯 Gate 與量測證據。

---

## 16. 立即執行順序

### Sprint 1：不通電基線

1. 建立單位與 gear ratio 契約。
2. 修正 motor RPM／Joint RPM API 語意。
3. 釐清 100 Hz minimum frequency。
4. 建立 `Software/ros2_ws`。
5. 建立 `doghead_interfaces`。
6. 擴充 `McuState`／diagnostics 規格。
7. 完成主機端單位轉換測試。

### Sprint 2：Telemetry-only

1. Firmware fixed-size state publisher。
2. `doghead_driver` telemetry subscriber。
3. `/joint_states` conversion。
4. rosbag2／CSV recorder。
5. rate、jitter、heap、reconnect soak test。

### Sprint 3：Command shadow

1. command subscription。
2. session／sequence／TTL。
3. mailbox overwrite。
4. whole-packet validation。
5. accepted／rejected telemetry。
6. output disabled fault injection。

### Sprint 4：Conditional GO

1. 單軸 RPM ramp。
2. S-curve motor command。
3. minimum frequency test。
4. command-to-speed identification。
5. timeout／disconnect／E-Stop。
6. 建立第一份 motor baseline report。

完成 Sprint 4 且通過審查後，才進入 Joint PVA 與 EF HIL。

---

## 17. T05 micro-ROS 實體層與 Humble 容器工作流程規範 (v1.0 Compliance)

### 17.1 Canonical Topic Contract
全系統統一使用標準 Topic 名稱，禁止使用舊版過時路徑：

- `/doghead/mcu/joint_setpoint` (Subscription on MCU, Publisher on ROS 2 Driver)
- `/doghead/mcu/motor_test_command` (Subscription on MCU, Publisher on ROS 2 Driver)
- `/doghead/mcu/state` (Publisher on MCU at 50 Hz, Subscription on ROS 2 Driver & HIL Validator)
- `/doghead/mcu/diagnostics` (Publisher on MCU at 1 Hz, Subscription on ROS 2 Driver & HIL Validator)
- `/joint_states` (Publisher on MCU & ROS 2 Driver at 50 Hz)

### 17.2 Telemetry & Spin Cadence Rules
- **Setpoint Reception**: MCU `commTask` 執行 `MicroROS_Spin()`，spin timeout 設定為 5 ms，以符合 100 Hz setpoint 接收需求。
- **McuState / JointStates Telemetry**: 由 `commTask` 定時迴圈以 50 Hz (20 ms 週期) 讀取 `RobotInterface_GetTelemetry()` 並發布。
- **McuDiagnostics Telemetry**: 由 `commTask` 定時迴圈以 1 Hz (1000 ms 週期) 讀取 `RobotInterface_GetDiagnostics()` 並發布。
- **ControlTask 隔離**: 1 kHz `ControlTask` 僅透過 `RobotInterface_SetTelemetry()` 寫入狀態快照，不直接呼叫 `rcl_publish`，確保通訊層與控制迴圈解開執行緒競爭。

### 17.3 Disconnect, Cleanup & Disarm Recovery Rules
- **Entity Creation Order**:
  1. `rclc_support_init`
  2. `rclc_node_init_default`
  3. `pub_heartbeat` (`/doghead/heartbeat`)
  4. `pub_joint_states` (`/joint_states`)
  5. `pub_mcu_state` (`/doghead/mcu/state`)
  6. `pub_mcu_diagnostics` (`/doghead/mcu/diagnostics`)
  7. `sub_joint_setpoint` (`/doghead/mcu/joint_setpoint`)
  8. `sub_motor_test` (`/doghead/mcu/motor_test_command`)
  9. `rclc_executor_init` & subscription registration
- **Entity Destruction Order (Reverse)**:
  `RobotInterface_InvalidateCommand()` → `set_entity_destroy_session_timeout(0)` → `executor` → `sub_motor_test` → `sub_joint_setpoint` → `pub_mcu_diagnostics` → `pub_mcu_state` → `pub_joint_states` → `pub_heartbeat` → `node` → `support`.
- **Reconnect State Baseline**:
  - Agent 斷線時，MCU 呼叫 `RobotInterface_InvalidateCommand()` 清空舊指令與佇列。
  - Agent 重新連線後，ROS 2 Driver 依據 `agent_connected` 斷開、`boot_id` 變更或 `agent_reconnect_total` 計數器增加，自動進入 `DISARMED` 狀態。
  - MCU 必須等待 ROS 2 上位機發起新 Session (呼叫 `/doghead/driver/arm`) 才能重新使能。

### 17.4 ROS 2 Humble Docker 容器工作流程
專案 `docker-compose.yml` 將 `./` (專案根目錄) 安全掛載至 `/workspace`，預設工作目錄設為 `/workspace/Software/ros2_ws`。

容器內標準建置與測試指令：

日常開發建議使用 `scripts/run_ros2_tests.ps1`。它會建立一次性測試容器，所有測試成功後自動刪除；失敗時則保留容器供除錯。`ros2-dev` 開發容器仍可用 `docker compose up -d ros2-dev` 長時間保留。
```bash
# 1. 進入容器與環境 setup
cd /workspace/Software/ros2_ws
source /opt/ros/humble/setup.bash

# 2. 建置全套件
colcon build --packages-select doghead_interfaces doghead_driver doghead_motion doghead_hil doghead_bringup

# 3. 執行容器測試
colcon test --packages-select doghead_driver doghead_hil doghead_bringup
colcon test-result --all --verbose
```

## Second-round software review evidence note

The second-round safety, shadow-execution, trajectory, telemetry, reconnect,
runtime-metric, QoS, and HIL-validator fixes are represented in source and
test updates. This is source-level evidence only: `ACTUATOR_HARDWARE_OUTPUT_ENABLED`
remains `0`, no powered motor test was performed, and Phases 3--7 remain
pending until their independent ROS 2 Humble/container, HIL, safety, and
hardware evidence requirements are met.

### 2026-08-05 second-round repair verification update

The source-level remediation now also includes the end-to-end RPM contract:
Motor Test `target_motor_rpm` is documented as joint/output-shaft RPM for wire
compatibility, `Motor_SetSpeed()` consumes that same unit, and the reducer is
applied exactly once in the firmware driver. Host regressions cover the former
legacy +/-100 cap and a 114:1 Motor Test/PVA conversion.

Observed local verification:

- driver pure-Python tests: **18 passed, 5 deselected**;
- motion targeted tests: **3 passed, 1 skipped** (ROS-dependent test);
- HIL targeted tests: **17 passed, 2 skipped** (ROS-dependent tests);
- Python compilation checks: **passed**;
- both `McuState.msg` and `McuMotorTestCommand.msg` source copies: **synchronized**.

Not verified in this environment: Firmware host execution (native `g++`/`gcc`
are unavailable), ROS-dependent tests requiring `rclpy`/`numpy`, ROS 2 Humble
`colcon build`/`colcon test` (`colcon`/`ros2` and a usable Docker Humble daemon
are unavailable), and all powered or physical motor evidence. The default
`ACTUATOR_HARDWARE_OUTPUT_ENABLED` remains `0`; Phases 3--7 remain **pending**.
