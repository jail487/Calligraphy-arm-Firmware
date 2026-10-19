# 書法機器臂軟體／韌體系統架構與後續實作計畫

> [!IMPORTANT]
> 本文件是架構計畫；其中 115200 與 Phase 4A–4D 的描述已被實作演進取代。
> 現況是 460800 baud、僅遙測與命令影子 HIL 通過，馬達帶電未授權。

**文件狀態**：Architecture Decision / Implementation Plan  
**適用平台**：STM32F446RET6 + FreeRTOS + micro-ROS + ROS 2 Humble  
**日期**：2026-07-30  
**前置基線**：Phase 3 micro-ROS Agent、USART2 DMA Transport 與連線狀態機已完成

---

## 1. 架構結論

本專案應採用「上位機全域規劃、MCU 確定性執行、MCU 獨立安全」的雙層控制架構：

```text
ROS 2 Humble / Linux
  書法筆畫規劃、紙面座標、MoveIt、IK、碰撞檢查、軌跡時序
                         │
                         │ 固定大小、具 sequence / TTL / arm token 的裝置協定
                         ▼
micro-ROS Agent ─── USART2 DMA / 未來 USB CDC ─── STM32F446RET6
                                                   1 kHz 回授與 PID
                                                   短時域 setpoint 插值
                                                   Homing / Limit / Watchdog
                                                   Safety Gate / Motor EN
```

主要原則如下：

1. ROS 2 或 WSL2 不是安全控制器；Agent、USB、DDS 或 Linux 故障時，MCU 必須自行安全停止。
2. 1 kHz encoder、PID、PWM 與最終 Motor EN 決策必須留在 STM32。
3. 字型／SVG 解析、筆順、紙面標定、全域 IK、碰撞檢查及長軌跡規劃應移到 ROS 2。
4. 不讓 ROS 2 經由 115200 UART 直接閉迴路控制 PWM 或馬達 RPM。
5. 通訊邊界統一使用 SI 單位、固定大小訊息及明確的 mode，不使用無語意的浮點陣列。
6. STM32 只接受最新、有效、屬於目前 Arm Session 的指令；重連後永不自動恢復運動。

---

## 2. 現有專案架構評估

### 2.1 已具備且應保留

- TIM6 通知驅動的 1 kHz `ControlTask`，優先權高於 `CommTask`。
- `CommTask` 與控制迴圈分離，micro-ROS 阻塞不直接破壞 1 ms 控制週期。
- USART2 Circular RX DMA 與有界 TX timeout。
- `RobotInterface` 長度 1 overwrite Queue，以及以 FreeRTOS tick 計算的 250 ms stale timeout。
- `SafetyManager`、Homing、限位開關、輸出有限性檢查及未測試軸實體停用。
- 斷線後 micro-ROS Entity 反向清理及重新建立。

### 2.2 目前不適合作為正式產品架構的部分

1. `mainpp.cpp` 同時負責：
   - 回授讀取；
   - IK/FK；
   - 軌跡；
   - PID；
   - Safety；
   - 測試模式；
   - 遙測組裝。

   這使 WCET、測試、模式切換及安全責任難以獨立驗證，後續應拆分。

2. `RobotCommand_t` 同時包含測試 RPM、Joint Position 與 Cartesian Pose。雖然欄位未直接重用，但仍是多種命令共用一個非 tagged 結構，容易出現未初始化欄位或錯誤 mode 組合。

3. Firmware 內同時存在全域 IK、軌跡規劃與 `Robot_SquareTest_Tick()`。正式運轉時會與 ROS 2 Planner 形成兩個命令來源。

4. Firmware 內部使用 deg、RPM、mm；ROS 2 標準介面使用 rad、rad/s、m。若沒有唯一轉換層，很容易產生單位錯誤。

5. 目前遙測狀態把 E-Stop、Fault、通訊遺失壓縮成較少的狀態，無法支援上位機精確診斷。

6. (已修正) Agent 狀態機已實作 `RobotInterface_InvalidateCommand()` 立刻銷毀無效指令。

7. USART2 現行 HIL 基線為 460800 8N1。實測 20.15 秒 wire rate 為
   17.20 kB/s（37.33%），50 Hz telemetry 使用 Best-Effort；115200 已證實不足。

8. Repository 已有 ROS 2 Humble 工作區、五個套件與自訂訊息；URDF、
   MoveIt config 與 production `ros2_control` hardware plugin 仍未完成。

---

## 3. 上下位機責任邊界

| 功能 | STM32 Firmware | ROS 2 Humble 上位機 | 理由 |
|---|---:|---:|---|
| Encoder capture、角度更新 | **主責** | 顯示／記錄 | 需要固定時序與硬體直接存取 |
| PWM、方向、Motor EN | **唯一主責** | 禁止直接操作 | 最終致動必須受 MCU Safety Gate 控制 |
| 1 kHz Joint PID | **主責** | 只提供 setpoint / tuning proposal | Linux、DDS、USB 不具硬即時性 |
| 短時域 setpoint 插值、slew limit | **主責** | 提供時間化 setpoint | 吸收 20–100 Hz 通訊 jitter |
| Homing、硬限位、encoder zero | **主責** | 發出 request、監看結果 | 即使 Agent 消失也必須完成安全處置 |
| E-Stop、Watchdog、Fault latch | **主責** | 監看／要求 reset | 安全不得依賴 ROS |
| Joint 軟限位 | **強制再檢查** | 規劃時預先檢查 | Defense in depth |
| 全域 IK／FK 模型 | 最小安全檢查或診斷備援 | **主責** | 方便與 URDF、MoveIt、標定資料一致 |
| 長軌跡、速度／加速度／jerk 規劃 | 局部執行與界線檢查 | **主責** | 上位機較適合最佳化與大量路徑資料 |
| SVG／字型／筆順／筆畫平滑 | 不放入 MCU | **主責** | 非即時、資料量大、演算法易變 |
| 紙面座標標定、TF | 僅接收已轉換結果 | **主責** | 應與 ROS TF、視覺及 URDF 共用 |
| 碰撞檢查、工作空間規劃 | 最終 joint envelope | **主責** | MoveIt 與完整機構模型適合此工作 |
| `/joint_states` 標準發布 | 可作啟動期暫時介面 | **建議由橋接程式主責** | 避免 MCU 動態 string/sequence 與頻寬浪費 |
| rosbag、圖表、診斷、UI | 不放入 MCU | **主責** | 資源與可維護性 |
| 參數管理 | 驗證、套用、必要時保存安全值 | 版本管理與下發 | MCU 不接受運轉中任意修改安全參數 |

### 3.1 建議從 Firmware 移到 ROS 2 的模組

- `kinematics.cpp` 中的正式 Cartesian IK 與全域 FK 模型：
  - 移到 `doghead_kinematics` 或 MoveIt kinematics plugin。
  - 韌體可暫時保留程式供啟動期比對，但正式命令路徑不接受任意 Cartesian Pose。
- `trajectory_planner.cpp` 的長距離軌跡規劃：
  - 移到 ros2_control `joint_trajectory_controller` 或專案 Planner。
  - MCU 只保留有界的相鄰 setpoint 插值、速度限制及最後一道軟限位。
- `Robot_SquareTest_Tick()` 與一般測試情境：
  - 改成 ROS 2 test node 或 CLI tool。
  - Firmware 僅保留 compile-time 啟用、必須實體解鎖的 Factory Bring-up 模式。
- 標準 `/joint_states` 組裝、joint name 與 ROS timestamp：
  - 正式架構中由上位機 Bridge 將固定大小 MCU state 轉成 `sensor_msgs/msg/JointState`。
- printf 診斷、人類可讀字串與歷史統計：
  - 移到上位機；MCU 只傳 enum、counter 與固定欄位。

### 3.2 必須留在 Firmware 的模組

- `nidec_motor_driver`、`hardware_io`、`motor_config`。
- 1 kHz feedback、PID 及致動輸出 stage。
- Safety Manager、Homing、E-Stop、limit switch、watchdog。
- Command freshness、sequence、Arm Session 驗證。
- 硬編碼的啟動期 RPM 上限與不可由 ROS 動態放寬的絕對 joint envelope。
- Agent 失聯時的 command invalidation 與安全停止。
- 固定大小 telemetry snapshot、transport counter、deadline miss counter。

---

## 4. 建議的 ROS 2 Humble 軟體架構

新增獨立 ROS 2 workspace，建議套件如下：

```text
ros2_ws/src/
├── doghead_msgs/             # MCU 固定大小 command/state/status 介面
├── doghead_description/      # URDF/Xacro、joint limit、mesh、TF
├── doghead_driver/           # MCU bridge、session、診斷、單位轉換
├── doghead_hardware/         # ros2_control SystemInterface plugin
├── doghead_control/          # controller YAML、joint_trajectory_controller
├── doghead_moveit_config/    # SRDF、planning group、碰撞矩陣
├── doghead_kinematics/       # 平行機構 IK/FK 與測試
├── doghead_calligraphy/      # SVG/字型、筆順、紙面路徑、速度/壓力規劃
├── doghead_bringup/          # launch、Lifecycle orchestration
└── doghead_diagnostics/      # fault decoder、health、rosbag、測試工具
```

### 4.1 `doghead_driver`

建議設為 ROS 2 Lifecycle Node：

- `Unconfigured`：尚未載入模型與 serial 設定。
- `Inactive`：Agent/MCU 可連線與讀取狀態，但禁止送運動命令。
- `Active`：完成 protocol、firmware version、homing 與安全前置條件檢查後，才允許命令。
- `ErrorProcessing`：停止發布命令、撤銷 Arm Session、保留診斷資訊。

Driver 的責任：

- 將 MCU 固定大小 state 轉成 `/joint_states`。
- 將 ros2_control setpoint 轉成 MCU command。
- 管理唯一 command owner、sequence、Arm Session 及 reconnect。
- 將 fault enum 轉成 `diagnostic_msgs`。
- 不替 MCU 做最終安全判定。

### 4.2 ros2_control 整合

正式整合建議使用 `hardware_interface::SystemInterface`：

- State interfaces：`position`、`velocity`。
- Command interface：第一版只開放 `position`。
- `read()`：讀取 Bridge 最新 MCU snapshot，不阻塞 serial。
- `write()`：只覆寫最新 command mailbox，不等待 ACK。
- Controller Manager update rate：初期 50–100 Hz，不以 UART 執行 1 kHz closed loop。
- 使用 `joint_trajectory_controller` 的 `FollowJointTrajectory` action 取得執行回饋、取消、容差與結果。
- MCU 仍以 1 kHz 插值並執行 joint position servo。

禁止同時存在下列多個 command owner：

- ros2_control；
- 直接 `ros2 topic pub`；
- Firmware square test；
- GUI joystick；
- MoveIt。

所有來源必須先通過上位機 command arbiter；任一時間只核發一個 Arm Session。

### 4.3 書法應用層

`doghead_calligraphy` 建議拆成以下 pipeline：

```text
文字／SVG
  → 筆畫拓撲與筆順
  → 紙面 2D 路徑
  → corner smoothing / resampling
  → 筆尖高度或壓力 profile
  → paper frame → robot base TF
  → IK / workspace / collision validation
  → time parameterization
  → FollowJointTrajectory
```

書法專案特別需要：

- 落筆、行筆、收筆與換筆畫的不同速度 profile。
- 紙面平面標定與筆尖 TCP calibration。
- 轉角減速、曲率限制與 jerk 限制，避免墨跡在角點堆積。
- 筆尖接觸策略；若未來要控制筆壓，應新增 force sensor 或獨立 compliant axis，不應以馬達電流猜測取代安全回授。
- 執行前完整 preview，正式運動時不在 MCU 即時計算字型或筆順。

---

## 5. MCU 內部目標架構

```text
TIM6 ISR
  └─ notify ControlTask (1 kHz)
       ├─ Feedback Acquisition
       ├─ Safety State Update
       ├─ Command Mailbox / TTL / Session Validation
       ├─ Setpoint Interpolator + Limiters
       ├─ Joint PID
       ├─ Output Validation
       └─ Actuator Output Stage  ← 唯一可呼叫 Motor_Start/SetSpeed

CommTask (50–100 Hz)
  ├─ micro-ROS Connection State Machine
  ├─ Executor callbacks
  ├─ Command decode/validate → overwrite mailbox
  └─ Fixed-size state/status publish
```

建議將目前 `mainpp.cpp` 漸進拆成：

```text
Application/
├── control_loop.cpp          # 1 kHz orchestration
├── feedback_estimator.cpp    # encoder → joint state
├── joint_servo.cpp           # PID
├── setpoint_interpolator.cpp # local interpolation / slew limit
├── actuator_output.cpp       # 唯一 Motor_* owner
├── safety_manager.cpp
├── homing_calibration.cpp
├── robot_interface.cpp       # command mailbox / state snapshot
└── factory_test.cpp          # compile-time gated
```

### 5.1 命令模式

不要再以 `test_mode` + `use_ik_mode` 的 bool 組合決定命令語意。改用單一 enum：

```c
typedef enum {
    ROBOT_CMD_NONE = 0,
    ROBOT_CMD_FACTORY_SINGLE_AXIS_RPM,
    ROBOT_CMD_JOINT_POSITION,
    ROBOT_CMD_HOLD,
    ROBOT_CMD_STOP
} RobotCommandMode_t;
```

每個 mode 使用專屬 payload 或 tagged union；production build 不啟用 Factory RPM mode。

### 5.2 Safety 狀態

建議狀態至少區分：

```text
BOOT → SELF_TEST → DISARMED → HOMING → READY
                                      │
                                      ▼
                                  ARMED_IDLE → EXECUTING
                                      │           │
                                      └── HOLD ◄──┘

任意運動狀態 → COMM_LOST / FAULT / ESTOP
```

必要規則：

- MCU 開機與 Agent reconnect 後皆為 `DISARMED` 或 `READY`，絕不直接回到 `ARMED`。
- Arm 必須滿足：已 homed、無 fault、E-Stop released、命令來源唯一、session 建立成功。
- Clear Fault 不等於 Arm。
- `COMM_LOST` 清空 command、interpolator target 與所有有效旗標。
- 通訊恢復只允許重新建 Entity 與上報狀態，必須重新 Arm。

### 5.3 停止策略

「安全停止」不可一律等同 EN low；若機構受重力影響，立即解除扭力可能造成下墜。實機風險評估後定義：

- Normal Stop：受限減速至零，進入 Hold 或 Disarmed。
- Command timeout／Comm lost：在有界時間內減速或 Hold，隨後依機構風險決定保扭或 EN low。
- Invalid command／軟限位／deadline miss：Latch Fault，禁止後續命令。
- Hard limit／E-Stop：硬體層立即切斷或煞車，不依賴 ROS、micro-ROS callback 或 Task 排程。

---

## 6. 通訊架構

### 6.1 不建議讓 MCU 直接承擔正式 `/joint_states`

Phase 4 可用 `/joint_states` 驗證三軸資料與 micro-ROS message memory，但正式架構建議：

```text
MCU: /doghead/mcu/joint_state_raw  (固定大小 custom message)
                          ↓
ROS 2 doghead_driver
                          ↓
PC:  /joint_states                (sensor_msgs/msg/JointState)
```

優點：

- MCU 不需要反覆 serialize joint name。
- 避免 `JointState` 的 string/sequence 動態配置。
- 可同時傳送 safety state、fault 與 counter，而不污染標準訊息。
- ROS timestamp、joint name、SI 轉換集中在 PC。
- 降低 115200 UART 使用率。

### 6.2 建議自訂訊息

以下為概念介面；實作前應在 `doghead_msgs` 固化 IDL 並重建 micro-ROS 靜態庫。

`McuJointCommand.msg`：

```text
uint16 protocol_version
uint8 mode
uint32 arm_token
uint32 sequence
uint16 valid_for_ms
float32[3] position_rad
float32[3] velocity_rad_s
```

MCU 驗證規則：

- `protocol_version` 必須完全匹配。
- `arm_token` 必須屬於當前 Arm Session。
- `sequence` 必須較上一筆新；考慮 uint32 wrap。
- `valid_for_ms` 必須在允許範圍內，並以 MCU 收件時的 FreeRTOS tick 計時。
- 所有 float 必須 finite。
- position、velocity 皆必須通過每軸限制。
- 任一欄錯誤即整包拒絕，不允許 partial apply。

`McuJointState.msg`（50 Hz）：

```text
uint16 protocol_version
uint32 boot_id
uint32 sequence
uint32 uptime_ms
float32[3] position_rad
float32[3] velocity_rad_s
float32[3] commanded_position_rad
uint8 safety_state
uint16 fault_code
bool[3] homed
uint16 command_age_ms
```

`McuDiagnostics.msg`（1–10 Hz）：

```text
uint32 control_deadline_miss_total
uint32 uart_ore_total
uint32 uart_ne_total
uint32 uart_fe_total
uint32 dma_rx_overflow_total
uint32 transport_tx_timeout_total
uint32 agent_reconnect_total
uint32 free_heap_bytes
uint32 minimum_ever_free_heap_bytes
uint32 control_wcet_cycles
```

高頻 state 與低頻 diagnostics 分開，避免每個 20 ms 封包重複傳送低變化欄位。

### 6.3 Management 介面

上位機可提供標準 action/service，但 MCU 端保持小型、明確：

- `request_homing`
- `request_arm`
- `request_disarm`
- `request_clear_fault`

每個 request 必須帶 request ID，回覆實際結果與拒絕原因。不得把 `clear_fault`、`home` 或 `arm` 隱含在速度／位置封包中。

不建議在 MCU 實作 `FollowJointTrajectory` action server；Action、取消、容差與 MoveIt 整合留在上位機，MCU 只執行短且固定大小的 setpoint contract。

### 6.4 QoS

| 介面 | 建議 QoS | 說明 |
|---|---|---|
| Joint state raw | Best Effort, Keep Last 1, Volatile | 最新回授比補發舊回授重要 |
| Joint command | Reliable, Keep Last | 低頻控制交易；MCU 仍以本地 TTL、session、sequence 作安全依據 |
| Session control / Diagnostics | Reliable, Keep Last | Arm epoch 與低頻 counter 必須可靠傳遞 |
| Arm/Homing/Fault management | Reliable, Volatile | 低頻交易，搭配 request ID 防重複副作用 |

DDS Deadline、Lifespan 或 Liveliness 可作上位機診斷，但不得取代 MCU 的 local RTOS TTL。

### 6.5 Transport 演進

1. **目前 HIL**：USART2 460800 8N1，Windows COM bridge，RXNE IRQ ring + DMA TX。
2. **已量測**：17.20 kB/s（37.33%），約 49.2–49.3 Hz，無 transport error/reconnect。
3. **中期**：原生 USB CDC，將 ROS binary transport 與人類可讀 debug UART 分離。
4. **量產／高雜訊環境**：
   - ROS supervisory link 可使用 USB／高速 UART；
   - 分散式馬達或安全 I/O 可另用 CAN；
   - 不因換 transport 而改變 command/session/safety contract。

任何 UART transport 都應以量測值限制：

- 平均線路使用率目標低於 50%；
- 壓力測試峰值低於 70%；
- 不允許 RX ring overflow；
- binary micro-ROS UART 禁止混入 `printf`。

---

## 7. 根據 Phase 3 Code Review 延伸的必做項目

### 7.1 Command invalidation

新增：

```c
void RobotInterface_InvalidateCommand(CommandInvalidationReason_t reason);
```

必須清除：

- Queue 中尚未消費的 command；
- cached command；
- `has_command`；
- setpoint interpolator；
- Arm Session 或 command lease（視原因決定）。

呼叫時機：

- Agent ping 或 publish/spin 失敗；
- transport timeout／DMA abnormal；
- Entity destruction；
- explicit disarm；
- command validation failure；
- Safety fault。

### 7.2 micro-ROS Entity lifecycle

加入 executor、subscription 與多 publisher 後，初始化使用逐項 flag；銷毀採反向順序：

```text
executor
→ management service(s)
→ command subscription
→ diagnostics publisher
→ joint state publisher
→ node
→ support
```

任何一步失敗都繼續清理其餘已建立項目，並記錄 cleanup error counter。

### 7.3 靜態記憶體與 reconnect

- Message sequence/string 只可在 Entity create 階段一次配置，控制／通訊 steady state 禁止 malloc。
- Custom message 優先使用 fixed array。
- 每次 create/destroy 前後記錄 free heap 與 minimum-ever-free heap。
- 1,000 次 reconnect 後 free heap 不得呈單調下降。
- 監測 CommTask 與 ControlTask stack high-water mark。

### 7.4 控制輸出單一所有權

目前 Safety Manager 與 `mainpp.cpp` 都可直接呼叫 `Motor_*`。重構後只保留一個 `actuator_output`：

- 所有上層模組只產生 desired output 或 stop reason。
- `actuator_output` 執行 finite、hard cap、enable mask、Safety state 檢查。
- 對硬體 API 的呼叫集中，便於審查及 fault injection。
- E-Stop 硬體切斷仍為獨立路徑。

---

## 8. 分階段實作計畫

### Phase 4A：Protocol 與 ROS workspace，不通電

- 建立 `ros2_ws` 與 `doghead_msgs`。
- 固化 joint name、方向、SI 單位、protocol version、fault code。
- 建立 `doghead_driver` skeleton 與 Lifecycle。
- 建立 command/state serialization host test。
- 重建 micro-ROS static library。

**Gate**：

- Firmware Clean Build 0 error / 0 warning。
- 訊息大小、RAM/Flash delta 有紀錄。
- malformed、NaN/Inf、size/version mismatch 全部拒絕。

### Phase 4B：Telemetry only，不接受運動命令

- MCU 發布 fixed-size joint state raw。
- Driver 轉成 `/joint_states`。
- RViz 顯示三軸；確認方向、零位、rad 與 rad/s。
- 加入 diagnostics counter 與頻寬量測。

**Gate**：

- `/joint_states` 正常連線時目標頻率 50 Hz，允許因 Agent ping timeout 或 DMA 阻塞產生合理 jitter。
- 1 小時 telemetry soak test 無 heap drift、無 RX overflow。
- Agent 反覆重啟不影響 1 kHz task。

### Phase 4C：Safety management

- 實作 command invalidation。
- 實作 Disarm/Homing/Arm/ClearFault 交易。
- 新增 arm token、sequence 與 reconnect disarm。
- 明確定義通訊 timeout 的停止策略。

**Gate**：

- Agent 拔除、process kill、USB disconnect、TX timeout 均能安全停止。
- Reconnect 後無新 Arm request 不得產生任何 Motor EN。
- Clear Fault 不會自動運動。

### Phase 4D：Command shadow mode

- Subscription callback 驗證 command 並寫入 mailbox。
- ControlTask 完整消費與計算，但 actuator output 強制 disabled。
- 比對 ROS setpoint、MCU accepted setpoint 與 reject reason。

**Gate**：

- 舊 sequence、錯 token、超 TTL、NaN、越界、亂 mode 全數拒絕。
- Command callback 不直接呼叫 Motor/HAL/Safety Run API。

### Phase 4E：單軸無負載 Bring-up

- Factory mode 僅允許單軸 10 RPM，其他軸 EN low。
- 逐軸提高至 50 RPM。
- 實測 E-Stop、limit、timeout 與 Agent disconnect。

**Gate**：

- 符合既有 Conditional GO 清單。
- 示波器／邏輯分析儀確認未選軸 EN low。
- 最壞停止時間有量測，不只觀察 ROS log。

### Phase 4F：三軸 Joint Position

- 僅接受 joint position + optional velocity feedforward。
- MCU local interpolator 與 slew/acceleration limit。
- 先低速點對點，再進行短 joint trajectory。

**Gate**：

- 所有 joint limit、方向與 homing 經實機確認。
- 無通訊時不動、重連不復動。
- 30 分鐘測試零 control deadline miss。

### Phase 5：ros2_control 與 MoveIt

- 實作 `doghead_hardware::SystemInterface`。
- 導入 `joint_state_broadcaster` 與 `joint_trajectory_controller`。
- 建立 URDF/Xacro、SRDF、MoveIt config。
- 上位機成為 production 唯一 trajectory owner。
- 將 Firmware Cartesian IK 與 square test 從 production path 移除。

### Phase 6：書法路徑

- 紙面 frame 與筆尖 TCP 標定。
- SVG／字型筆畫解析。
- 筆順、抬筆／落筆、曲率速度與 jerk 規劃。
- Dry-run（筆尖離紙）後才進行沾墨測試。
- 建立 path tracking、筆畫幾何誤差與重複性指標。

### Phase 7：Production hardening

- 1,000 次 Agent reconnect。
- 72 小時 soak。
- UART ORE/NE/FE、DMA overflow、封包洪水與電源擾動 fault injection。
- Watchdog、brownout、E-Stop、limit switch stuck 測試。
- Firmware/protocol compatibility matrix 與升級／回退程序。

---

## 9. 驗收指標

### Firmware real-time

- ControlTask 固定由 TIM6 notification 喚醒。
- 量測最大 WCET 小於 800 µs；正常目標小於 500 µs。
- 30 分鐘實機運轉零 deadline miss；任意連續 miss 仍觸發 Fault。
- steady state 不做動態記憶體配置。

### Communication

- Command freshness 完全使用 MCU FreeRTOS tick。
- 不以 PC timestamp 判斷安全有效期。
- command timeout 最差 250 ms；偵測到明確 transport failure 時應更快 invalid。
- 115200 時平均頻寬低於 50%、峰值低於 70%。
- 1,000 次 reconnect 後 heap 無趨勢性下降。

### Safety

- Boot、reconnect、clear fault 後皆保持 Disarmed。
- 未 homed 不可 Arm。
- 每次 Arm 產生新 token，舊 token 永遠失效。
- 未選測試軸實體停用。
- E-Stop 不依賴 ROS 2、Agent 或 FreeRTOS callback。

### Motion quality

- Joint state 與 command 全部使用 rad / rad/s。
- 每軸追蹤誤差與 overshoot 有可量測門檻。
- 書法階段另定義筆尖平面誤差、重複性、角點墨跡與落筆位置誤差。

---

## 10. 近期優先順序

依目前專案狀態，下一個安全且高價值的工作順序為：

1. 不實作 `/motor_cmd Float32MultiArray`。
2. 先定義 `doghead_msgs` 固定大小 command/state/status。
3. 建立 ROS 2 workspace、URDF skeleton 與 `doghead_driver`。
4. (部分完成) 實作 telemetry raw → `/joint_states`，不通電。
5. (部分完成) 實作 `RobotInterface_InvalidateCommand()`。仍需補齊 Arm Session 與詳細 Safety Status。
6. 完成 shadow command 後才進行單軸 10 RPM。
7. 三軸位置控制穩定後，再導入 ros2_control／MoveIt。
8. 最後才加入正式書法筆畫規劃。

---

## 11. 參考依據

- ROS 2 Humble QoS：<https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html>
- ROS 2 Managed Node 設計：<https://design.ros2.org/articles/node_lifecycle.html>
- ros2_control Humble Hardware Component：<https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html>
- ros2_control Joint Trajectory Controller：<https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>
- MoveIt Trajectory Execution：<https://moveit.picknik.ai/humble/api/html/trajectory_execution.html>
