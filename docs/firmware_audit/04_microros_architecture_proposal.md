# micro-ROS 架構提案

## 推薦

採用 **FreeRTOS + micro-ROS、USART2 custom serial transport、DMA RX circular / DMA TX normal**。保留TIM6驅動的ControlTask；micro-ROS只在較低優先級CommTask執行。第一版不讓executor主導1 kHz時序，也不讓callback直接操作PWM。

理由：專案已穩定使用FreeRTOS task notification、HAL timebase也已移到TIM7。改回bare-metal會重做排程與reconnect而沒有實際收益；真正風險是priority、memory與資料邊界，應在既有RTOS上修正。

## Bare-metal vs FreeRTOS

| 面向 | Bare-metal | FreeRTOS（推薦） |
|---|---|---|
| 導入成本 | 需重排現有task/control | 保留現況，只加CommTask |
| 控制即時性 | 可做到，但spin/reconnect易污染super-loop | ControlTask可高於CommTask且完全不等待ROS |
| 記憶體 | 少TCB/stack，較省 | 多約task stack/heap；F446 128K仍可規劃 |
| callback管理 | 手動state machine | task + snapshot/queue邊界清楚 |
| executor spin | 必須嵌入loop並限時 | CommTask以bounded spin_some |
| UART transport | polling/IRQ state machine | DMA + task等待更自然 |
| agent斷線 | loop中自行管理 | CommTask connection state machine |
| 未來擴充 | entity多後複雜 | 可加低頻diagnostics而不碰control |
| debug | 單thread較直觀 | 需正確priority/stack監測 |
| 本專案適配 | 會推翻已存在架構 | 最小改動 |

## Transport

### 首選：USART2 + DMA

實際腳位 PA2 TX / PA3 RX。標準 Nucleo-F446RE通常經ST-Link VCP呈現為USB serial，適合Ubuntu/Raspberry Pi用 `/dev/ttyACM*`；自製板或改solder bridge時要實測。

第一個通信baseline保留115200，因為一個25 Hz joint state與20 Hz command有足夠raw bandwidth；通過丟包與24 h測試後，可依序測460800、921600。不要在同一步同時換DMA、baud、message set，否則難定位。

硬體注意：

- MCU與Raspberry Pi UART都是3.3 V TTL才可直連，必須共地。
- 不可接RS-232電壓，也不可接5 V TTL。
- PC應使用ST-Link VCP或3.3 V USB-UART。
- 無hardware flow control時依賴DMA buffer、XRCE framing及速率控制。
- USART2 byte stream只給micro-ROS；log走SWO或另一UART。

DMA配置：RX circular、TX normal；DMA arbitration priority Very High；USART/DMA NVIC建議6，使TIM6 priority5仍更高，且若callback使用RTOS FromISR API也符合`>=5`規則。

### 備選

1. USART2 interrupt transport：Phase 3最少Cube設定，但每byte IRQ與舊transport缺陷使其不適合長期版本。
2. USB CDC：吞吐與接線方便，但會引入USB device middleware、占PA11/PA12；目前PA11/PA12正是home switches，**存在直接pin衝突，故不推薦**。
3. Ethernet：F446RE/Nucleo-64現有專案無PHY/LwIP配置，改動與RAM成本大。
4. CAN：適合底層設備網路，不是micro-ROS Agent首版transport；現況未配置。

## STM32 / host 職責

STM32負責：

- 1 kHz safety、encoder acquisition、motor output
- homing與limit switch
- command validation、age timeout、enable/estop state
- joint-level trajectory interpolation與PID
- 產生joint/fault/health snapshot

ROS 2 host負責：

- MoveIt/高階path planning、書法軌跡生成
- joint name/order mapping與trajectory streaming
- UI、recording、visualization
- 跨sensor融合與長期log
- diagnostics轉成使用者可讀訊息

目前IK/FK可先留STM32以保留既有功能，但ROS遠端介面優先傳 **joint target**，避免host與MCU同時做IK造成解分支不同。未來若host planning成熟，可把Cartesian command留作本機測試而非第一版ROS API。

Odometry、mecanum、`nav_msgs/Odometry`與`cmd_vel`不適用這個arm firmware。若之後是另一個底盤MCU，應另做node/firmware audit。

## 最小 topic 集合

### Phase 3：純通訊 smoke test

| Topic | 方向 | Type | 頻率 / QoS | timestamp / frame | 行為 |
|---|---|---|---|---|---|
| `/doghead/mcu/heartbeat` | pub | `std_msgs/msg/UInt32` | 1 Hz, best effort, depth 1 | 無 | uptime或sequence；必要 |
| `/doghead/mcu/ping` | sub | `std_msgs/msg/UInt32` | on change, best effort, depth 1 | 無 | 只記錄sequence，不接管馬達；必要 |

一個publisher、一個subscription、一個executor、無ROS timer（由CommTask低頻排程publish）即可。

### Phase 4：接入joint control

| Topic | 方向 | Type | 建議頻率 / QoS | timestamp / frame_id | 資料與timeout | 首版 |
|---|---|---|---|---|---|---|
| `/doghead/arm/joint_command` | sub | `trajectory_msgs/msg/JointTrajectoryPoint` | ≤20 Hz；best effort depth1 | message無header；本機記receipt tick | positions固定3、rad；velocities可空或3；250 ms未更新→COMM_LOST | 必要 |
| `/joint_states` | pub | `sensor_msgs/msg/JointState` | 25 Hz；best effort depth1 | agent同步ROS time；frame_id留空 | names固定3；position rad、velocity rad/s | 必要 |
| `/doghead/arm/enable` | sub | `std_msgs/msg/Bool` | event；reliable depth1 | 無 | 只允許READY↔RUNNING；false立即stop | 建議 |
| `/doghead/mcu/fault_code` | pub | `std_msgs/msg/UInt32` | 2 Hz + change；reliable depth1 | 無 | bitmask/enum版本化 | 建議 |
| `/doghead/mcu/heartbeat` | pub | `std_msgs/msg/UInt32` | 1 Hz | 無 | agent觀察liveness | 保留 |

`JointTrajectoryPoint` sequences在entity init時配置capacity=3，callback拒絕其他長度、NaN/Inf、超joint limit與過大速度。若需要joint names與multi-point trajectory，應由host controller拆成單點stream；第一版不要把大型`JointTrajectory`整包塞進MCU。

`diagnostic_msgs/DiagnosticArray`含多個動態字串，第一版不放MCU；host可把fault code轉成完整diagnostics。之後資源量測足夠再評估。

## Executor / callback

建議一個rclc executor：

- Phase 3 handles = 1 subscription。
- Phase 4 handles = joint command + enable，共2；heartbeat/joint state由task schedule publish，不需rcl timer。
- `rclc_executor_spin_some`每5–10 ms一次，必須有bounded timeout。
- CommTask native FreeRTOS priority建議2；ControlTask提高到明確的native priority，例如4，並確保所有task都使用同一套priority語意。不要用CMSIS normal 24作CommTask。
- callback只做長度、finite、範圍驗證和command buffer commit；不可IK、PID、HAL UART blocking、publish、delay或PWM。

資料交接：

```text
subscription callback
├─ deserialize into preallocated ROS message
├─ validate length / finite / SI limits
├─ convert rad → degree at adapter boundary
└─ short critical section:
   command_back = {3 targets, optional velocities, receipt_tick, seq, valid}
   swap active index

ControlTask @1 kHz
├─ short critical copy of active command
├─ check estop / enable / command age
├─ apply to local trajectory target
└─ create telemetry snapshot with seq

CommTask
└─ copy telemetry snapshot and publish
```

短critical copy比逐欄volatile安全。若實測critical時間過長，再改seqlock/double-buffer；不要使用mutex讓高優先control等待低優先CommTask。

## Connection state machine

```text
WAITING_AGENT
  └─ ping agent every 500 ms（non-blocking/bounded）
      ↓ success
CREATE_ENTITIES
      ↓ all success
CONNECTED
  ├─ spin_some
  ├─ publish snapshots
  └─ periodic ping / publish error threshold
      ↓ failure
COMM_LOST
  ├─ immediately invalidate command
  ├─ ControlTask enters safe stop
  ├─ fini entities in reverse order
  └─ backoff 0.5 → 1 → 2 s
      ↘ WAITING_AGENT
```

每次 reconnect都重新建立support/node/pub/sub/executor/message storage；所有partial init要有位元mask，fini只清成功建立者。重連不得阻塞ControlTask。

ROS time sync只用於published header；command timeout永遠使用`xTaskGetTickCount`/HAL monotonic subtraction，能自然處理32-bit wrap。時間未同步時可發布zero stamp並設fault bit，不得用錯誤epoch影響motor safety。

## 斷線安全策略

建議初值（實機依慣性/重力調整）：

1. command age >250 ms：拒絕舊command、trajectory target凍結到current position或速度斜降為0。
2. >500 ms或agent明確lost：`Motor_Stop`，state=COMM_LOST。
3. 新agent連上後仍保持SAFE/READY；不得自動恢復最後命令。
4. 必須收到新的valid command與新的enable edge才可RUNNING。
5. ESTOP永遠latched，ROS message不能單獨clear。

機械臂斷電/disable可能因重力下墜；在實機確認brake與自鎖之前，「hold current position」和「disable」哪個更安全不能只靠軟體假設。C-01硬體驗證是前置條件。

## 頻率分層

根據現有程式：

| 工作 | 建議 |
|---|---:|
| hard limit、command timeout、motor output | 1 kHz |
| encoder count update | 1 kHz |
| velocity estimate / inner feedback | 現有100 Hz；量測後100–500 Hz |
| IK/FK / trajectory target update | 100–200 Hz或新command事件 |
| executor spin | 100–200 Hz（5–10 ms bounded） |
| joint command | ≤20 Hz首版 |
| joint states | 25 Hz首版 |
| fault status | 2 Hz + change |
| heartbeat | 1 Hz |

以約100–150 serialized bytes估算`JointState` 25 Hz，加command與XRCE overhead仍低於115200 8N1約11.5 kB/s raw payload budget；實際值要由agent traffic/丟包量測，不以估算替代驗收。

## 記憶體策略

- init/reconnect階段可以bounded allocation；CONNECTED正常loop不配置。
- ROS sequences和strings固定capacity；joint names只配置一次。
- message、executor、support與transport buffer用static/application-lifetime storage。
- heap先調48 KB、micro-ROS task stack 12 KB作bring-up，不把此值當最終答案。
- 每2 s記錄`xPortGetMinimumEverFreeHeapSize`與stack high-water；24 h後保留至少30% stack margin與明確heap margin。
- 每次library重建記錄ROS distro、utility commit、GCC version、colcon.meta與ELF map。

