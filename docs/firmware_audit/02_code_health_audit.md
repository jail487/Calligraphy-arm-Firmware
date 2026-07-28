# 程式健康與安全健檢

## 問題統計

| Critical | High | Medium | Low | 合計 |
|---:|---:|---:|---:|---:|
| 3 | 11 | 12 | 3 | 29 |

「Critical」表示在上馬達或啟用遠端通訊前必須關閉；不表示已在桌面 build 中重現事故。

## Critical

| ID | 檔案 / 函式 / 行號 | 問題與後果 | 建議處理 | CubeMX | 難度 / 驗證 |
|---|---|---|---|---|---|
| C-01 | `Application/Inc/motor_config.h:19-34`; `Application/Src/motor_config.c:19-22,44-47,68-71`; `Core/Src/main.c:740-757` | 原預留 Motor Enable 腳位（PC4 共用、PC0 未配置）；現確認硬體改為無需 MCU GPIO 控制 Enable。 | 在 CubeMX 將 PC4 及 PC5 改為 Disable / Reset (Free) 腳位；程式碼中清理 Enable 設定；PC1/PC2/PC3 保留為 DIR。 | **是** | 低；將 PC4/PC5 腳位切回 Unconfigured，編譯並驗證動作 |
| C-02 | `Middleware/FreeRTOS/Src/freertos.c:294-397`; `Application/Src/test_scenarios.cpp:19-42` | test RPM一旦進入 RUNNING會持續套用，沒有 command age / communication watchdog；未來遠端斷線可能永遠保持最後命令。 | safety manager加入 monotonic receipt tick、250 ms command timeout、COMM_LOST state；timeout先命令零，再依機械安全策略 disable/brake。 | 否（IWDG另列） | 中；拔除 agent/停止 publish後量測停止時間 |
| C-03 | `Application/Src/mainpp.cpp:14,40-74`; `homing_calibration.cpp:11-29`; `freertos.c:121-195` | 每次開機自動進 HOMING，ControlTask無外部 arm許可便以 500 motor RPM驅動三軸。只有軟體 timeout/travel limit，沒有硬體 e-stop input與 operator arming。上電即動屬機械安全風險。 | 預設 SAFE/DISABLED；實體 enable + home command + switch sanity成立才開始；e-stop硬體直接切斷 driver enable，軟體只回報。 | **是**（EXTI/IWDG/可能GPIO） | 高；安全治具、單軸低速、斷線與卡 switch測試 |

## High

| ID | 檔案 / 函式 / 行號 | 問題與後果 | 建議處理 | CubeMX | 難度 / 驗證 |
|---|---|---|---|---|---|
| H-01 | `microros_app.c:9-33`; `freertos.c:480-487`; `.cproject:111-119` | micro-ROS是永遠 false的 stub、CommTask未建立、linker沒有 `-lmicroros`。舊文件宣稱已完成，會導致錯誤 baseline。 | Phase 3以一 pub/一 sub做真實 agent smoke test；用 ELF symbols與 topic list驗收。 | 部分 | 高；clean build、agent log、topic echo/pub |
| H-02 | `.ioc:7-9`; `main.c:64-70`; `FreeRTOSConfig.h:71,97`; `freertos.c:474-486` | heap 20 KB；無用 defaultTask先占12 KB，ControlTask 2 KB，預定 CommTask僅4 KB；utility要求 micro-ROS task >10 KB。entity init很可能 allocation fail。 | 先量 high-water；縮 defaultTask或改為 comm task；micro-ROS stack先12 KB；heap先48 KB再以 minimum-ever-free收斂。檢查所有 create return。 | **是** | 中；boot後記錄 heap minimum與每 task high-water，壓力測24 h |
| H-03 | `freertos.c:117-120,259-267`; `main.c:816-837` | `ulTaskNotifyTake(pdTRUE)`會把累積通知一次清空，Robot_Loop卻固定 dt=1 ms；overrun時漏週期且軌跡/PID時間仍當1 ms，控制時基失真。 | 用實際 monotonic delta並限幅；監測notification count、deadline miss與WCET；明確決定catch-up或drop。 | 否 | 中；GPIO pulse/DWT量WCET，人工製造overrun |
| H-04 | `Drivers/Src/nidec_motor_driver.c:194-204`; TIM5 32-bit | 將兩個 `uint32_t` counter cast成 `int32_t`再相減可能發生 signed overflow（C undefined behavior）；TIM5全32-bit範圍特別容易觸發。 | 以 unsigned modular subtraction後再明確轉換，或使用方向位/窄採樣假設並加static test。 | 否 | 中；跨0、0x7fffffff、0xffffffff單元測試 |
| H-05 | `.ioc:324-325`; `stm32f4xx_hal_msp.c:540-562`; `stm32f4xx_it.c` | USART2沒有 global IRQ、DMA、USART handler，不能承載目前預想的 IT/DMA custom transport。 | 依 05 文件只用CubeMX配置RX circular DMA、TX normal DMA及IRQ。 | **是** | 中；loopback、CRC/sequence、長時間無丟包 |
| H-06 | `Middleware/MicroROS/Src/it_transport.c:17-72` | 停用 transport若直接復活：ring indices有race、無full detection、callback不篩UART、TX等待blocking、timeout未實作。可能資料破壞或控制抖動。 | 不解除 `#if 0`；改用版本匹配的utility DMA transport並包裝callback dispatcher。 | **是** | 中；burst、buffer wrap、agent restart |
| H-07 | `robot_config.h:55-93`; `robot_config.cpp:10-47`; motor struct | 大量 `volatile` float/多欄位狀態未形成一致snapshot。單個對齊32-bit讀寫雖原子，三軸+timestamp組合不是；64-bit encoder count也不是M4原子。 | command double buffer或短critical-section copy；telemetry由ControlTask產生snapshot，CommTask只讀snapshot。 | 否 | 中；sequence counter一致性測試 |
| H-08 | `main.c:845-853`; `stm32f4xx_it.c:90-145`; `freertos.c:467-478` | init error/HardFault只死循環；task create與TIM start return未檢查；沒有reset reason、stack/malloc hook或IWDG。現場只看到「不動」。 | 統一 fault record、safe outputs、LED/SWO、reset cause；檢查return；最後才由CubeMX加IWDG。 | 部分 | 中高；故障注入、故意縮stack/heap |
| H-09 | `.ioc:87`; `stm32f4xx_hal_msp.c:239-247` | Cube生成TIM6 priority 10後USER區再改5；重新Generate、人工清理或順序改變可能讓FromISR API在錯誤優先級執行。 | CubeMX NVIC把TIM6設5，再移除USER override。 | **是** | 低；讀NVIC register + configASSERT |
| H-10 | `kinematics.cpp:28-45` | L1=L2且target=(0,0)時dist=0仍被判reachable，接著除以0；NaN被clip掩蓋後可能產生虛假解。 | 先檢查finite與 `dist > epsilon`，再做workspace/joint limits；用explicit error result。 | 否 | 低；zero/near-zero/NaN/邊界測試 |
| H-11 | `freertos.c:201-267`; `mainpp.cpp:92-199` | 84 MHz上每1 ms執行多次encoder、IK/FK、sqrt/atan2/acos/sin/cos及三軸控制，未量WCET或deadline miss。負載增加micro-ROS後可能抖動。 | 高階IK/trajectory降到100–200 Hz或事件更新；1 kHz只做必要feedback/safety；DWT/GPIO量測。 | 否 | 中；release build下worst-case target量測CPU margin |

## Medium

| ID | 檔案 / 函式 / 行號 | 問題與後果 | 建議處理 | CubeMX | 難度 / 驗證 |
|---|---|---|---|---|---|
| M-01 | `main.c:529-599,643-687`; `.ioc:17-20,275-323` | TIM8、TIM13、PC5初始化但application無引用，占pins/clock並混淆硬體圖。 | schematic確認後由CubeMX disable，不手刪init。 | 是 | 低；全功能回歸 |
| M-02 | `Core/Src/freertos.c`; `Middleware/FreeRTOS/Src/freertos.c`; `main.c:172` | 兩個同名freertos source同時編譯，自訂 `MX_FREERTOS_Init`與Cube task混用，task ownership難維護。 | 保留Cube-generated task定義於單一位置；business loop放獨立module。 | 是 | 中；task list一致 |
| M-03 | `microros_time.c`; `microros_stubs.c`; allocator files | 自製rcutils time、TLS與atomic symbols可能和正式library重複；allocator副本目前全停用。 | 對最終lib做 `nm`，只加入必要且版本配套的sources。 | 否 | 中；link map無duplicate/weak override |
| M-04 | repo舊三份micro-ROS文件與 `Project_Documentation_v3.md` | 文件路徑、topic、IRQ/DMA及完成度與程式不符。 | 標記historical，Phase 3驗證後重寫。 | 否 | 低；文件命令可實際重跑 |
| M-05 | `syscalls.c:35,80-89`; `freertos.c:416-457` | `printf`無 backend；未來啟用CommTask可能呼叫weak undefined `__io_putchar`。同UART文字也會破壞XRCE framing。 | debug改SWO或另一UART；USART2 transport禁止文字。 | 可能 | 低；ELF symbol + serial capture |
| M-06 | `main.c:269-277,416-424,465-473` | 三個encoder digital filter全0，機械/線材噪聲可能形成假count。 | scope量測後在CubeMX設適當IC filter，不盲填。 | 是 | 中；靜止/運轉噪聲count |
| M-07 | `nidec_motor_driver.c:22-25` | timer input clock永遠假設APB1×2；driver一旦移到TIM1/8會算錯frequency。 | 依timer instance判APB bus/prescaler，或由BSP提供clock。 | 否 | 低；各timer clock test |
| M-08 | application各控制模組 | firmware用degree/mm/RPM，ROS標準用rad/m；若無單一boundary易混單位。 | `command_adapter`/`telemetry_adapter`唯一轉換，命名加suffix。 | 否 | 低；round-trip unit tests |
| M-09 | `robot_config.h:52-93` | 二十多個extern globals使control、homing、test、future comm高度耦合，不易host test。 | 合併為command/state/config structs與窄API，不做一次性大搬家。 | 否 | 中；host unit tests |
| M-10 | `freertos.c:81-90,299-320`; motor max `motor_config.c:25,50,74` | test mode目標5000 RPM但motor上限1710，長期飽和且註解/測試預期錯誤。 | 使用config max_rpm與明確saturation status。 | 否 | 低；boundary test |
| M-11 | `FreeRTOSConfig.h`; tasks | 沒有週期記錄heap minimum、stack high-water、missed cycles；靜態build數字無法證明runtime安全。 | diagnostics低頻採樣並設threshold fault。 | 否 | 低；長時間log |
| M-12 | `main.c:776-785`; `homing_calibration.h:19-21`; `freertos.c:154-166` | ISR設三個homing trigger flags，但task不讀，實際完全靠polling；雙機制容易讓維護者誤判。 | 選一個：polling作安全主路徑並刪旗標，或集中event consume；先保留polling。 | 否 | 低；switch bounce test |

## Low

| ID | 檔案 / 函式 / 行號 | 問題與後果 | 建議處理 | CubeMX | 難度 / 驗證 |
|---|---|---|---|---|---|
| L-01 | `motor_config.c:41,77-78` | 註解稱TIM4是PA8/PA9（實際PB6/PB7），M3「方向取反」但值false。 | 修註解並用pin table當真源。 | 否 | 低；review |
| L-02 | `.cproject:35-95,170-228` | Application/MicroROS include path重複，scanner/build設定較吵。 | baseline後清理一份相對path。 | 否 | 低；雙config clean build |
| L-03 | compiler settings | 只有`-Wall`，application的conversion/shadow問題不易出現。 | 先以CI附加 `-Wextra -Wshadow -Wconversion` 作非阻塞報告。 | 否 | 低；warning baseline |

## 重要模組逐項審查

| 模組 | 用途 / 介面 / 依賴 | 頻率 / context | 主要問題 | 建議 / 優先級 / 影響 |
|---|---|---|---|---|
| Motor driver | `Motor_Init/Update/SetSpeed/Start/Stop`; HAL TIM/GPIO | update 1 kHz呼叫、速度估測100 Hz；task | TIM5 overflow、clock假設、hardware mapping | 先修C-01/H-04；影響三軸 |
| PWM/frequency | dynamic PSC/ARR、DIR、ENABLE | 每次命令；task | runtime timer config與Cube初始值需清楚分工 | 保留driver動態行為；scope驗頻率 |
| Encoder | timer quadrature→int64 count→angle/RPM | count 1 kHz、RPM 100 Hz | filter 0、64-bit snapshot非原子 | filter實測、snapshot |
| Position PID | angle error + feedforward + anti-windup | 1 kHz task | dt固定；deadband與gains未有實機證據；Kd目前0 | 修時基、step test、unit test |
| Trajectory | trapezoidal pos/vel/acc | 1 kHz | target頻繁改變時reset；沒有explicit status | 保留，小步測邊界 |
| Kinematics | IK/FK，degree/radian adapter | 1 kHz task | zero singular、計算負載、D=0意圖需機構確認 | H-10/H-11 |
| Homing | switch polling、calibrate angle | boot 1 kHz task；EXTI只設旗標 | boot auto motion、flags unused | C-03，建立arming |
| Safety | state/fault + disable all | ControlTask；未來command task | 無COMM_LOST、IWDG、硬e-stop input | C-02/C-03/H-08 |
| UART | HAL USART2 init | 尚無通訊 | no IRQ/DMA、log conflict | H-05/M-05 |
| micro-ROS | stub API | 不在runtime | 未link/未create entities | H-01 |
| Diagnostics | printf source但task disabled | none | backend與fault record皆無 | fault snapshot + SWO/topic |
| IMU | 不存在 | N/A | 本機械臂現階段無需求證據 | 不新增，等硬體需求 |
| Odometry/mecanum | 不存在且不適用機械臂 | N/A | 舊泛用需求與實際專案不同 | 不加入 |

## 記憶體與資料安全

- application未發現 active `malloc/free/new/delete`、大 local array、`sprintf/strcpy` 或無界 memcpy。
- FreeRTOS本身使用 heap_4 dynamic task allocation；micro-ROS allocator殘留目前 `#if 0`。
- C++ exceptions/RTTI已關閉，沒有動態容器。
- `sensor_msgs/JointState` 與 `trajectory_msgs/JointTrajectoryPoint` 都含 sequences；正式導入時只能在init/reconnect配置固定容量3，正常control cycle不得反覆allocate。
- F446只有single-precision hardware FPU；ROS標準joint messages序列化常用float64，轉換放CommTask，不放ISR。

## 建議 fault / diagnostics 最小架構

```text
FaultManager
├─ state: SAFE / HOMING / READY / RUNNING / COMM_LOST / FAULT / ESTOP
├─ latched_fault_bits
├─ first_fault_time_ms + reset_reason
├─ command_age_ms / agent_state
├─ control_deadline_miss / encoder_health
└─ safe_output(): zero command → stop/disable all motors

ControlTask
├─ every tick: hard limit, estop, command timeout
├─ publishes immutable telemetry/fault snapshot
└─ never waits for ROS

CommTask
├─ consumes/publishes snapshots
├─ reconnects entities
└─ cannot clear ESTOP or directly write PWM
```

E-stop優先級最高；一般command、enable、homing都不能覆蓋。通信丟失用本機monotonic tick判斷，不依賴ROS timestamp。

