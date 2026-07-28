# 分階段改善路線

原則：每一phase單獨commit、先測再擴；任何馬達測試先確認C-01/C-03，具備可手動切斷動力的方法。

## Phase 0：保留目前可建置版本

- 目標：可回退baseline，不改功能。
- 前置：本audit完成。
- 修改：記錄utility nested repo commit、CubeIDE/GCC、Debug/Release map與binary hash；建立Git tag。
- CubeMX：無。
- 測試：重新執行兩種clean build；若可安全上板，記錄boot狀態但不驅動。
- 驗收：0 error/0 warning；build數字與01文件同級；repo無意外generated diff。
- 回退：checkout baseline tag。
- 風險：目前repo多數檔案顯示untracked，建立baseline前要先確認真正版本控制範圍。

## Phase 1：修正阻礙安全與明顯bug

- 目標：在沒有ROS下讓motor mapping、boot/homing與1 kHz時基可信。
- 前置：schematic、pinout、可斷電治具。
- 修改：C-01 PC0/PC4 mapping、C-03 arming、H-03 real dt/deadline counter、H-04 TIM5 modular delta、H-09 TIM6 priority、H-10 singularity；檢查所有create/start return。
- CubeMX：GPIO、TIM6 NVIC；encoder filter只在量測後。
- 測試：GPIO電壓、單軸低速、counter wrap unit test、IK boundary unit test、homing switch stuck/open、control WCET。
- 驗收：任何boot不自動動；每軸enable獨立；fault必停；1 kHz miss可觀察。
- 回退：每個修正一commit，Cube生成diff獨立commit。
- 風險：pin mapping錯誤可動馬達，必須先無motor power測。

## Phase 2：整理通訊與控制介面

- 目標：ROS-independent command/telemetry contract與安全timeout。
- 前置：Phase 1。
- 修改：`RobotCommand` double buffer、`RobotTelemetry` snapshot、SI adapter、COMM_LOST state、250/500 ms timeout、callback dispatcher；移除/隔離stale transport，不刪歷史。
- CubeMX：規劃task table與heap/stack，但此phase可先不開UART DMA。
- 測試：host-side command validation、NaN/length/limit、并發snapshot、停止command stream。
- 驗收：假的producer中斷後在規定時間safe stop；控制可完全脫離ROS測試。
- 回退：保留舊local API adapter。
- 風險：狀態機改動；用shadow mode先比對outputs。

## Phase 3：最小 micro-ROS 通訊

- 目標：真實agent連線、heartbeat pub、ping sub，不控制馬達。
- 前置：鎖定ROS distro/utility commit/GCC；Phase 2 interface。
- 修改：正式link `libmicroros.a`、DMA transport、allocator、單executor一handle、connection state machine。
- CubeMX：USART2 RX/TX DMA、IRQ、CommTask 12 KB、heap 48 KB bring-up。
- 測試：agent start/stop、topic pub/echo、sequence loss、100次reconnect、heap/stack記錄。
- 驗收：24 h無heap下降/HardFault；agent restart自動恢復；ControlTask deadline不退化。
- 回退：compile-time `MICROROS_ENABLED=0`回到local-only，但不要用stubs假裝connected。
- 風險：library flags/ABI不符、callback重複、USART2 log污染。

## Phase 4：接入馬達控制

- 目標：joint command經buffer/validation/timeout驅動既有trajectory與PID，回傳joint states。
- 前置：Phase 3穩定、實體e-stop、C-01/C-03完成。
- 修改：`JointTrajectoryPoint` sub、`JointState` pub、enable/fault topics；callback不碰HAL；reconnect後保持disabled。
- CubeMX：通常無新增；Phase 3設定定版。
- 測試：小角度/低速、正負方向、joint order、rad↔deg、limit、NaN、publish flood、拔線。
- 驗收：command age >250 ms開始安全停止、>500 ms進COMM_LOST；舊command不會在reconnect後恢復；joint error符合允收。
- 回退：disable ROS command adapter，保留local command source。
- 風險：機械臂disable可能下墜；safe-stop策略須按實機brake確認。

## Phase 5：軌跡、額外感測與高階整合

- 目標：與ROS 2 planner/controller整合；只加入實際存在的sensor。
- 前置：Phase 4長時間穩定。
- 修改：host把trajectory拆成bounded joint points；MCU維持local interpolation。若新增IMU才加`sensor_msgs/Imu`；本arm不加odometry。
- CubeMX：依真實sensor才開I2C/SPI/DMA，不預先占資源。
- 測試：timestamp sync、joint names/order、rosbag、planned vs measured、agent clock reset。
- 驗收：時間單調、frame語意一致、無dynamic allocation steady-state。
- 回退：回到single-point command。
- 風險：message/entity膨脹與bandwidth；每次只加一個。

## Phase 6：長時間與異常測試

- 目標：證明可維護與fail-safe。
- 前置：所有目標功能完成。
- 修改：只補測試暴露的bug；考慮IWDG定版。
- CubeMX：必要時IWDG；其他不變。
- 測試矩陣：agent restart、USB/UART拔除、RX flood/corruption、MCU reset、host reset、連續24–72 h、home switch stuck/bounce、encoder disconnect、task overrun、heap exhaustion、baud sweep。
- 驗收：無失控、無memory leak、可重連、fault可診斷；Flash/RAM/stack/CPU margin有數字。
- 回退：IWDG與高baud可分別關閉回baseline。
- 風險：測試本身可能動機械；使用限位治具、低速、兩人確認與硬斷電。

## 建議下一步

先做Phase 0，接著只做Phase 1的 **GPIO mapping實物核對與boot arming**。在這兩項未關閉前，不應解除CommTask註解或讓ROS命令接管馬達。

