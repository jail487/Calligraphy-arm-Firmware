# STM32 韌體專案總覽

審查日期：2026-07-28  
審查目標：`Firmware/doghead_arm`  
審查方式：韌體與 `.ioc` 唯讀；建置在 `/tmp` 副本進行；本目錄是本階段唯一新增內容。

## 結論先行

實際專案不是四輪麥克納姆底盤，而是 **三軸書法機械臂**。因此目前沒有底盤、`cmd_vel`、里程計、IMU 或四輪運動學；micro-ROS 設計應以 joint command / joint state 為核心，不應照搬移動底盤 topic。

| 項目 | 實際狀態 |
|---|---|
| MCU | STM32F446RET6，LQFP64，Nucleo-F446RE board profile |
| CPU | Cortex-M4F，single-precision FPU，hard-float ABI |
| 時脈 | HSI 16 MHz → PLL → SYSCLK/HCLK 84 MHz；APB1 42 MHz、APB1 timer 84 MHz、APB2 84 MHz |
| IDE / Cube | STM32CubeIDE project；`.ioc` 由 CubeMX 6.14.1、STM32Cube FW_F4 V1.28.3 產生 |
| RTOS | FreeRTOS V10.3.1 + CMSIS-RTOS2；1 kHz kernel tick；HAL tick 使用 TIM7 |
| 控制 | TIM6 1 kHz IRQ 通知 native FreeRTOS `ControlTask` |
| 通訊 | USART2 PA2/PA3、115200 8N1；目前 polling HAL 初始化，未開 USART IRQ / DMA |
| micro-ROS | 原始碼 stub 與停用殘留存在，但 entity、transport、library link 與 CommTask 均未啟用 |
| 主要致動器 | 三顆 Nidec 13-pin frequency-command motor |
| 回授 | TIM1、TIM4、TIM5 encoder mode |
| 感測器 | 三個 active-low home switches（PA10/PA12/PA11） |

## 目錄與角色

```text
Firmware/doghead_arm/
├─ doghead_arm.ioc               CubeMX 唯一設定來源
├─ Core/
│  ├─ Src/main.c                 周邊初始化、RTOS 啟動、HAL callbacks
│  ├─ Src/stm32f4xx_it.c         EXTI、TIM6、TIM7 IRQ
│  ├─ Src/freertos.c             Cube 產生但目前沒有 application task
│  └─ Startup/                   STM32F446RE startup
├─ Application/
│  ├─ mainpp / robot_config      控制主流程、全域狀態、C/C++ bridge
│  ├─ homing_calibration         歸零
│  ├─ safety_manager             宣告；實作目前放在 mainpp.cpp
│  ├─ kinematics                 3-DOF IK/FK
│  ├─ trajectory_planner         梯形速度規劃
│  ├─ pid_controller             position PID + feedforward
│  └─ motor_config               馬達與 timer/GPIO mapping
├─ Drivers/
│  └─ nidec_motor_driver         PWM/frequency、encoder、速度估測
├─ Middleware/FreeRTOS/
│  └─ Src/freertos.c             自訂 ControlTask、停用的 CommTask
├─ Middleware/MicroROS/
│  └─ stub、停用 allocator/IT transport、time shim
├─ Middlewares/Third_Party/
│  └─ FreeRTOS kernel
└─ micro_ros_stm32cubemx_utils/  Humble branch utility clone + libmicroros.a
```

HAL/CMSIS/FreeRTOS vendor source沒有發現直接修改需求，本階段也未修改。

## 開機與執行流程

```text
Reset_Handler
└─ main
   ├─ HAL_Init
   │  └─ TIM7 建立 1 ms HAL timebase（IRQ priority 15）
   ├─ SystemClock_Config（HSI PLL，84 MHz）
   ├─ GPIO / USART2 / TIM1,2,3,4,6,5,8,12,13 init
   ├─ Robot_Init
   │  ├─ Motor_System_Config
   │  ├─ 三顆 motor init 並 disable
   │  ├─ safety state = HOMING
   │  └─ reset PID / trajectory / target
   ├─ osKernelInitialize
   ├─ osThreadNew(defaultTask, priority 24, stack 12,000 B)
   ├─ MX_FREERTOS_Init
   │  └─ xTaskCreate(ControlTask, native priority 3, stack 2,048 B)
   └─ osKernelStart
```

排程與資料流：

```text
TIM6 update @ 1 kHz, runtime NVIC priority 5
└─ HAL_TIM_PeriodElapsedCallback
   └─ vTaskNotifyGiveFromISR(ControlTask)
      └─ ControlTask
         ├─ homing active?
         │  ├─ update encoders / poll switches
         │  ├─ timeout + travel-limit checks
         │  └─ drive unfinished joints toward switches
         ├─ mode 0: Robot_Loop(0.001)
         │  ├─ encoder snapshot
         │  ├─ optional IK / FK
         │  ├─ trajectory
         │  ├─ position PID
         │  ├─ limit check
         │  └─ Motor_SetSpeed
         └─ mode 1..4: direct test RPM path

EXTI15_10, priority 5
└─ PA10/11/12 callback
   └─ only set homing flags
      （ControlTask 現在沒有消費這些 flags，實際靠 1 kHz polling）

defaultTask, FreeRTOS priority 24
└─ osDelay(1000), no application work

CommTask, native priority 2
└─ source exists but xTaskCreate is commented out
```

架構分類：**FreeRTOS + hardware-interrupt-triggered control task 的混合架構**。ROS executor 不在執行路徑中。

## 現有控制資料流

```text
API / debugger target
→ global target variables
→ target validation（joint command only）
→ optional IK
→ trapezoidal planner
→ position controller
→ motor-shaft RPM command
→ frequency PWM + direction + enable

encoder timer
→ Motor_Update
→ accumulated counts
→ output-shaft angle / filtered RPM
→ FK / control / debug globals
```

內部單位目前是：

- 機構長度：mm
- 關節角：degree
- 關節速度：degree/s
- 馬達命令：motor-shaft RPM
- encoder：100 PPR × quadrature 4 × gear ratio 114
- 控制時間：seconds；HAL/RTOS time：milliseconds/ticks

ROS 邊界必須轉成 SI：position rad、velocity rad/s、Cartesian m；不得讓 ROS message type 滲入 motor driver。

## Peripheral 摘要

| Peripheral | 用途 | Pins / 設定 |
|---|---|---|
| TIM1 | M1 encoder | PA8/PA9, TI12, 16-bit, filter 0 |
| TIM4 | M2 encoder | PB6/PB7, TI12, 16-bit, filter 0 |
| TIM5 | M3 encoder | PA0/PA1, TI12, 32-bit, filter 0 |
| TIM2 CH1 | M1 frequency PWM | PA5 |
| TIM3 CH1 | M2 frequency PWM | PB4 |
| TIM12 CH1 | M3 frequency PWM | PB14 |
| TIM6 | control trigger | PSC 83, ARR 999 → 1 kHz |
| TIM7 | HAL tick | generated 1 kHz timebase |
| TIM8 CH1..3 | configured PWM but application unused | PC6/PC7/PC8 |
| TIM13 CH1 | configured PWM but application unused | PA6 |
| USART2 | candidate micro-ROS / Nucleo VCP | PA2/PA3, 115200 8N1, no flow control |
| EXTI15_10 | home switches + blue button | PA10/11/12 both edges; PC13 falling |
| DMA | HAL module compiled, no stream configured | none |
| SPI/I2C/CAN/USB/Ethernet | not configured | none |

STM32F446 的 Cortex-M4 沒有 data cache，因此 UART DMA 不涉及 D-cache clean/invalidate；仍需確保 DMA buffer 為 static、對齊且生命週期完整。

## 現況判定

- 韌體可 build，但「可 build」不代表三軸硬體 mapping 與安全鏈已驗證。
- FreeRTOS 已經是既有架構，沒有理由退回 bare-metal。
- 現有 application/control 分層已有可保留基礎。
- `main.c` 871 行主要是 Cube 生成初始化，真正應改善的是 task ownership、全域 snapshot、safety 與 communication boundary，不是為了行數搬檔。
- micro-ROS 第一版應服務機械臂 joint interface；odometry、mecanum、`cmd_vel` 不適用此專案。

