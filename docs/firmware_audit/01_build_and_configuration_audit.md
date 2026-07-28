# 建置與設定健檢

## 可重現建置

為保持原專案唯讀，將完整 `Firmware/doghead_arm` 複製到 `/tmp`，用 STM32CubeIDE 1.19.0 headless clean build。原專案沒有產生 `Debug/` 或 `Release/`。

| Configuration | 結果 | error | warning | text | data | bss（含 linker heap/stack reservation） | Flash = text+data | RAM = data+bss |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Debug, `-O0 -g3 -Wall` | 成功 | 0 | 0 | 48,476 B | 132 B | 27,000 B | 48,608 B / 512 KiB (9.27%) | 27,132 B / 128 KiB (20.70%) |
| Release, `-Os -g0 -Wall` | 成功 | 0 | 0 | 30,756 B | 136 B | 26,992 B | 30,892 B / 512 KiB (5.89%) | 27,128 B / 128 KiB (20.70%) |

Debug section detail：`.bss` 25,460 B、`._user_heap_stack` 1,540 B。其中 FreeRTOS `ucHeap` 本身是 20,000 B。

最大 application 靜態 stack frame 估算：`ControlTask` 112 B、`Robot_Loop` 96 B、`solveFK3D` 88 B、PID update 80 B；這不包含呼叫鏈、interrupt nesting、library 內部及 micro-ROS。

### 重要解讀

- 現有映像餘裕大，但 `libmicroros.a` 沒有被 linker 引用，所以數字不能代表導入後用量。
- defaultTask 動態 stack 12,000 B；ControlTask 2,048 B。每個 dynamic task 另需約 188 B TCB/`_reent` 加 heap metadata。
- 20 KB FreeRTOS heap 在現況已約先承擔 14.5 KB；若再開原註解中的 4 KB CommTask，留給 micro-ROS entity/message allocation 的空間很小。
- utility clone 的 README 明確要求 micro-ROS task **>10 KB stack**。因此原計畫 4 KB 不合格，不能只因連結成功就上板。
- Linker `_Min_Heap_Size=0x200` 是 newlib heap reservation，與 FreeRTOS `configTOTAL_HEAP_SIZE=20000` 是不同資源。

## MCU / startup / linker 一致性

| 來源 | 值 | 判定 |
|---|---|---|
| `.ioc:13` | STM32F446RET6 | 一致 |
| `.cproject:20` | STM32F446RETx | 一致 |
| startup | `startup_stm32f446retx.s` | 一致 |
| linker | FLASH 512K, RAM 128K | 符合 STM32F446RE |
| defines | `STM32F446xx`, `USE_HAL_DRIVER` | 一致 |
| FPU | FPv4-SP-D16, hard ABI | 一致 |

沒有硬編碼到某位使用者 home directory；`${workspace_loc}` 與相對 include path 可攜。`micro_ros_stm32cubemx_utils` 則是額外 clone，且本身含 nested `.git`，baseline 必須記錄其 commit（審查時 humble branch `bd531b2`）。

## Clock

生成碼實際使用 HSI：

```text
HSI 16 MHz / PLLM 16 × PLLN 336 / PLLP 4 = 84 MHz SYSCLK
AHB = 84 MHz
APB1 = 42 MHz, APB1 timers = 84 MHz
APB2 = 84 MHz, APB2 timers = 84 MHz
Flash latency = 2
```

`.ioc` 把 HSE PH0/PH1、LSE PC14/PC15 保留為 external oscillator pins，但 clock tree 沒使用它們。這不是 MCU 型號衝突，卻是未說明的 pin reservation；若實板沒有晶振可釋放，若希望時脈精準則應在 CubeMX 明確切換並量測。UART 115200 由 HSI/PLL 通常可運作，但高 baud 與長時間同步要做誤碼測試。

## FreeRTOS

| 設定 | 現值 | 評估 |
|---|---:|---|
| CMSIS API | V2 | 可保留 |
| Tick | 1 kHz | 符合控制通知與 timeout 粒度 |
| HAL timebase | TIM7 priority 15 | 正確避開 SysTick ownership |
| Heap | heap_4, 20,000 B | 現況可 build；micro-ROS 前需量測與調整 |
| `USE_NEWLIB_REENTRANT` | 1 | printf/newlib task safety需要，但增加每 task RAM |
| defaultTask | CMSIS priority 24, 12,000 B | 工作只有 `osDelay`; 浪費且優先級比 native ControlTask 3 高 |
| ControlTask | native priority 3, 2,048 B | 實際低於 CMSIS normal 24；註解「最高」不正確 |
| CommTask | native priority 2, 4 KB proposed | 未建立；stack 不符合 utility 指引 |

`configMAX_PRIORITIES=56` 讓 CMSIS priority 24 合法。CMSIS 與 native FreeRTOS 數值直接共用，不能把 normal 誤認為 1。

## NVIC / IRQ

| IRQ | `.ioc` priority | runtime priority | 內容 | RTOS API |
|---|---:|---:|---|---|
| EXTI15_10 | 5 | 5 | PA10/11/12/PC13 HAL dispatch | callback 只設 bool |
| TIM6_DAC | 10 | **5** | 1 kHz control notify | `vTaskNotifyGiveFromISR` |
| TIM7 | 15 | 15 | HAL tick | none |
| PendSV / SysTick | 15 | 15 | FreeRTOS kernel | kernel |

FreeRTOS `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`；TIM6 runtime priority 5 可合法呼叫 FromISR API。問題是 `.ioc` 仍為 10，生成區先設 10，USER 區又設 5（`stm32f4xx_hal_msp.c:239-247`）。應在 CubeMX 改為 5，再移除重複 override。

沒有 USART2 IRQ handler、DMA stream IRQ、SPI/I2C IRQ。Startup 裡的 weak vector 不代表功能已開啟。

## UART / DMA

USART2 生成碼為 115200、8 data bits、no parity、1 stop、TX/RX、no flow control、oversampling 16。PA2/PA3 在標準 Nucleo-F446RE 配置上通往 ST-Link VCP，方便 Ubuntu/Raspberry Pi 透過 USB serial 使用；仍需核對實板 solder bridge/自製配線。

目前：

- `.ioc` 沒有 USART2 global interrupt。
- 沒有 RX/TX DMA request、DMA handle 或 handler。
- `NVIC.ForceEnableDMAVector=true` 不等於配置了 DMA。
- `printf` 沒有 `__io_putchar` backend；不能當成有效 UART log。
- XRCE-DDS serial transport 不可和純文字 log 混在同一 UART byte stream。

## Timer / GPIO 一致性

- TIM1/4/5 encoder 與 main 生成碼一致。
- TIM2/3/12 PWM 與 application motor mapping 一致。
- TIM6 84 MHz / (83+1) / (999+1) = 1 kHz。
- motor driver 在 runtime 依 RPM 改 PWM timer PSC/ARR；這是 frequency-command driver 行為，不應另建第二套 `MX_TIMx_Init`。
- TIM8、TIM13 及 PC5 已初始化但 application 無引用；先確認硬體再由 CubeMX 移除。
- Motor Enable 腳位需求已取消：PC4 及 PC0 / PC5 在 CubeMX 中可直接切換為 Disable / Reset (Free) 腳位。


## C / C++ 與 source participation

- C++14、`-fno-exceptions -fno-rtti -fno-use-cxa-atexit`，符合 embedded 使用。
- C bridge headers 大致有 `extern "C"`；clean build 證實沒有 name-mangling link error。
- source roots 同時包含 `Core`、`Application`、`Drivers`、`Middleware`、`Middlewares`。
- `Core/Src/freertos.c` 與 `Middleware/FreeRTOS/Src/freertos.c` 同時編譯；目前函式不重複，但 ownership 容易誤判。
- MicroROS allocator/IT transport 被 `#if 0` 停用；time shim有編譯但因無引用被 section GC。
- cproject 有重複 include path；micro-ROS include 已加，但 linker library list/search path為空。
- Debug / Release 都只有 `-Wall`，尚未用 `-Wextra -Wconversion -Wshadow`；不建議立刻 `-Werror` 阻塞 vendor code，可先只套 application sources。

## `.ioc` / 生成碼總判定

MCU、主要 clock frequency、timer mode、UART基本設定、RTOS 與 linker一致。以下不是一致：

1. TIM6 NVIC `.ioc=10`、runtime被 USER code改成 5。
2. application 使用 PC0 做 M3 enable，但 CubeMX 未配置。
3. M1/M2 共用 PC4 的意圖沒有在 CubeMX label/硬體說明中呈現。
4. 文件宣稱 USART IRQ/DMA 與 active micro-ROS，實際 `.ioc`/ELF 都沒有。
5. HSE/LSE pins被保留但 clock未使用，需確認意圖。

所有建議操作集中在 [05_cubemx_change_guide.md](05_cubemx_change_guide.md)。

