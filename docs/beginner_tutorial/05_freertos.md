# 05：FreeRTOS 與即時控制

## RTOS 與一般 while loop 的差異

Bare-metal 常見 super-loop：

```c
while (1) {
    read_uart();
    control();
    publish();
}
```

問題是 `read_uart()` 若卡住，control 也延遲。RTOS 把工作拆成 task，由 scheduler 依 priority 與 ready/block state 決定 CPU 給誰。RTOS 不會自動讓 code「即時」；你仍須設計 bounded execution、priority、同步與錯誤處理。

## 本專案的 task

`Core/Src/freertos.c` 建立：

| task | priority | stack 設定 | 工作 |
|---|---|---|---|
| `controlTask` | AboveNormal | `2000 * 4` bytes | 1 ms `Robot_ControlTask_Tick(0.001f)` |
| `commTask` | Normal | `3000 * 4` bytes | micro-ROS spin、50 Hz state、1 Hz diagnostics |

ControlTask priority 較高，通訊負載增加時仍應優先控制。高優先權 task 也不能永久忙迴圈，否則低優先權 task starvation；ControlTask 以 `vTaskDelayUntil()` 阻塞到下一週期。

## Tick 與週期

`FreeRTOSConfig.h`：

```c
#define configTICK_RATE_HZ 1000
```

因此通常 1 tick = 1 ms，`pdMS_TO_TICKS(1)` 為 1 tick。`vTaskDelayUntil(&last, period)` 以絕對週期基準喚醒，適合週期 task；`vTaskDelay(period)` 是從呼叫當下再延遲，task 執行時間會累積造成 drift。

概念例子：task 本身執行 0.2 ms。

```text
vTaskDelay(1ms):       start 0.0, 1.2, 2.4, 3.6 ...
vTaskDelayUntil(1ms):  start 0.0, 1.0, 2.0, 3.0 ...
```

但 tick resolution 不等於 deadline 一定達成。必須量測 WCET、deadline miss、interrupt blocking 與 scheduler latency。

## 1 kHz control path

每個 tick：

1. `Robot_ReadFeedback()` 取得 encoder/velocity。
2. 取出並二次確認 home switch event。
3. 依 safety state 做 homing 或正常 control。
4. `Robot_Loop()` 取最新 command、檢查 TTL/session/mode。
5. PVA interpolation 與 PID 計算 RPM。
6. joint envelope、limit switch、finite/RPM limit。
7. `UltimateGate_ApplyActuatorCommand()` 決定輸出或 stop。
8. 更新 telemetry snapshot。

任何錯誤都應在「真正寫 PWM/driver」之前被收斂為 zero mask/stop。

## Queue 長度 1：latest value mailbox

`robot_interface.cpp` 用長度 1 的 static queue 和 `xQueueOverwrite()`。它不是要保存所有歷史命令，而是讓 ControlTask 永遠取得「最新 setpoint」。

```text
CommTask 收到 seq 101 -> mailbox [101]
又收到 seq 102       -> mailbox [102]（101 被覆蓋）
ControlTask 取出      -> 使用 102
```

對 setpoint control，處理過期中間值通常比跳到最新值更糟。若是交易、log 或不可丟事件，就不應使用長度 1 overwrite queue。

使用 `xQueueCreateStatic()` 讓 queue storage 在編譯期存在，降低 heap fragmentation 與 runtime allocation 不確定性。

## Shared data 與 critical section

CommTask 寫 command/讀 telemetry；ControlTask 讀 command/寫 telemetry，形成 concurrency。code 用 queue 傳 command，對 snapshot/counter 則用：

```c
taskENTER_CRITICAL();
memcpy(...);
taskEXIT_CRITICAL();
```

critical section 期間會限制可搶占/中斷行為，必須短。不可在其中做 UART transmit、迴圈等待或複雜計算。`volatile` 只能限制 compiler optimization，不能保證原子性、互斥或 memory consistency，不能取代 queue/mutex/critical section。

## ISR 與 Task context

GPIO EXTI callback 只鎖存 bit：

```c
g_home_event_mask |= ...;
```

ControlTask 之後用 `HW_TakeHomeEvents()` 取出並重新讀 pin 過濾假中斷。這是好的 ISR 原則：ISR 做最少工作，把複雜 state transition 留給 task。

在 ISR 中只能用 `...FromISR()` API，例如 `xQueueOverwriteFromISR()`，並用 `portYIELD_FROM_ISR()` 要求必要的 context switch。一般 `xQueueOverwrite()` 可能碰 scheduler/blocking 邏輯，不能隨意從 ISR 呼叫。

`__get_IPSR() != 0` 用來判斷目前是否在 exception/interrupt context。

## Priority inversion 與阻塞

若高 priority ControlTask 等待被低 priority CommTask 持有的 mutex，而中 priority task 又一直執行，就可能 priority inversion。FreeRTOS mutex 可用 priority inheritance 緩解，但最佳設計仍是：1 kHz path 避免等待不受控 I/O，透過 mailbox/snapshot 交換固定大小資料。

本專案 ControlTask 不直接跑 micro-ROS executor，是重要的隔離。

## Stack 與 heap

每個 task 有獨立 stack。stack overflow 可能破壞其他記憶體，因此開啟：

```c
#define configCHECK_FOR_STACK_OVERFLOW 2
```

`vApplicationStackOverflowHook()` 觸發 fault、關中斷並停住。診斷 message 有：

- `control_stack_high_watermark`
- `comm_stack_high_watermark`
- `free_heap_bytes`
- `minimum_ever_free_heap_bytes`

High-water mark 通常代表歷史最少剩餘 stack 空間，單位依 API/port 可能是 words 而非 bytes；顯示或設 threshold 前要確認所用 API。

FreeRTOS heap 為 57344 bytes，使用 `heap_4`。`heap_4` 可合併相鄰 free block，但即時系統仍應避免 steady-state 中反覆動態配置。micro-ROS message buffer 優先固定容量。

## Interrupt priority 規則

Cortex-M 數字越小，中斷 priority 越高。設定：

```c
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5
configLIBRARY_LOWEST_INTERRUPT_PRIORITY = 15
```

會呼叫 FreeRTOS FromISR API 的中斷，其 NVIC priority 必須在 FreeRTOS 允許範圍；priority 太高（數字太小）卻呼叫 kernel API，可能造成難以重現的 queue/scheduler corruption。這是 STM32 + FreeRTOS 常見大坑。

## WCET、jitter、deadline

- **execution time**：一次 control 計算花多久。
- **WCET**：觀測/分析到的最壞執行時間。
- **period**：兩次 release 間隔，本案 1 ms。
- **deadline**：工作必須完成的期限，通常不晚於下一 period。
- **jitter**：實際開始/完成時間偏離理想週期的變化。

平均 0.2 ms 不代表安全；若偶爾 1.4 ms，仍會 miss 1 ms deadline。應用 DWT cycle counter/GPIO toggle/trace 量測，而不是用 ROS 50 Hz 到達率推論。

## 本章練習

1. 解釋為何 CommTask stack 比 ControlTask 大，但 priority 較低。
2. 把 `vTaskDelayUntil` 改成 `vTaskDelay` 會造成什麼長期效果？先回答，不要直接改。
3. 哪些資料適合 mailbox overwrite，哪些不適合？
4. `volatile bool ready` 為何不能完整解決兩 task 的資料競爭？

