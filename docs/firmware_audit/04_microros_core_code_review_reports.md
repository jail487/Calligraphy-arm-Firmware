# micro-ROS 核心韌體 Code Review 歷程與最終檢驗報告

> [!IMPORTANT]
> T15–T27 後已重建 Humble static library、完成 RXNE IRQ ring、460800 baud、
> 完整 entities 與 session-control lifecycle。本文保留早期 review finding；最新結果
> 見 `docs/project_management/05_motion_control_hil_learning_report.md`。

**最後更新**：2026-07-30  
**目標平台**：STM32F446RE / FreeRTOS / micro-ROS / USART2 DMA  
**最終判定**：**Pass (Conditional)** — 所有第一輪列出之重大安全隱患與架構缺陷（AR-01 ~ AR-06）已全部實作修正並複審通過。

---

## 2026-08-04 remediation addendum

### Second-round review follow-up

The second-round findings were rechecked against the active source paths. The
follow-up remediation adds a driver-level physical-output guard to
`Motor_Update`, a separate guarded homing permission, logical/applied/physical
enable-mask provenance, boot-tagged session IDs with firmware high-water
validation, state-sequence/uptime replay rejection, active FreeRTOS runtime
sampling and stack-watermark reporting, Best-Effort McuState subscription QoS,
fresh measured-state gating and clock-based trajectory preemption, resolved YAML
limits, and expanded HIL provenance CSV fields plus telemetry silence gating.

The source and Python checks below are not a substitute for ROS 2 Humble
container execution, native firmware host execution, or powered hardware
validation. `ACTUATOR_HARDWARE_OUTPUT_ENABLED` remains `0`, and Phase 3--7 are
still not marked complete.

This addendum records the source-level remediation for the motion-control code-review findings. It does not advance any powered-test phase.

| Finding | Remediation | Evidence added |
|---|---|---|
| Motor Test `enable=false` | The micro-ROS callback treats disable as an idempotent safety stop, clears legacy `test_mode` state, invalidates the mailbox, and never falls through to a non-zero enable mask. | `Application/Inc/test_scenarios.h`, `Application/Tests/test_robot_interface.c` |
| Shadow execution | `SAFE` is now a valid execution state when hardware output is compiled off. Validation, PVA interpolation, PID, and telemetry snapshots continue; `RobotSafety_CanDrive()` remains separate and the final actuator gate remains closed. | `Application/Src/mainpp.cpp`, `Application/Src/safety_manager.cpp` |
| EF/joint trajectory | Motion node samples a persistent trajectory at 100 Hz. Firmware position target drives interpolation even when target velocity and acceleration are zero; generated applied state is not replaced by the endpoint on receipt. | `doghead_motion/motion_node.py`, `trajectory_generator.py`, `Application/Src/trajectory_planner.cpp` |
| McuState field provenance | Boot epoch uses an RTC backup counter, command age uses MCU receive tick, encoder telemetry uses cumulative pulse count, commanded RPM is separated from measured RPM, and applied PVA comes from the interpolator state. | `Middleware/MicroROS/Src/microros_app.c`, `Application/Src/mainpp.cpp`, `Application/Inc/robot_interface.h` |
| Reconnect/reboot fail-safe | Telemetry silence disarms; boot changes and reconnect-counter changes invalidate the active session; old session replay is rejected until a new Arm/session handshake. | `doghead_driver/safety_validator.py`, `driver_node.py`, firmware session callbacks |
| HIL report gates | Rate tolerance, maximum timing deviation, dropped frames, and jitter threshold now contribute to pass/fail. Timing standard deviation, maximum period, and maximum deviation are reported separately; `tracking_report` exports/reports deterministic CSV data. | `doghead_hil/tracking_metrics.py`, `tracking_report.py`, `telemetry_validator.py` |

The `McuState` contract was extended with `physical_enable_mask`; both source
copies are synchronized. The default `ACTUATOR_HARDWARE_OUTPUT_ENABLED` remains
`0`; no powered or physical motor test was performed.

Local evidence from this review: STM32 compiler syntax checks passed for the modified firmware sources and test sources; Python pure-logic tests passed. Full firmware host execution, ROS 2 Humble `colcon build`, ROS 2 Humble `colcon test`, and any Phase 3–7 hardware evidence remain pending because the current environment has no native host compiler, ROS 2 Humble runtime, or available powered test authorization.

## 1. Code Review 修正歷程對照表

| ID | 類別 | 第一輪審查判定 | 第二輪複審結果 | 修正技術細節 |
|---|---|---|---|---|
| **AR-01** | State Machine | Action Required | **【Pass】** | 移除會導致初始化無條件失敗的 0-handle executor，狀態機成功轉入 `RUNNING` 態。 |
| **AR-02** | DMA/IRQ Safety | Action Required | **【Pass】** | 於 `cubemx_transport_open()` Critical Section 中停用 `UART_IT_ERR/PE`，防止 HAL 中斷處理在 ORE/NE/FE 時 abort RX DMA。 |
| **AR-03** | RTOS Timing | Action Required | **【Pass】** | TX 增加 `UART_TX_TIMEOUT_MS` (100ms) 的 RTOS tick 逾時，超時呼叫 `HAL_UART_AbortTransmit()`，不破壞 RX Stream。 |
| **AR-04** | Task Priority | Action Required | **【Pass】** | 修正 raw priority 與 CMSIS priority 混用錯位問題。`ControlTask` (48) > `CommTask` (32) > `defaultTask` (24)。 |
| **AR-05** | Entity Lifecycle | Action Required | **【Pass】** | 實作 `*_initialized` 生命週期旗標與 `MicroROS_DestroyEntities()`，Agent 離線時設置 destroy timeout 為 0。 |
| **AR-06** | RTOS Init | Action Required | **【Pass】** | 檢查 `xTaskCreate` 是否回傳 `pdPASS`，失敗時立刻觸發 `Error_Handler()` 防禦。 |
| **W-01** | RTOS Timing | Warning | **【Pass】** | `vTaskDelayUntil` 補上 overrun check，防止系統啟動後的 catch-up 快速迴圈。 |
| **W-02** | DMA Buffer | Warning | **【Warning】** | Ring buffer 尚未實作計數溢位追蹤，需依賴 1000ms Ping 或實機 Soak Test 觀察。 |
| **W-03** | IRQ Safety | Warning | **【Pass】** | 移除 `dma_transport.c` 不當的 `__disable_irq()` 呼叫，改用無副作用的暫存器直接讀取。 |
| **W-04** | API Handling | Warning | **【Pass】** | 修復 `open()` / `close()` 忽略 HAL 狀態、`err` 未維護與型別縮窄隱患。 |
| **W-05** | State Machine | Warning | **【Pass】** | 於 `RUNNING` 狀態實作 1000ms 週期 `rmw_uros_ping_agent()` 主動離線偵測。 |
| **W-06** | Memory Leak | Warning | **【Warning】** | 仍使用 Newlib Default Allocator，需透過 1,000 次 Agent 重連 Soak Test 驗證堆疊穩定度。 |

---

## 2. 實機 Fault-Injection 測試清單 (驗收規範)

為將本版程式碼標記為 `Production-Ready`，請在開發板上執行下列 5 項測試並觀察預期結果：

1. **ORE/NE/FE 注入測試 (AR-02 驗證)**
   * **步驟**：在與 Agent 連線狀態下，刻意在 RX 腳位注入突波，或使用終端機發送錯誤 Baudrate 的封包。
   * **預期**：韌體不當機、`CommTask` 不卡死，UART 硬體之 `DMAR` 旗標與 DMA Stream5 的 `EN` 旗標應維持在 1。
2. **TX IRQ Timeout 測試 (AR-03 驗證)**
   * **步驟**：在 STM32CubeIDE 暫停執行並修改 DMA1 Stream6 的暫存器（例如手動清除 EN 旗標，模擬硬體卡死），然後繼續執行。
   * **預期**：`CommTask` 應在 100ms 後自行跳出 `cubemx_transport_write`，並透過 `HAL_UART_AbortTransmit` 結束 TX，狀態機退回 `WAIT_AGENT`。
3. **Agent Reconnect 壓力測試 (AR-05 & W-06 驗證)**
   * **步驟**：寫一個 Python script 或 Shell script，每隔 5 秒開關一次 `micro-ros-agent` 容器，反覆 1000 次。
   * **預期**：MCU 能成功斷線並自動重連 1000 次而不發生 HardFault。請同步監控 `_sbrk` 邊界或自訂 heap 指標，確保無 leak。
4. **Ring Overflow 盲測 (W-02 驗證)**
   * **步驟**：限制 `CommTask` 執行頻率 (例如將 delay 暫改為 500ms)，並從 PC 端連續發送大量大封包。
   * **預期**：系統可能無法解析該封包，但應能透過 1000ms Ping 發現連線異常而重啟 Session，不可發生指標越界或記憶體崩潰。
5. **System Core Deadline 測試 (AR-04 驗證)**
   * **步驟**：正常運作中，量測 `ControlTask` 內 `Robot_ControlTask_Tick` 的週期。
   * **預期**：即使 `CommTask` 正忙於 micro-ROS 的大量 TX/RX，`ControlTask` 仍應精準保持在 1ms 週期，且不觸發 `ROBOT_FAULT_DEADLINE_MISS`。

## 2026-08-05 second-round repair verification update

The remediation addendum was extended after independent review. In addition
to the safety, shadow-execution, trajectory, telemetry, reconnect, and HIL
changes already listed above, the RPM contract is now explicit end-to-end:
Motor Test `target_motor_rpm` remains wire-compatible but means
joint/output-shaft RPM, `Motor_SetSpeed()` consumes joint RPM, and the reducer
is applied once by the driver. Regressions cover the old +/-100 cap and a 114:1
Motor Test/PVA conversion.

Observed local evidence:

- driver pure-Python tests: **18 passed, 5 deselected**;
- motion targeted tests: **3 passed, 1 skipped**;
- HIL targeted tests: **17 passed, 2 skipped**;
- Python compilation: **passed**;
- both `McuState.msg` and `McuMotorTestCommand.msg` source copies:
  **synchronized**.

The following remain unverified: native Firmware host execution because
`g++`/`gcc` are unavailable; ROS-dependent tests because `rclpy`/`numpy` are
unavailable; ROS 2 Humble `colcon build`/`colcon test` because `colcon`/`ros2`
and a usable Docker Humble daemon are unavailable; and all powered/physical
motor evidence. `ACTUATOR_HARDWARE_OUTPUT_ENABLED` remains `0`, no powered
test was performed, and Phases 3--7 are still **pending**.
