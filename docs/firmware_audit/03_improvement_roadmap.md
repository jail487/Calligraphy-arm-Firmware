# 分階段改善路線與進度規劃 (Roadmap)

> [!IMPORTANT]
> 本文件的 Phase 0–3 是早期「韌體重構階段」編號，不等於現行 motion HIL
> Phase 編號。現行運動控制 HIL Phase 0–2 已通過；馬達帶電的 Phase 3 尚未授權。
> Canonical 狀態見 `docs/project_management/05_motion_control_hil_learning_report.md`。

**最後更新**：2026-07-31（早期 firmware Phase 0–3 完成；現行 motion HIL Phase 0–2 通過）  
**總體原則**：每一 Phase 單獨 commit、先測試再擴展；任何馬達驅動測試均須通過 Ultimate Safety Gate，並具備手動切斷動力的 E-Stop 實體開關。

---

## 2026-08-04 review remediation status

### Second-round follow-up status

The follow-up source pass also closes the remaining review-level gaps around
the active actuator path, homing permission, enable-mask provenance, PWM unit
conversion, MCU-state replay freshness, boot-bound session binding, active
FreeRTOS execution metrics, Best-Effort telemetry QoS, measured-state motion
startup/preemption, resolved YAML safety limits, and HIL provenance/silence
reporting. These are source-level and software-test changes only.

They do not constitute physical evidence. Phase 3, Phase 4, Phase 5, Phase 6,
and Phase 7 remain **PENDING / NOT COMPLETE** until the project plan's required
ROS 2 Humble/container, HIL, safety-authorized, and hardware evidence exists.

The latest software remediation closes the identified command, shadow-execution, trajectory, telemetry-provenance, reconnect, and HIL-reporting defects at source level. It adds regression tests for Motor Test disable, shadow applied state, zero-velocity/acceleration position convergence, command receive age, cumulative encoder count, boot/reconnect invalidation, stale-session replay, telemetry rate/jitter thresholds, and dropped frames.

This status is intentionally not a Phase 3–7 completion claim. `ACTUATOR_HARDWARE_OUTPUT_ENABLED` remains `0`, and no powered motor test was run. Phase 3–7 still require the project plan's independent ROS 2 Humble/container evidence, hardware-in-the-loop evidence, safety authorization, and (where applicable) controlled physical validation.

### Effective phase-gate status

For acceptance purposes, Phase 3, Phase 4, Phase 5, Phase 6, and Phase 7 are **PENDING / NOT COMPLETE**. Any older status text elsewhere in this legacy roadmap is historical context and is superseded by the evidence rule above.

## 📅 各階段進度總覽

| Phase | 名稱 | 狀態 | 核心成果 / 下一步目標 |
|---|---|---|---|
| **Phase 0** | Baseline 保存與建置對齊 | **已完成 ✅** | 建立可安全回退的基線標籤，修復編譯與版本控制範圍 |
| **Phase 1** | 阻礙安全與硬體 Bug 修復 | **已完成 ✅** | C-01/C-03 硬體引腳映射與 Arming 機制對齊，TIM6 1kHz 定時與缺勤紀錄完成 |
| **Phase 2** | 介面解耦與安全合約分離 | **已完成 ✅** | `RobotCommand_t` 與 `RobotTelemetry_t` 雙緩衝與語意分離，逾時 250ms 自動清空 |
| **Phase 3** | micro-ROS 核心與連線狀態機 | **已完成 ✅** | 解決 FPU ABI 浮點崩潰、DMA RX 中斷中斷保護、0-handle 轉入 RUNNING、Pass 第二輪審查 |
| **Phase 4** | ROS telemetry / command shadow 與後續實體控制 | **軟體與 gate=0 HIL 完成；powered test 待授權** | `/joint_states`、McuState/Diagnostics、versioned command 與 session binding 已驗證；下一步是有前置條件的單軸測試 |
| **Phase 5** | ROS 2 Planner & 軌跡整合 | **規劃中 📅** | 與 ROS 2 MoveIt / Controller 整合，Rviz 視覺化雙向驗證 |
| **Phase 6** | 長時間穩定度與 Fault-Injection 測試 | **規劃中 📅** | 進行 72 小時連續運轉測試與 1000 次 Agent 重連、爆量封包及斷電壓力測試 |

---

## 🔍 各階段詳細說明與完成細節

### Phase 0：保留目前可建置版本 (已完成 ✅)

- **目標**：可回退 baseline，不改功能。
- **成果**：完成建置環境確認、去除編譯警告，確保 0 Error / 0 Warning 可重複 Clean Build。

### Phase 1：修正阻礙安全與明顯 bug (已完成 ✅)

- **目標**：在沒有 ROS 下讓 motor mapping、boot/homing 與 1 kHz 時基可信。
- **成果**：
  - PC0/PC4 介面與 Arming 邏輯修復。
  - TIM6 1kHz 優先權修正 (CMSIS `osPriorityRealtime`)，並加入 Task Notification 與 miss 紀錄。
  - TIM7 作為系統 HAL Timebase，避免定時器搶佔。

### Phase 2：整理通訊與控制介面 (已完成 ✅)

- **目標**：ROS-independent command/telemetry contract 與安全 timeout。
- **成果**：
  - `RobotCommand_t` 實作雙緩衝與測試/運轉模式參數分離。
  - 超過 250ms 無更新時自動 `memset` 歸零並重置標誌，防止舊指令復活。
  - 實作 Ultimate Safety Gate Pattern，強制 `std::isfinite` 與 `RobotSafety_CanDrive()` 檢查。

### Phase 3：最小 micro-ROS 通訊與狀態機 (已完成 ✅)

- **目標**：真實 agent 連線、連線狀態機、DMA Transport 與中斷防禦。
- **成果**：
  - 統一 ABI 參數為 `-mfloat-abi=softfp -mfpu=fpv4-sp-d16`。
  - `dma_transport.c` 實作 critical section 關閉 `UART_IT_ERR/PE` 中斷，防止硬體雜訊導致 HAL 取消 RX DMA。
  - 移除 0-handle executor，成功於 `microros_app.c` 中轉入 `MICROROS_STATE_RUNNING`。
  - 實作 100ms TX Timeout (`HAL_UART_AbortTransmit`) 隔離發送死鎖。
  - 實作 1000ms 週期 `rmw_uros_ping_agent` 與無阻斷資源釋放。
  - 通過第二輪嚴格韌體 Code Review (`04_microros_core_code_review_reports.md`)。

---

## 🎯 下一個受控階段：單軸馬達實體控制

- **已完成基線**：`RobotTelemetry_t`、McuState、Diagnostics、`/joint_states`、
  command shadow、session/sequence/TTL 與 Agent reconnect invalidation 已完成。
- **下一個目標**：取得硬體規格與獨立授權後，以 gate=1 的專用映像進行單軸、
  無負載、低速啟動。
- **具體任務**：
  1. **Topic 擴充**：
     - 已完成 `/joint_states` 與 `/doghead/mcu/state` 50 Hz Best-Effort telemetry。
     - 正式 command 使用固定長度 `McuMotorTestCommand` / `McuJointSetpoint`，不使用
       無版本與語意不明的 `Float32MultiArray`。
  2. **安全整合**：
     - 確保連線中斷或 Agent 離線時，狀態機自動觸發 `DESTROY_ENTITIES` 並將馬達驅動強制切回 Safe Stop (EN PIN 拉低)。
     - 驗證連線恢復後，舊指令不會自動復活，需由上層重新 Arming。
  3. **實機無負載 Bench Test**：
     - 依照 `Conditional GO` 規範，於單軸低速 (10~50 RPM) 下進行無負載空轉驗證。

---

## 🔮 未來展望：Phase 5 & Phase 6

- **Phase 5 (軌跡與高階整合)**：
  - 開發 ROS 2 端 Control Node，對接 MoveIt / ros2_control。
  - 於 Rviz 中即時呈現三軸手臂關節姿態。
- **Phase 6 (極限測試與 Production-Ready)**：
  - 執行 1,000 次 Agent 斷線重連抗壓測試（驗證 Newlib Memory Leak）。
  - 執行 ORE/NE/FE 突波注入測試與 Ring Buffer Overflow 壓力測試。
  - 完成 72 小時不間斷 Soak Test。
