# DogHead 書法機械手臂 - 專案 Master 技術文檔

> [!IMPORTANT]
> 2026-07-31 現況：ROS 2 Humble 輸出關閉 HIL Phase 0–2 已通過；USART2
> 基線為 460800 8N1；session 綁定、命令拒絕矩陣與 Agent 重連後失效
> 已實機驗證。馬達帶電的 Phase 3 尚未授權。

**版本**: v5.0（ROS 2 Humble 輸出關閉 HIL Phase 0–2 已驗證）  
**最後更新**: 2026-07-31  
**微控制器**: STM32F446RET6  
**開發板**: NUCLEO-F446RE  
**作業系統**: FreeRTOS V10.3.1 (CMSIS-RTOS V2 API)  
**中間件**: micro-ROS (Humble) / XRCE-DDS Custom DMA Transport

---

# 1. 專案狀態與架構概述

本專案為 DogHead 三軸書法機械手臂之工業級高即時性韌體。系統採用 **FreeRTOS 雙任務架構**，分別處理 1kHz 硬體即時控制與 50Hz micro-ROS 狀態機通訊。

## 1.1 核心里程碑（截至 2026-07-31）
- **早期韌體 Phase 0–3 完成；現行運動控制 HIL Phase 0–2 通過**：
  1. 微控制器 FPU ABI 統一對齊為 `-mfloat-abi=softfp -mfpu=fpv4-sp-d16`，排除開機 HardFault 崩潰。
  2. 客製化 USART2 Circular DMA Transport 於 Critical Section 中關閉 `UART_IT_ERR/PE` 中斷，防止硬體突波使 HAL 強行 abort RX DMA。
  3. 狀態機可完整建立 heartbeat、JointState、McuState、Diagnostics、session control
     與 command entities，並在斷線時反向銷毀及清除 session/mailbox。
  4. 通過第二輪嚴格核心韌體 Code Review (`docs/firmware_audit/04_microros_core_code_review_reports.md`)。
- **目前成果**：USART2 460800 8N1、約 49.2–49.3 Hz 遙測、5 個套件／
  31 個測試結果，session／sequence／TTL／版本與重連後 fail-closed 實機通過。
- **下一階段**：馬達帶電的 Phase 3，等待硬體資料、E-stop 條件與獨立授權。

---

# 2. 韌體工業級安全架構

在系統中，所有致動器控制邏輯均強制遵守以下安全防禦機制：

## 2.1 終極安全閘門模式
在最終呼叫硬體 API (`Motor_SetSpeed`, `Motor_Start` 等) 的前一列，必須通過終極閘門檢查：
- **數值有效性**: 強制使用 `std::isfinite` 檢查所有浮點數輸出，杜絕 NaN / Inf。
- **絕對硬體限幅**: 對 RPM 命令進行絕對硬降頻/限幅（如 Bringup 階段上限 150 RPM）。
- **安全狀態檢查**: 於呼叫硬體 API 的前一行，強制檢查 `RobotSafety_CanDrive()`。

## 2.2 RTOS 時基與舊指令銷毀 (Timebase & Stale Command Annihilation)
- **RTOS 原生計時**: 全面採用 `xTaskGetTickCount()` (型別為 `TickType_t`)，禁止使用簡單的中斷自增計時器，防範 32-bit tick wrap-around（49.7天溢位點）。
- **過期指令清空**: 當 Queue 中的指令判定逾時（如 250ms 無更新）時，必須立即呼叫 `memset` 將快取的 `RobotCommand_t` 清空為 0 並重置 `has_command` 標誌，防止舊指令於系統重置或時間 wrap-around 後被誤復活。

## 2.3 迴圈死鎖防範 (Loop Deadlock Prevention)
- **非阻塞退回**: 在 `Robot_Loop` 或控制任務中，一旦發生超時或 Fault，觸發安全停止後必須**積極 `return` 離開任務迴圈**。嚴禁使用 `while(1)` 或 `continue` 自旋等待，以確保 FreeRTOS 能進行任務切換並處理復歸指令。

## 2.4 致動器單軸硬體隔離 (Strict Actuator Isolation)
- **硬體 EN 腳位控制**: 在測試模式中，未被選定的軸不僅轉速設為 0，更必須透過 `Motor_Stop()` 或將 Enable PIN 拉低進行物理隔離，防止演算法突波或 PWM 雜訊驅動未測試軸。

## 2.5 指令語意分離 (Separation of Command Semantics)
- 測試模式（Test Mode）與一般運轉模式的參數欄位嚴格分離，杜絕同欄位跨模式複用。測試模式參數由控制層專屬驗證，防止正常 Joint Angle 限幅邏輯誤阻擋測試指令。

## 2.6 工具鏈獨立性 (Toolchain Independence)
- 避免使用特定 C++ 版本函數（如 `<algorithm>` 的 `std::clamp`），核心控制層退回最基礎 `if-else` 邏輯，確保在各式嵌入式 ARM Toolchain 下皆可達 **zero warning / clean build**。

---

# 3. 系統硬體配置與任務規劃

## 3.1 FreeRTOS 任務架構

| 任務名稱 | 優先級 | 頻率 | 堆疊大小 | 職責 |
|---------|--------|------|----------|------|
| **ControlTask** | `osPriorityRealtime` (48 / raw 6) | 1000 Hz (1ms) | 512 Words (2KB) | 負責馬達驅動、Safety Gate 驗證、PID 與軌跡更新。由 TIM6 硬體中斷觸發 Task Notification 喚醒。 |
| **CommTask** | `osPriorityAboveNormal` (32 / raw 5) | 50 Hz (20ms) | 1024 Words (4KB) | 執行 micro-ROS 狀態機 (`WAIT_AGENT` -> `CREATE_ENTITIES` -> `RUNNING` -> `DESTROY_ENTITIES`)，發布心跳包與處理遙測。 |
| **defaultTask** | `osPriorityNormal` (24) | - | 128 Words | 系統 CubeMX 生成之預設任務。 |

## 3.2 時鐘與 Timer 規劃
- **SYSCLK / HCLK**: 84 MHz
- **Timebase Source (HAL)**: **TIM7** (避免與控速定時器衝突)
- **ControlTask 觸發**: **TIM6** 中斷 (1kHz 精確即時性)

---

# 4. 通訊協定與資料結構 (`RobotCommand_t`)

```cpp
typedef struct {
    uint32_t timestamp_ms;
    int32_t test_mode;     // 0: Normal PID / IK Mode, 1: Single-Axis Test Mode
    uint8_t test_axis;     // Target Axis (0, 1, 2)
    int32_t test_rpm;      // Target Test RPM (e.g. +-100 RPM limit)

    float target_theta1_deg;
    float target_theta2_deg;
    float target_theta3_deg;

    float pose_x;
    float pose_y;
    float pose_z;
    bool use_ik_mode;
} RobotCommand_t;
```

---

# 5. PID 與運動控制架構

位置控制器整合前饋 (Feedforward) 補償以降低動態滯後：
```cpp
// 1. 誤差計算
error = target_pos - current_pos;

// 2. PID 反饋
feedback = Kp*error + Ki*integral + Kd*derivative;

// 3. 前饋補償 (Feedforward)
feedforward = (target_vel * Kv) + (target_acc * Ka);

// 4. 總輸出
output_rpm = feedback + feedforward;
```

---

**維護團隊**: Development & Safety Audit Team  
