# Motor Bringup 經驗總結與 Conditional GO 放行條款

> [!WARNING]
> 條件式 GO 不是目前的執行授權。現行只完成輸出關閉 HIL Phase 0–2；
> 任一 gate=1 映像、馬達通電或機構動作都必須另行審核與授權。

**文件位置**：`docs/firmware_bringup_audit/02_bringup_lessons_learned_and_go_release.md`  
**最後更新**：2026-07-29  

---

## 1. Bringup 核心經驗與架構教訓 (Lessons Learned)

### 1.1 避免 Safety Gate 死鎖 (Deadlock Prevention in Safety Gates)
- **核心教訓**：在主迴圈或控制任務中，一旦發生超時 (Timeout) 或通訊中斷，觸發 `RobotSafety_Stop()` 後必須**立刻 `return` 離開迴圈**。嚴禁使用 `while(1)` 或 `continue` 自旋等待，否則會導致無法消耗 Queue 裡的逾時指令，並奪走 FreeRTOS 的任務調度權。

### 1.2 RTOS 時基與舊指令銷毀 (Timebase & Stale Command Annihilation)
- **核心教訓**：全面改用 RTOS 提供的無號時間 API (`xTaskGetTickCount()`) 計算時間差。判定逾時（如 250ms 無更新）時，必須**立刻用 `memset` 清空佇列快取的命令並重置 `has_command` 標誌**，防止舊指令於系統重置或時間 wrap-around (49.7天) 後被誤復活。

### 1.3 嚴格的單軸物理隔離 (Strict Actuator Isolation)
- **核心教訓**：測試模式下未選定的軸，不能只將轉速設為 0 RPM。必須透過 `enable_mask` 控制，實體呼叫 `Motor_Stop()` 或將 Enable PIN 拉低進行物理隔離，防止 PWM 雜訊或演算法突波驅動未測試軸。

### 1.4 命令語意分離，不共用欄位 (Separation of Command Semantics)
- **核心教訓**：測試模式（Test Mode）與一般運轉模式的參數必須具備明確且獨立的欄位 (如 `test_axis` 與 `test_rpm`)，嚴禁拿 `target_theta1_deg` 來帶轉速。`ApplyCommand()` 必須根據 `test_mode` 分支，防止正常關節角度限幅器誤阻擋測試指令。

### 1.5 終極安全閘門 (The Ultimate Safety Gate Pattern)
- **核心教訓**：致動器 API (`Motor_SetSpeed`) 呼叫的前一列，必須由 Ultimate Gate 進行三道硬性檢查：
  1. `std::isfinite` 浮點數有限性檢查。
  2. 絕對硬體限幅 (如 Bringup 階段上限 150 RPM)。
  3. 全域安全狀態 `RobotSafety_CanDrive()`。

---

## 2. Conditional GO 條款與通電防呆規則

1. **單次命令脈衝驗證**：首次測試使用 `Mock_TestCommand_Producer(axis, 10)` 發送短脈衝，確認馬達旋轉方向、Encoder 回授與電源電流符合預期。
2. **通電瞬間異常即刻切斷**：
   - 尚未送出 Test Command 馬達即自動旋轉。
   - 未選軸出現 Enable 訊號或 PWM Pulse。
   - 電流迅速升至電源限流上限或馬達堵轉發出高頻異音。
   - 命令停止更新 250ms 後，PWM / Enable 訊號未自動關閉。
