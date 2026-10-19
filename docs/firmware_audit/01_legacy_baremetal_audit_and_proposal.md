# 裸機舊版程式碼審查與 RTOS 重構提案歷史紀錄

> [!IMPORTANT]
> 這是歷史稽核文件。現行系統已完成 ROS 2 Humble 輸出關閉 HIL
> Phase 0–2，UART 基線為 460800 8N1，並採 fail-closed session binding。
> 最新狀態與教學請讀 `docs/project_management/05_motion_control_hil_learning_report.md`。

**歷史時間**：2026-07-29  
**目的**：記錄由舊版 Bare-Metal 專案遷移至 FreeRTOS + micro-ROS 雙任務架構前的程式碼健康度審查、漏洞診斷與改善提案。

---

## 1. 建置與環境配置審查

### 1.1 歷史問題
- **編譯器與工具鏈**：STM32CubeIDE 預設選用 ARM GNU C/C++ 工具鏈，舊版本專案中存在若干裸機全域變數未進行鎖定（Lock）保護的問題。
- **時脈樹 (Clock Tree)**：早期設定於 SYSCLK 90MHz，後續配合 STM32F446RE HSI/PLL 標準修正為 **SYSCLK 84MHz**。

---

## 2. 程式碼健康度診斷

### 2.1 歷史缺陷發現
1. **中斷與時基爭用**：原先 `HAL_Delay()` 與 `g_current_time_ms` 變數在中斷中自增，在多工作業系統中缺乏原子性（Atomicity），且易在 32-bit 49.7 天溢位後產生不可預期的超時判定。
2. **安全閘門缺失**：致動器控制 API 直通，未在最終輸出列檢查 `std::isfinite`，遇到演算 NaN / Inf 時會將極端數值直接寫入 PWM / 轉速暫存器。
3. **無窮自旋與死鎖**：控制迴圈發生錯誤時使用 `while(1)` 或 `continue`，造成 CPU 被死死佔據，FreeRTOS 排程器無法切換任務，外部清錯指令無法寫入 Queue。

---

## 3. micro-ROS 架構提案

為解決裸機通訊受限問題，提出基於 **FreeRTOS + micro-ROS** 的雙任務與自訂 DMA 傳輸架構：

1. **控制任務 (ControlTask)**：以 1kHz 高優先級定時運行，專心處理 PWM 驅動、Encoder 讀取、PID 運算與 Safety Gate 驗證。
2. **通訊任務 (CommTask)**：以 50Hz 優先級運行，負責 micro-ROS 自動重連狀態機與 XRCE-DDS 訊息處理。
3. **自訂 DMA Transport**：使用 USART2 + DMA (Circular RX) 實現二進位傳輸，避免阻塞 CPU。

---

**備註**：本文件內容已於 Phase 1 ~ Phase 3 期間完整實作並通過驗收。
