# CubeMX 配置與韌體架構規範指南

> [!IMPORTANT]
> 現行 HIL 通訊基線為 USART2 460800 8N1；115200 僅保留為低流量診斷選項。
> micro-ROS RX 現採 RXNE IRQ ring，硬體輸出 gate 仍固定為 0。

**文件位置**：`docs/firmware_audit/02_cubemx_and_architecture_guidelines.md`  
**最後更新**：2026-07-30  

---

## 1. STM32CubeMX 硬體與周邊配置指南

### 1.1 時脈系統
- **HCLK / SYSCLK**: 84 MHz
- **APB1 Peripheral Clock**: 42 MHz (TIM6 / TIM7 脈衝頻率為 84 MHz)
- **APB2 Peripheral Clock**: 84 MHz

### 1.2 定時器分工
- **TIM7**: 專用為 STM32 HAL Library 之 Timebase Source (避免 `HAL_Delay()` 佔用與控制衝突)。
- **TIM6**: 專用為 `ControlTask` 之 1kHz (1ms) 硬體即時觸發源。
  - Period/Prescaler 設定：84MHz / 84 = 1MHz Count Frequency，Period = 1000 - 1 (1kHz 中斷)。
  - NVIC Priority: 5 (可安全呼叫 FreeRTOS `vTaskNotifyGiveFromISR`)。

### 1.3 串列埠與 DMA 設定
- **USART2**: 460800 Baud, 8 Bits Data, 1 Stop Bit, No Parity, No Hardware Flow Control。
  115200 只適合降載診斷，不足以承載目前兩條 50 Hz telemetry。
- **DMA1 Stream5 Channel4 (USART2_RX)**:
  - Mode: **Circular**
  - Priority: Very High
  - Data Width: Byte / Byte
- **DMA1 Stream6 Channel4 (USART2_TX)**:
  - Mode: Normal
  - Priority: Very High
  - Data Width: Byte / Byte
- **NVIC Priority**:
  - USART2 global interrupt: 6
  - DMA1 Stream5 / Stream6 interrupts: 6 (需低於 FreeRTOS Max Syscall Boundary Priority 5)

---

## 2. 韌體設計架構規範

1. **Safety Gate 優先**：任何驅動 API 呼叫前，必須完成浮點數有限性、軟體限幅與 Safety State 檢查。
2. **ISR 簡潔原則**：中斷服務常式中嚴禁進行複雜計算、鎖等待或多餘 LOG 輸出；僅執行極簡 API 並喚醒 RTOS Task。
3. **記憶體管理規範**：
   - 通訊層對應配有 `*_initialized` 生命週期旗標。
   - 釋放資源時採用反向順序清理。
4. **零警告原則 (Zero-Warning Standard)**：所有 C/C++ 源碼必須在 `-Wall -Wextra` 下達成 0 Compile Warnings。
