# DogHead 書法機械手臂 - 專案完整技術文檔

**版本**: v3.2 (FreeRTOS + micro-ROS 架構版)  
**最後更新**: 2026-01-07  
**微控制器**: STM32F446RET6  
**開發板**: NUCLEO-F446RE  
**作業系統**: FreeRTOS  
**中間件**: micro-ROS (Humble)

---

# 1. 專案狀態更新 (2026-01-07)

## 1.1 重大架構變更
本專案已從裸機 (Bare-Metal) 架構遷移至 **FreeRTOS + micro-ROS** 架構，以支援更複雜的通訊需求與即時控制任務。最新版本 (v3.2) 引入了硬體中斷觸發機制，確保控制迴圈的嚴格即時性。

### 新舊架構對比
| 特性 | 舊架構 (v2.0) | 新架構 (v3.2) |
|------|--------------|--------------|
| **核心排程** | `while(1)` 主迴圈 + Timer 中斷 | FreeRTOS 搶佔式多工 + 硬體中斷觸發 |
| **通訊協定** | 簡單 UART 字串 | micro-ROS (DDS over UART) |
| **控制頻率** | 依賴迴圈速度 (不穩定) | TIM6 觸發 `ControlTask` (嚴格 1kHz) |
| **關節控制** | 簡易 P 控制 | PID + 速度/加速度前饋 |

## 1.2 FreeRTOS 任務規劃
應用程式邏輯被劃分為兩個主要任務，並由 CubeMX 自動生成的 `defaultTask` 處理底層初始化：

| 任務名稱 | 優先級 | 頻率 | 堆疊大小 | 職責 |
|---------|--------|------|----------|------|
| **ControlTask** | `osPriorityAboveNormal` (3) | 1000 Hz (1ms) | 256 Words | 負責馬達直接驅動、PID 計算、軌跡跟隨。由 TIM6 中斷發送 Task Notification 觸發，確保微秒級準確度。 |
| **CommTask** | `osPriorityNormal` (2) | 50 Hz (20ms) | 256 Words | 負責 micro-ROS 通訊：執行 Executor (處理訂閱) 與發布 `/joint_states`；同時負責系統狀態監控。 |
| **defaultTask** | `osPriorityLow` (1) | - | 128 Words | 系統閒置任務 (未積極使用)。 |

## 1.3 micro-ROS 整合狀態
- **Library**: `libmicroros.a` (Humble) 針對 Cortex-M4 靜態編譯。
- **Transport**: 基於 DMA/Interrupt 的 UART 傳輸層。
- **應用層 (`microros_app.c`)**:
    - **Node**: `doghead_arm_node`
    - **Publisher**: `/joint_states` (sensor_msgs/JointState) @ 50Hz
    - **Subscriber**: `/motor_cmd` (std_msgs/Float32MultiArray)
    - **Executor**: 負責訂閱回調與服務請求的處理

---

# 2. STM32CubeMX (IOC) 硬體配置詳情

## 2.1 系統時鐘與基本配置 (RCC/SYS)
- **High Speed Clock (HSE)**: Bypass Clock Source (或 Crystal)
- **SYSCLK**: **90 MHz**
    - PLL Source: HSI/HSE
    - APB1 Prescaler: /2 (45MHz PCLK, 90MHz Timer)
    - APB2 Prescaler: /1 (90MHz)
- **⚠️ Timebase Source (關鍵設定)**: 改為 **TIM7** (或 TIM14)
    - **原因**: FreeRTOS 會佔用 SysTick，導致 HAL 庫需要另一個 Timer。CubeMX 預設會搶佔 **TIM6**。
    - **操作**: 到 `System Core` > `SYS` 將 `Timebase Source` 改掉，以釋放 **TIM6** 給控制迴圈使用。

ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc

## 2.3 通訊接口 (Connectivity)

### USART2 (micro-ROS)
- **Mode**: Asynchronous
- **Baud Rate**: 115200 Bits/s
- **Word Length**: 8 Bits
- **Parity**: None
- **Stop Bits**: 1
- **Pins**: PA2 (TX), PA3 (RX)
- **NVIC**: USART2 global interrupt ENABLED (用於 micro-ROS 傳輸層)
- **DMA**: 建議啟用 DMA RX/TX 提升效率 (目前使用 IT 模式)

## 2.4 其他 GPIO (根據 Firmware/nidec_motor_driver.c 配置)
### 馬達 1 (13-Pin, Freq Control)
- **PWM (Freq)**: PA5 (TIM2_CH1) / 100-6000Hz (ARR動態調整)
- **Encoder**: PA8, PA9 (TIM1) / 100 PPR * 4
- **Start/Stop**: PC1 (Output)
- **Direction**: PC1 (Output)

### 馬達 2 (13-Pin, Freq Control)
- **PWM (Freq)**: PB4 (TIM3_CH1)
- **Encoder**: PB6, PB7 (TIM4)
- **Start/Stop**: PC0 (Output) Note: 需確認 GPIO Init
- **Direction**: PC2 (Output) Note: 需確認 GPIO Init

### 馬達 3 (13-Pin, Freq Control)
- **PWM (Freq)**: PB14 (TIM12_CH1) 
- **Encoder**: PA0, PA1 (TIM5)
- **Brake**: PC4 (Output, Low=Brake)
- **Direction**: PC3 (Output)


### 其他
- **PC13**: B1 [Blue PushButton] (Input)

---

# 3. 軟體實作細節

## 3.1 控制任務 (ControlTask)
位於 `freertos.c`，透過 `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` 實現由 TIM6 中斷觸發的精確同步。
- **運作頻率**: 1000 Hz
- **核心職責**:
    1. **Motor_Update**: 讀取並計算即時轉速/位置
    2. **Mode Handler**: 根據 `test_mode_selection` 切換 PID 或測試模式
    3. **Robot_Loop**: 執行運動學與閉環控制 (若在 PID 模式)
    4. **Motor_SetSpeed**: 輸出最終指令

## 3.2 micro-ROS 應用層 (`microros_app.c`)
負責所有 ROS2 通訊邏輯，與 RTOS 低耦合。
- **Publisher**: 
    - Topic: `/joint_states`
    - Msg Type: `sensor_msgs/msg/JointState`
    - Data: 轉換後的弧度 (Rad) 與角速度 (Rad/s)
- **Subscriber**: 
    - Topic: `/motor_cmd`
    - Msg Type: `std_msgs/msg/Float32MultiArray`
    - Callback: 接收 [RPM1, RPM2] 並更新全局目標變數

## 3.3 中斷服務常式 (`stm32f4xx_it.c`)
TIM6 中斷是整個系統的心跳，負責維持時序精確度，避免 RTOS 軟體計時器的抖動 (Jitter)。
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        // 發送通知給 ControlTask，解除其阻塞
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(xControlTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```

---

# 4. PID 前饋控制架構詳解

## 4.1 控制器類別 (PositionController)
**檔案位置**: `Core/Inc/robot_arm_core.hpp` (或 .cpp 內實作)

### 參數說明
| 參數 | 說明 | 調整建議 |
|------|------|----------|
| `kp` | 比例增益 | 基礎響應力道 |
| `ki` | 積分增益 | 消除穩態誤差 |
| `kd` | 微分增益 | 抑制震盪 |
| `kv` | 速度前饋 | **關鍵**: 消除動態滯後 (Lag) |
| `ka` | 加速度前饋 | 補償慣性 (Inertia) |

### 控制算法
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

# 5. 開發環境與編譯

## 5.1 必要依賴
- **IDE**: STM32CubeIDE 1.14+
- **Toolchain**: GNU Tools for STM32
- **micro-ROS Agent**: Docker Image (`microros/micro-ros-agent:humble`)

## 5.2 編譯設定
1. **Include Paths**: 
   - `Core/Inc`
   - `Middlewares/Third_Party/FreeRTOS/...`
   - `micro_ros_stm32/library/includes`
2. **Library Paths**:
   - `micro_ros_stm32/library`
3. **Libraries**:
   - `microros`

## 5.3 部署步驟
1. 燒錄靭體 (`.elf` / `.bin`)
2. 啟動 Agent: `./scripts/start_microros_agent.sh`
3. RESET 開發板
4. 驗證: `ros2 topic list`

---

**維護者**: Jail487 & GitHub Copilot  

