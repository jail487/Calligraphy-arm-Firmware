# 技術設計理念、痛點剖析與 API/指令規格全書 (Technical Rationale & Specification)

> [!IMPORTANT]
> 現行基線已更新為 ROS 2 Humble container、USART2 460800 8N1、Windows VCP
> TCP bridge、50 Hz Best-Effort telemetry 與 Reliable `McuSessionControl`。

**專案**：Calligraphy-arm-Firmware (DogHead 三軸書法機械手臂)  
**文件位置**：`docs/project_management/02_technical_design_rationale_and_commands.md`  
**最後更新**：2026-07-30  
**目標**：詳盡說明本專案每一部分架構設計的「為什麼要這樣做 (Why)」、「解決了什麼痛點 (Problem)」、「技術意義 (Significance)」，以及所有核心 API、函式與系統指令的功能規格。

---

# 1. 專案目標與整體設計哲學

## 1.1 專案目標
本專案旨在為 **DogHead 三軸書法機械手臂** 打造一套高精度、高即時性、具備 ROS 2 生態系相容性，且符合**工業級安全防禦 (Industrial-Grade Safety)** 的嵌入式韌體。

## 1.2 核心痛點與設計哲學
1. **控制與通訊的矛盾**：馬達 PID 控制演算法需要極度穩定、無抖動的 1kHz (`dt=1ms`) 週期；而 micro-ROS 通訊涉及 XRCE-DDS 封包解析與串列傳輸，耗時具備不確定性。
   - 👉 **解法**：**FreeRTOS 雙任務完全解耦架構**。
2. **極端狀況下的暴衝風險**：微控制器若遇到奇異點 (Singularity)、浮點數 NaN/Inf、網路斷線或佇列舊指令復活，機械手臂極易發生全速衝撞。
   - 👉 **解法**：**Ultimate Safety Gate + 舊指令銷毀 (Stale Command Annihilation) + 單軸硬體隔離**。
3. **跨系統傳輸易卡死**：STM32 HAL 庫在 DMA RX 遇到雜訊時會自動 abort DMA，導致連線永久鎖死。
   - 👉 **解法**：**Custom DMA Transport + Critical Section 關閉硬體錯誤中斷**。

---

# 2. 架構設計剖析：「為什麼要這樣做？」(Why & Significance)

### 2.1 為什麼要採用 FreeRTOS 雙任務架構？
* **做法**：將系統拆分為 `ControlTask` (1kHz, Priority=48) 與 `CommTask` (50Hz, Priority=32)。
* **痛點**：若將通訊與控制寫在同一個迴圈，micro-ROS 處理傳輸或等待 Agent 時耗時數毫秒至數百毫秒，會直接破壞控制器的 1ms 週期 (`dt`)，導致馬達產生劇烈抖動或 PID 積分項發散。
* **意義**：`ControlTask` 為最高即時優先級，由 TIM6 硬體中斷以 Task Notification 喚醒，達成 **0 抖動 (Zero Jitter)**；`CommTask` 於背景異步執行，兩者互不干擾。

### 2.2 為什麼要有 Ultimate Safety Gate Pattern？
* **做法**：在呼叫致動器 API (`Motor_SetSpeed` 等) 的前一列，強制執行 `std::isfinite()`、硬體 RPM 限幅與 `RobotSafety_CanDrive()`。
* **痛點**：軌跡演算法或逆運動學 (IK) 在奇異點或除以零時會產生 `NaN` 或 `Inf`。轉型成整數寫入 PWM 暫存器會變成極大值或垃圾值，造成馬達以全速衝撞硬體機構。
* **意義**：給馬達加裝「物理防彈鎖」，不論上層演算法如何出錯，最後一道防線都能暴力阻斷 (Violent Validate)。

### 2.3 為什麼要銷毀舊指令 (Stale Command Annihilation)？
* **做法**：超時 250ms 無新命令時，`memset` 歸零 `RobotCommand_t` 並重置 `has_command` 標誌。
* **痛點**：若佇列留著舊指令，當系統發生 32-bit RTOS tick 溢位（49.7 天）或重新連線時，舊指令會突然被視為「剛收到的新指令」而執行，造成手臂突發性暴衝。
* **意義**：徹底抹除歷史幽靈指令，確保未收到新指令時手臂維持安全停機。

### 2.4 為什麼要進行單軸硬體隔離 (Strict Actuator Isolation)？
* **做法**：測試模式下未選定的軸不僅轉速給 0，還必須呼叫 `Motor_Stop()` 或將 Enable PIN 拉低。
* **痛點**：馬達驅動器在 Enable PIN 為高位準時，即使 PWM 為 0，伺服/步進驅動器仍可能因演算法突波或漏電雜訊產生微小蠕動或高頻震動。
* **意義**：測試一軸時，實體切斷其他兩軸的動力與 Enable，達到真正的「物理硬隔離」。

### 2.5 為什麼要自訂 DMA Transport 並在 Critical Section 關閉 `UART_IT_ERR/PE`？
* **做法**：在 `cubemx_transport_open()` 中以 `taskENTER_CRITICAL()` 關閉 `UART_IT_ERR` 與 `UART_IT_PE`。
* **痛點**：STM32 HAL 庫預設在 DMA 模式下，只要遇到一次線路雜訊（ORE/NE/FE 錯誤），`HAL_UART_IRQHandler()` 就會自動呼叫 `UART_EndRxTransfer()` 徹底中止 DMA 接收，導致 micro-ROS 連線一遇到雜訊就永久鎖死。
* **意義**：阻止 HAL 庫自作聰明去終止 DMA。我們關閉硬體錯誤中斷，並在 polling `read()` 邏輯中「單純清除旗標而不重置指標/不停止 DMA」，讓 Circular DMA Buffer 順暢運轉。

### 2.6 為什麼要移除 0-handle Executor？
* **做法**：在 `MICROROS_STATE_CREATE_ENTITIES` 中，僅初始化 Support, Node, Publisher，不建立 0-handle executor。
* **痛點**：綁定的 `rclc` 版本在 `rclc_executor_init()` 傳入 `number_of_handles = 0` 時會直接回傳 `RCL_RET_INVALID_ARGUMENT` 錯誤，導致狀態機永遠卡在連線重試而無法進入 `RUNNING` 態。
* **意義**：移除無意義的空 Executor，使狀態機成功進入 `RUNNING` 態並正常發布心跳包。

### 2.7 為什麼要在 `RUNNING` 態實作 1000ms 的 `rmw_uros_ping_agent()`？
* **做法**：每秒主動 Ping 一次 Agent。
* **痛點**：若拔除 USB 線或 Agent 崩潰，單純發布 (`rcl_publish`) 的封包可能只是排入本機佇列，需要數秒甚至無法察覺物理斷線。
* **意義**：主動、快速發現實體連線中斷，並立刻切換至 `MICROROS_STATE_DESTROY_ENTITIES` 清理資源，準備重新連線。

### 2.8 為什麼 Agent 斷線時要把 `destroy session timeout` 設為 0？
* **做法**：`rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0)`。
* **痛點**：當 Agent 已經死掉或拔線時，`rcl_publisher_fini` 等函式預設會嘗試傳送 XRCE-DDS 銷毀訊息並等待 Agent 回應。若 Agent 不存在，每個 entity 都會卡死等待 1~2 秒，造成 RTOS 任務嚴重阻斷。
* **意義**：告知 micro-ROS 「Agent 已經離線，請直接在本機銷毀資源，不要傻傻等待回應」，達成極速本地記憶體回收。

---

# 3. 核心 API / 函式與系統指令規格說明 (API & Command Specification)

## 3.1 韌體核心 C/C++ API 規格

| 函式 / API 名稱 | 所在檔案 | 功能說明與參數意義 |
|---|---|---|
| `HAL_TIM_Base_Start_IT(&htim6)` | `freertos.c` | **啟動 1kHz 定時器中斷**。驅動 TIM6 硬體計時器，每 1ms 產生一次 ISR。 |
| `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` | `freertos.c` | **RTOS 任務阻塞等待**。讓 `ControlTask` 進入休眠，直到收到 TIM6 ISR 的 Direct Task Notification，極低延遲喚醒。 |
| `Robot_ControlTask_Tick(0.001f)` | `mainpp.cpp` | **控制主 entry**。帶入 `dt=0.001s`，執行運動學、PID 計算與 Safety Gate 驗證。 |
| `std::isfinite(val)` | `safety_manager.c` | **浮點數有限性檢查**。回傳 `true` 代表數值正常；若為 `NaN` 或 `Inf` 則立刻攔截。 |
| `RobotSafety_CanDrive()` | `safety_manager.c` | **全域安全狀態檢查**。於致動器 API 呼叫前一列執行，回傳 `false` 時禁止驅動馬達。 |
| `HAL_UART_Receive_DMA(uart, buf, size)` | `dma_transport.c` | **啟動 Circular DMA 接收**。讓 DMA1 Stream5 在背景自動將 UART 字元填入環形緩衝區。 |
| `__HAL_UART_DISABLE_IT(uart, UART_IT_ERR)` | `dma_transport.c` | **關閉 UART 錯誤中斷 (EIE)**。清除 CR3 暫存器的 EIE Bit，防止 HAL 在 ORE/NE/FE 時 abort DMA。 |
| `HAL_UART_AbortTransmit(uart)` | `dma_transport.c` | **單獨終止 TX DMA**。當發送逾時 (100ms) 時呼叫，僅停止 TX Stream，不影響 RX Stream。 |
| `rmw_uros_set_custom_transport(...)` | `microros_app.c` | **綁定自訂 Transport**。將 `cubemx_transport_open/close/write/read` 註冊給 micro-ROS。 |
| `rmw_uros_ping_agent(100, 1)` | `microros_app.c` | **Ping 偵測 Agent**。等待 100ms 嘗試連線 Agent，回傳 `RMW_RET_OK` 代表連線正常。 |
| `rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0)` | `microros_app.c` | **設定銷毀逾時為 0**。Agent 斷線時告知 rmw 層無需向 Agent 發送 destroy 封包，直接本機釋放。 |

---

## 3.2 系統與周邊 Shell 指令規格

| 系統指令 | 執行環境 | 功能說明與參數意義 |
|---|---|---|
| `usbipd list` | Windows PowerShell | **查詢 USB 裝置清單**。列出所有 Connected 與 Persisted 的 USB 裝置及其 Bus ID。 |
| `usbipd bind --busid <BUSID>` | Windows PowerShell | **將指定 USB 裝置綁定至 usbipd 轉接驅動**。為附掛至 WSL2 作準備。 |
| `usbipd attach --wsl --busid <BUSID>` | Windows PowerShell | **將 USB 裝置直通透傳至 WSL2**。將 ST-LINK 附掛至 WSL2 虛擬匯流排（例如 `--busid 2-3`）。 |
| `usbipd detach --busid <BUSID>` | Windows PowerShell | **斷開 WSL2 與 USB 裝置的連線**。取消透傳至 WSL2。 |
| `usbipd unbind --busid <BUSID>` | Windows PowerShell | **解除綁定並還原 Windows 原生驅動**。**重要步驟**！解開 usbipd stub 轉接驅動，使 STM32CubeIDE 或 Windows GDB Server 能重新偵測到 ST-LINK。 |
| `ls -l /dev/ttyACM0` | WSL2 Linux | **驗證序列埠裝置**。確認 ST-LINK VCP 已成功註冊為 `/dev/ttyACM0` 字元裝置。 |
| `dmesg \| tail -n 20` | WSL2 Linux | **檢視 Linux Kernel 日誌**。確認出現 `cdc_acm ... ttyACM0: USB ACM device` 訊息。 |
| `.\scripts\start_vcp_bridge.ps1 -ComPort COM6 -BaudRate 460800 -TcpPort 8888` | Windows PowerShell | **啟動目前已驗證的 Windows VCP-to-container bridge 與 Humble Agent**。Windows 原生 pyserial 持有 COM6；container 透過 TCP/socat pseudo-TTY 連接。Bridge 模式不可同時由 usbipd 將同一裝置 attach 到 WSL2。 |
| `ros2 topic echo /doghead/heartbeat` | WSL2 Linux (ROS 2) | **監聽心跳 Topic**。檢視來自 STM32 每 100ms 發布之 `std_msgs/msg/UInt32` 心跳數據。 |

---

**維護團隊**: Development & Safety Audit Team  
