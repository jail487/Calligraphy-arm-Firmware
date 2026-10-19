# micro-ROS 通訊架構、自訂 DMA Transport 與連線狀態機手冊

> [!IMPORTANT]
> 現行狀態機含 Reliable session-control subscription；init、disarm、create failure、
> destroy 與 Agent disconnect 都會清除 active session 與 command mailbox。

**文件位置**：`docs/ros2_integration/01_microros_architecture_and_state_machine.md`  
**最後更新**：2026-07-30 (整合 Phase 1 ~ Phase 3 變更與第二輪 Code Review 修復)  

---

## 1. 系統架構概覽

在嵌入式系統 (FreeRTOS) 中執行 micro-ROS，採用獨立的 `CommTask` (50Hz) 驅動 **自動重連狀態機 (Connection State Machine)**，與 1kHz 即時 `ControlTask` 完全解耦。

```text
┌─────────────────────────────────────────────────────────────┐
│                 FreeRTOS 任務架構                           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌────────────────────────┐    ┌─────────────────────────┐  │
│  │  ControlTask           │    │   CommTask              │  │
│  │  (1kHz, P=48)          │    │   (50Hz, P=32)          │  │
│  │                        │    │                         │  │
│  │  - 馬達驅動            │    │  - micro-ROS            │  │
│  │  - Safety Gate 驗證    │    │    自動重連狀態機       │  │
│  │  - PID / 軌跡計算      │    │  - Spin / Heartbeat     │  │
│  └────────────────────────┘    └─────────────────────────┘  │
│             ↓                               ↑               │
│             └──────────────┬────────────────┘               │
│                            ↓                                │
│                  ┌───────────────────┐                      │
│                  │   USART2 + DMA    │                      │
│                  └───────────────────┘                      │
└────────────────────────────┬────────────────────────────────┘
                             │ UART 460800 8N1 (Custom DMA Transport)
                             ▼
                  ┌────────────────────┐
                  │  micro-ROS Agent   │
                  │ (WSL2 / Docker /PC)│
                  └────────────────────┘
```

---

## 2. 客製化 DMA Transport 運作原理 (`dma_transport.c`)

在 `rmw_microxrcedds` 中，傳輸層由四個主要 Hook 函式組成：

```c
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);
```

### 2.1 關鍵修復與中斷安全機制
1. **Open (開啟傳輸)**：
   * 清空內部 ring index (`dma_head = dma_tail = 0`)。
   * 進入 `taskENTER_CRITICAL()` 呼叫 `HAL_UART_Receive_DMA()` 啟動 USART2 RX Circular DMA。
   * **關閉中斷防範**：呼叫 `__HAL_UART_DISABLE_IT(uart, UART_IT_ERR)` 與 `__HAL_UART_DISABLE_IT(uart, UART_IT_PE)` 關閉 HAL 線路錯誤中斷。可防止實體線路產生 ORE/NE/FE 雜訊時，`HAL_UART_IRQHandler()` 自動將 RX DMA 停止/Abort。
2. **Write (發送與 Timeout)**：
   * 呼叫 `HAL_UART_Transmit_DMA(&huart2, buf, len)` 送出 DDS 封包。
   * 採用 FreeRTOS tick 進行 100ms 超時等待（`UART_TX_TIMEOUT_MS`）。超時呼叫 `HAL_UART_AbortTransmit(uart)` 僅單獨終止 TX DMA Stream，**絕不破壞 RX Circular DMA**。
3. **Read (接收與環形緩衝區)**：
   * 採用 **Circular DMA Buffer**。
   * 讀取時檢查 ORE/NE/FE 旗標；若有錯誤僅呼叫 `__HAL_UART_CLEAR_*FLAG` 清除，**絕對不重置 `dma_head` 或停止 DMA**。
   * 計算 `dma_tail_loc = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx)` 拷貝資料。

---

## 3. 連線狀態機技術細節 (`microros_app.c`)

```c
typedef enum {
    MICROROS_STATE_WAIT_AGENT,      // 等待 Agent 在線上
    MICROROS_STATE_CREATE_ENTITIES, // 建立 Support, Node, Publisher
    MICROROS_STATE_RUNNING,         // 正常 Spin、Ping 偵測與 1Hz 發布心跳包
    MICROROS_STATE_DESTROY_ENTITIES // 連線中斷，安全釋放記憶體
} MicroROSState_t;
```

### 3.1 狀態轉移細節
- **`WAIT_AGENT`**：每 100ms 呼叫 `rmw_uros_ping_agent(100, 1)`，成功轉入 `CREATE_ENTITIES`。
- **`CREATE_ENTITIES`**：
  1. `allocator = rcl_get_default_allocator();`
  2. `rclc_support_init(&support, 0, NULL, &allocator);`
  3. `rclc_node_init_default(&node, "doghead_arm_node", "", &support);`
  4. `rclc_publisher_init_default(&pub_heartbeat, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/doghead/heartbeat");`
  5. 建立 JointState、McuState、McuDiagnostics publishers，Reliable
     `McuSessionControl`、JointSetpoint、MotorTest subscriptions 與 executor。
  6. 完成後進入 `MICROROS_ENTITY_STAGE_COMPLETE`，自動清除舊有離線診斷紀錄 (`g_microros_last_disconnect_reason = MICROROS_DISCONNECT_NONE`)。
- **`RUNNING`**：每 1000ms 執行一次 Ping 離線偵測，並以 1 Hz 頻率 (FreeRTOS 刻度包容算術) 發布 `/doghead/heartbeat` 心跳包。若 Ping 或任一 Topic 發布失敗，精確記錄失敗原因 (`g_microros_last_disconnect_reason`) 與 RCL/RMW 回傳碼 (`g_microros_last_error_rc`) 後轉入 `DESTROY_ENTITIES`。
- **`DESTROY_ENTITIES`**：先將 active session 設為 0 並清空 command mailbox；若連線
  已斷開，再設定 destroy timeout 為 0，按反向順序釋放 executor、subscriptions、
  publishers、node 與 support 後回到 `WAIT_AGENT`。

---

## 4. 460800 Baud HIL 基線與 Telemetry Overload 分析

### 4.1 遙測封包頻寛需求計算
Phase-1 完整 Telemetry 集合包含：
- **50 Hz McuState** (`/doghead/mcu/state`, 184 位元組 DDS 負載) -> 約 9.2 kB/s
- **50 Hz JointState** (`/joint_states`, 124 位元組 DDS 負載) -> 約 6.2 kB/s
- **1 Hz McuDiagnostics** (`/doghead/mcu/diagnostics`) -> 約 0.1 kB/s
- **1 Hz Heartbeat** (`/doghead/heartbeat`) -> 約 0.02 kB/s

**淨資料庫負載頻寬**：約 15.4 kB/s（尚未加算 Micro-XRCE-DDS 標頭與邊框）。

### 4.2 115200 Baud 超載原因
在 115200 8N1 規格下（1 bit 起始位 + 8 bit 資料 + 1 bit 停止位 = 10 bits/byte），物理極限傳輸率為：
$$\text{理論最大吞吐量} = \frac{115200 \text{ bps}}{10 \text{ bits/byte}} = 11,520 \text{ B/s} = 11.52 \text{ kB/s}$$

在 T24 修復前，Heartbeat 每 5ms Spin 即發布一次 (200 Hz)，導致超過 16.2 kB/s 負載。即使將 Heartbeat 降頻至 1 Hz，50 Hz McuState (9.2 kB/s) + 50 Hz JointState (6.2 kB/s) 合計 15.4 kB/s 仍超出 115200 鮑率上限 33% 以上，必然引發 TX 佇列溢位、傳輸 Timeout 與連線不斷重新建立/銷毀 (destroy/recreate cycle)。

### 4.3 採用 460800 Baud 的 HIL 基線理由
在 460800 8N1 規格下：
$$\text{理論最大吞吐量} = \frac{460800 \text{ bps}}{10 \text{ bits/byte}} = 46,080 \text{ B/s} = 46.08 \text{ kB/s}$$

15.4 kB/s 遙測負載僅佔用約 33% 頻寬，為 XRCE-DDS 標頭、雙向指令訂閱以及 Ping/Pong 檢測提供充裕頻寬，確保零封包遺失與極致穩定性。

### 4.4 斷線診斷記錄機制 (`g_microros_last_disconnect_reason`)
為在 Entity 清除前與過程中保留明確斷線原因，系統提供 volatile 變數與列舉：
- `g_microros_last_disconnect_reason`:
  - `MICROROS_DISCONNECT_NONE` (0)
  - `MICROROS_DISCONNECT_PING_FAIL` (1)
  - `MICROROS_DISCONNECT_HEARTBEAT_PUB_FAIL` (2)
  - `MICROROS_DISCONNECT_JOINT_STATES_PUB_FAIL` (3)
  - `MICROROS_DISCONNECT_MCU_STATE_PUB_FAIL` (4)
  - `MICROROS_DISCONNECT_MCU_DIAG_PUB_FAIL` (5)
- `g_microros_last_error_rc`: 紀錄最後一次失敗之 RCL (`rcl_ret_t`) 或 RMW (`rmw_ret_t`) 錯誤碼。

### 4.5 低延遲 Bridge 接收機制需求 (Low-Latency Bridge Serial Read Requirement)
為了防止 Windows Python VCP Bridge 在轉接 XRCE Ping/Pong 小封包時產生人為延遲，Host 端 Bridge (`win_vcp_bridge.py`) 採用 **有界低延遲清空 (Bounded Low-Latency Drain)** 機制：
1. 串列埠接收超時時間設定為 `timeout=0.01` (10 ms)，避免預設 `ser.read(4096)` 堵塞滿 100 ms 而耗盡韌體 100 ms 的 Ping 應答期限。
2. 接收邏輯採用 `ser.read(1)` 搭配 `ser.in_waiting` 即時清空：收到第一個位元組後立即將當前作業系統 Serial RX 緩衝區中的所有封包轉發至 TCP Socket，使 XRCE Ping/Pong 封包能在 < 2 ms 內完成轉接，徹底防止 `MICROROS_DISCONNECT_PING_FAIL` (rc=1) 誤判與重連。

### 4.6 高頻遙測 QoS 契約 (Best-Effort Telemetry QoS Contract)
為了消解串列埠 Reliable 通訊中 ACK 應答延遲導致 20 ms 遙測週期被拉長至 ~27 ms (頻率拉低至 ~36.5 Hz) 的問題：
1. **50 Hz 高頻遙測 Topics** (`/doghead/mcu/state` 與 `/joint_states`)：韌體發布者切換為 `rclc_publisher_init_best_effort` (Best-Effort QoS)；ROS 2 Host 端訂閱者 (`doghead_driver` 與 `telemetry_validator`) 顯式配置 `QoSReliabilityPolicy.BEST_EFFORT`，使 50 Hz 遙測無障礙達到精確 50.0 Hz 發布率。
2. **低頻與控制 Topics** (`/doghead/heartbeat` 1 Hz, `/doghead/mcu/diagnostics` 1 Hz, `/doghead/mcu/joint_setpoint`, `/doghead/mcu/motor_test_command`)：繼續維持 `RELIABLE` 預設 QoS 契約，確保心跳、診斷與控制指令封包之可靠傳輸。
