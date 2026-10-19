# 未來 STM32 專案微服務 micro-ROS 快速遷移與注意事項指南

> [!CAUTION]
> `libmicroros.a` 不可僅因同為 Cortex-M4F 就假設可直接重用；ROS distribution、
> message set、FPU ABI、compiler flags 與 packing attributes 都必須一致。建議每個
> target 依 `micro_ros_header_generation.md` 重新產生並驗證。

**文件位置**：`docs/ros2_integration/04_future_project_reusability_guide.md`  
**最後更新**：2026-07-30  
**目標**：作為未來任何 STM32 (Cortex-M4) + FreeRTOS 專案快速導入 micro-ROS 的標準 SOP 範本、避坑指南與資產複用手冊。

---

## ⚡ 1. 5 分鐘極速遷移 SOP (Fast Migration Checklist)

若未來的專案滿足：**STM32F4 系列 (Cortex-M4F)** + **FreeRTOS** + **ROS 2 Humble**，直接遵循以下 5 步驟快速完成對接：

### 步驟 1：複製資產檔案至新專案 (1 分鐘)
將本專案已驗證的黃金模組複製到新專案對應目錄：
1. `libmicroros.a` 與 `microros_include/` (靜態庫與標頭檔)
2. `dma_transport.c` / `dma_transport.h` (底層 DMA 二進位驅動)
3. `microros_app.c` / `microros_app.h` (連線自動重連狀態機)

### 步驟 2：STM32CubeMX 周邊設置 (2 分鐘)
1. **FreeRTOS**：
   - Dynamic Heap 配置至少 `49152` (48KB)。
   - 建立 `CommTask`：Stack `1024` words (4KB)，Priority 設為 `osPriorityAboveNormal` (32)。
2. **USART 周邊**：
   - 開啟 USART（例如 USART2）。baud 應依實際 message wire-rate 計算；本專案
     50 Hz 完整 telemetry 的已驗證值為 460800 8N1。
   - 配置 DMA：RX 為 **Circular Mode**，TX 為 **Normal Mode**。
   - NVIC Priority：USART 與 DMA 中斷優先級設為 `6` (低於 FreeRTOS Max Syscall Priority 5)。

### 步驟 3：設定 Compiler / Linker 參數 (1 分鐘)
在 STM32CubeIDE 或 Makefile 的 C/C++ Compiler 與 Linker Flags 中加入 ABI 參數：
```make
-mfloat-abi=softfp -mfpu=fpv4-sp-d16
```

### 步驟 4：自訂商務 Topic (1 分鐘)
打開複製過去的 `microros_app.c`，將 `MICROROS_STATE_CREATE_ENTITIES` 中的 Topic 名稱與 Data Type 修改為新專案所需（例如改為 `/my_robot/joint_states` 或 `/my_robot/cmd`）。

### 步驟 5：電腦端啟動 Agent
在 WSL2/Linux 執行標準 Agent 命令：
```bash
docker run -it --rm --net=host --device=/dev/ttyACM0:/dev/ttySTM32:rw microros/micro-ros-agent:humble serial --dev /dev/ttySTM32 -b 460800 -v6
```

---

## 🧩 2. 資產可複用性矩陣 (Reusability Matrix)

| 檔案 / 模組名稱 | 複用程度 | 說明 |
|---|---|---|
| **`libmicroros.a`** | **必須驗證或重建** | 除 Cortex-M4F/Humble 外，message set、FPU ABI、compiler flags、packing attributes 與 transport profiles 也必須完全一致。 |
| **`dma_transport.c`** | **100% 直用** | 二進位 DMA Circular 傳輸與中斷防禦，只要同為 USART DMA 即可全自動運轉。 |
| **`microros_app.c` (狀態機)** | **90% 結構直用** | 四態重連 (`WAIT_AGENT` -> `CREATE_ENTITIES` -> `RUNNING` -> `DESTROY_ENTITIES`) 與 1000ms Ping 結構直用，僅需改寫具體 Pub/Sub Topic。 |
| **`cflags.mk` / `.cproject` ABI** | **100% 直用** | 直接套用 `-mfloat-abi=softfp -mfpu=fpv4-sp-d16`。 |
| **WSL2 / Docker 腳本與 SOP** | **100% 直用** | `usbipd` 附掛與 Docker Agent 啟動命令完全通用。 |

---

## ⚠️ 3. 未來專案踩坑注意事項與避雷防線 (Critical Precautions)

### 🚨 注意事項一：FPU ABI 位元組級對齊 (避免開機即 HardFault)
* **避坑要點**：STM32CubeIDE 新建專案預設常帶 `-mfloat-abi=hard` 或軟浮點 `-mfloat-abi=soft`。一旦不匹配 `libmicroros.a` 的 `softfp`，晶片在呼叫數學或時間運算時會**開機立刻 HardFault 猝死**。
* **防線**：檢查 Compiler 與 Linker **兩者**皆帶有 `-mfloat-abi=softfp -mfpu=fpv4-sp-d16`。

### 🚨 注意事項二：關閉 UART 錯誤中斷 (避免 DMA RX 遭暴力中止)
* **避坑要點**：STM32 HAL 庫在接收 DMA 時預設開啟 `EIE` (Error Interrupt Enable)。實體線路只要產生微小雜訊 (ORE/NE/FE 錯誤)，`HAL_UART_IRQHandler()` 就會自動清掉 `DMAR` 旗標並 abort DMA。
* **防線**：`cubemx_transport_open()` 必須在 `taskENTER_CRITICAL()` 內呼叫 `__HAL_UART_DISABLE_IT(uart, UART_IT_ERR)` 與 `UART_IT_PE`。且在 `read()` 時「只清硬體旗標，絕不停止 DMA 或重置 ring index」。

### 🚨 注意事項三：Executor 與 0-Handle 陷阱
* **避坑要點**：若新專案只需要 Publisher 而無 Subscriber/Timer，**不要初始化 0-handle 的 Executor** (`rclc_executor_init(..., 0, ...)`). `rclc` 函式庫內部會回傳 `INVALID_ARGUMENT` 導致狀態機無限重試。
* **防線**：僅在確實有 Subscription / Timer 處理時才建立 Executor，並將 handle 數量設為實質數量。

### 🚨 注意事項四：Agent 斷線時的離線銷毀 (`timeout = 0`)
* **避坑要點**：當實體線路拔除或 Agent 崩潰時，呼叫 `rcl_*_fini()` 預設會嘗試傳送 XRCE DDS 銷毀訊息並等待 Agent 回應。若 Agent 不存在，任務會阻斷數秒。
* **防線**：轉入 `DESTROY_ENTITIES` 時，若判定 `agent_disconnected == true`，先執行 `rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0)`，達成極速本地記憶體回收。

### 🚨 注意事項五：PC 端 TTY 獨佔性與 Debug 搶佔
* **避坑要點**：
  1. Linux 背景若殘留多個 Agent 容器，序列埠資料會被多個容器隨機分刮，造成資料破碎連線持續超時。啟動 Docker 必須帶 `--rm`。
  2. 在 Windows IDE 按下 Debug 時，ST-LINK 的 USB 控制權會被 Windows 搶走，造成 WSL2 的 `/dev/ttyACM0` 消失。Debug 結束後須重新於 PowerShell 執行 `usbipd attach --wsl`。

---

## 💡 4. 什麼時候才「需要」重新開 Docker 編譯 `libmicroros.a`？

若未來專案出現以下情況之一，才需要重新使用 Docker 開啟 `micro_ros_setup` 環境重新編譯 static library：

1. **更換 ROS 2 Distro**：從 Humble 換成 Iron / Jazzy / Rolling。
2. **更換 MCU 指令集**：從 Cortex-M4 換成 Cortex-M0/M3 (無 FPU) 或 Cortex-M7/H7 (雙精度 FPU `fpv5-d16`)。
3. **新增自訂 ROS 2 訊息**：定義了專案專屬的 `.msg` 或 `.srv` 檔案，需要重新生成 C 語言對應的 msg code。

---

**維護團隊**: Development & Safety Audit Team  
