# WSL2 USB Passthrough 與 micro-ROS Agent 操作指南

> [!IMPORTANT]
> 本專案目前採 Windows-native COM6 bridge + Docker Agent，標準 baud 為 460800。
> 不應同時讓 usbipd/WSL2 與 Windows bridge 搶占同一個 ST-Link VCP。

**文件位置**：`docs/ros2_integration/02_wsl2_usb_and_agent_setup_guide.md`  
**最後更新**：2026-07-30  

---

## 1. WSL2 與 ST-LINK USB 直通設定 SOP

在 Windows + WSL2 開發環境中，硬體直通 (USB Passthrough) 常因硬體重插或 IDE Debug 搶佔而中斷。請遵循以下標準作業程序：

### 1.1 Windows PowerShell 操作 (系統管理員權限)
1. 查詢 ST-LINK 裝置 Bus ID：
   ```powershell
   usbipd list
   ```
   *尋找 `ST-Link Debug` 或 `USB Serial Device` 對應的 BUSID (例如 `2-3`)*。
2. 綁定並附掛至 WSL2：
   ```powershell
   usbipd bind --busid 2-3
   usbipd attach --wsl --busid 2-3
   ```
3. 若要將 ST-LINK 完全切回 Windows (供 STM32CubeIDE 除錯或燒錄時)：
   ```powershell
   # 1. 斷開 WSL2 連線
   usbipd detach --busid 2-3
   # 2. 解除綁定（重要！還原原生 Windows ST-LINK 驅動，否則 CubeIDE 無法識別 ST-LINK）
   usbipd unbind --busid 2-3
   ```

### 1.2 WSL2 終端機驗證
1. 確認 `/dev/ttyACM0` 已產生：
   ```bash
   ls -l /dev/ttyACM0
   ```
2. 檢查 Linux 驅動日誌：
   ```bash
   dmesg | tail -n 20
   ```
   *必須出現 `cdc_acm ... ttyACM0: USB ACM device`*。

> ⚠️ **重要提示**：`usbipd bind` 會暫時將原生的 Windows ST-LINK 驅動替換為 usbipd 轉接驅動。要在 STM32CubeIDE 進行 Debug 或燒錄時，**必須執行 `usbipd unbind` 還原原生驅動**，IDE 才能正常偵測到 ST-LINK！

---

## 2. micro-ROS Agent 啟動與測試

### 2.1 使用 Docker 啟動唯一 Agent
```bash
docker run -it --rm --net=host \
  --device=/dev/ttyACM0:/dev/ttySTM32:rw \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttySTM32 -b 460800 -v6
```

> ⚠️ **防呆警語**：背景嚴禁同時運行多個 Agent 容器！多個 Agent 讀取同一個 TTY 會導致 Linux 隨機瓜分字元，造成封包破碎且無法建立 Session。

### 2.2 ROS 2 Topic 檢驗
在 WSL2 另開終端機：
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /doghead/heartbeat
```
*正常情況下每 1 秒 (1 Hz) 會收到自增的心跳數字*。

---

## 3. Windows ST-Link VCP 雙向 TCP Bridge 模式 (推薦 WSL2 環境)

### 3.1 根本原因與適用場景

在部分 Windows + WSL2 / Docker Desktop 環境中，`usbipd` 的硬體轉接存在**單向傳輸失效問題**：
- Docker / WSL2 可正常讀取 STM32 上傳的 UART 封包（MCU 傳至 Agent）。
- Agent 寫入（Agent 傳至 MCU）在 WSL2 `strace` 中顯示 `write(42 bytes)` 成功，但 STM32 USART2 DR 接收暫存器始終為 0。
- 直接在 Windows 以原生 `pyserial` 操作 `COM6` 注入資料時，STM32 能 100% 正常收到。

**使用時機**：
- **Bridge 模式 (`start_vcp_bridge.ps1`)**：當在 Windows Host + Docker Desktop / WSL2 上開發，且 `usbipd` 出現 Host-to-Target OUT 封包丟失時使用。
- **Direct 模式 (`/dev/ttyACM0`)**：當運行於原生 Linux 主機，或 `usbipd` 在該環境具備完整雙向轉接能力時使用。

> ⚠️ **關鍵前置條件：嚴禁 `usbipd attach` 佔用 VCP**  
> 啟用 Bridge 模式時，**切勿將 ST-Link VCP attach 到 WSL2**！若 `usbipd` 已 attach，Windows 原生 `pyserial` 將無法開啟 `COM6`（提示 Access Denied）。請先執行：
> ```powershell
> usbipd detach --busid <BUSID>
> usbipd unbind --busid <BUSID>
> ```

### 3.2 可設定參數

PowerShell 入口腳本 `scripts/start_vcp_bridge.ps1` 與 Python Bridge `scripts/win_vcp_bridge.py` 支援以下設定：

| 參數名 | 預設值 | 說明 |
|---|---|---|
| `-ComPort` / `--com` | `COM6` | Windows ST-Link VCP 串列埠代號 |
| `-BaudRate` / `--baud` | `460800` | UART 通訊鮑率 (HIL 標準基線) |
| `-TcpPort` / `--port` | `8888` | Windows Host 與 Docker 容器間 TCP 轉接埠 |
| `-ContainerName` | `doghead_microros_agent_bridge` | micro-ROS Agent Docker 容器名稱 |

### 3.3 啟動與停止步驟

#### 方法 A：單鍵 PowerShell 一鍵啟動 (推薦)
在 Windows PowerShell 終端機執行：
```powershell
.\scripts\start_vcp_bridge.ps1 -ComPort COM6 -BaudRate 460800 -TcpPort 8888
```
該腳本會自動：
1. 檢查 Python、`pyserial` 與 Docker 狀態。
2. 啟動背景 Python 雙向 Serial-to-TCP Bridge (`COM6 <-> TCP:8888`)。
3. 覆寫容器預設 Entrypoint 為 `/bin/sh`（避免被舊 `/micro-ros_entrypoint.sh` 誤解為參數）。
4. 在容器內啟動 `socat` 建立 `/dev/ttySTM32`，並先檢查 `socat` 與 TCP 監聽器連接狀態（若連線失敗立即中斷，防止 Agent 盲目啟動）。
5. 自動載入 `/opt/ros/humble/setup.sh` 與 `/uros_ws/install/setup.sh` 環境變數。
6. 執行 `/uros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent` 通訊代理。

#### 方法 B：Bash 腳本啟動 (WSL2 / Linux 側)
1. 在 Windows 先啟動 Python Bridge：
   ```powershell
   python .\scripts\win_vcp_bridge.py --com COM6 --baud 460800 --port 8888
   ```
2. 在 WSL2 / Bash 中啟動 Agent：
   ```bash
   ./scripts/start_microros_agent.sh bridge host.docker.internal 8888 460800
   ```

#### 方法 C：Docker Compose 啟動
```bash
docker compose up microros-agent-bridge
```

### 3.4 安全停止流程 (Clean Shutdown)

按下 `Ctrl+C` 結束 PowerShell 或 Bash 腳本時，系統會自動：
1. 清除並關閉 Docker Agent 容器 (`doghead_microros_agent_bridge`)。
2. 強制終止背景運行的 Python TCP Bridge 行程，釋放 COM6 串列埠與 TCP 8888 埠。
3. 絕不殘留孤兒行程 (Orphan processes)。

---

## 4. 460800 HIL 基線與診斷覆寫指引 (Diagnostic Override Path)

### 4.1 鮑率基線選擇理由
Phase-1 Telemetry 包含 50 Hz `/doghead/mcu/state` (184 bytes), 50 Hz `/joint_states` (124 bytes), 1 Hz `/doghead/heartbeat`, 1 Hz `/doghead/mcu/diagnostics`。
- 淨遙測負載流量約 15.4 kB/s。
- 115200 8N1 之理論極限頻寬僅 $115200 / 10 = 11.52 \text{ kB/s}$，無法傳輸 Phase-1 50 Hz 完整遙測。
- 460800 8N1 之理論極限頻寬為 $460800 / 10 = 46.08 \text{ kB/s}$，負載僅佔用 ~33% 頻寬，為 HIL 穩定執行的標準基線。

### 4.2 診斷建置鮑率覆寫方法 (Explicit Diagnostic Override)
若需要進行低鮑率除錯或診斷測試，可在編譯韌體與啟動腳本時指定覆寫：
1. **韌體編譯**：在 C/C++ 編譯選項中指定預處理巨集 `-DMICROROS_BAUD_RATE=115200`（或任意自訂鮑率），即可覆寫 `main.h` 預設之 `460800U`。
2. **Bridge / Agent 腳本**：傳入對應的 `-BaudRate` 參數，例如：
   ```powershell
   .\scripts\start_vcp_bridge.ps1 -ComPort COM6 -BaudRate 115200 -TcpPort 8888
   ```

### 4.3 低延遲 Bridge 轉接需求 (Low-Latency Bridge Read)
Host 端 `scripts/win_vcp_bridge.py` 已實作 10 ms (`timeout=0.01`) 搭配 `ser.read(1) + ser.in_waiting` 的即時清空機制，避免預設 `read(4096)` 滿 100 ms 超時才轉發小封包，確保 XRCE Ping 應答延遲控制在 2 ms 以內，徹底消解 `MICROROS_DISCONNECT_PING_FAIL` (rc=1) 離線誤觸發。

### 4.4 遙測 QoS 契約與 CLI 工具操作 (QoS Contract & CLI Usage)
高頻 50 Hz 遙測 Topics (`/doghead/mcu/state` 與 `/joint_states`) 已配置為 **BEST_EFFORT** QoS：
- **慣用量測工具**：建議使用 `telemetry_validator` 進行 50 Hz 遙測的 Rate、Jitter、Sequence 連續性與 MCU Uptime 評估：
  ```bash
  ros2 run doghead_hil telemetry_validator
  # 或直接訂閱 5 秒彙整報告：
  ros2 topic echo /doghead/hil/telemetry_report
  ```
- **CLI 檢驗 (Humble 格式)**：注意 ROS 2 Humble 的 `ros2 topic hz` 命令不支援 `--qos-reliability` 參數。欲在 CLI 觀察 BEST_EFFORT Topic，請使用相容的 `ros2 topic echo` CSV 輸出搭配 sequence / uptime 分析：
  ```bash
  ros2 topic echo /doghead/mcu/state --csv --qos-reliability best_effort
  ```
- 1 Hz 心跳 (`/doghead/heartbeat`) 與 1 Hz 診斷 (`/doghead/mcu/diagnostics`) 維持 `RELIABLE` QoS，可直接使用 `ros2 topic hz` 或 `ros2 topic echo` 訂閱：
  ```bash
  ros2 topic hz /doghead/heartbeat
  ros2 topic hz /doghead/mcu/diagnostics
  ```
