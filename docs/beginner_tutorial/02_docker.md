# 02：Docker 從零到能開發

## Docker 解決什麼

ROS 2 Humble 的主要平台是 Ubuntu 22.04。Docker image 把作業系統使用者空間、ROS 套件與工具版本封裝起來，讓 Windows 開發機不用把所有 ROS 依賴直接安裝到 host。

- **image**：唯讀模板，例如 `osrf/ros:humble-desktop-full`。
- **container**：image 的執行實例，有自己的 process、filesystem view 與 network namespace。
- **volume/bind mount**：把 host 路徑映射進 container；本專案把 `./` 掛到 `/workspace`。
- **Compose**：用 YAML 描述多個 container 的 image、command、mount、device 與 network。

修改 `/workspace` 內的檔案會直接修改 Windows 專案，因為它是 bind mount；安裝在 container 其他位置的臨時套件，container 刪除後通常會消失。

## 逐段看 `docker-compose.yml`

### `ros2-dev`

```yaml
image: osrf/ros:humble-desktop-full
network_mode: host
volumes:
  - ./:/workspace:rw
working_dir: /workspace/Software/ros2_ws
command: /bin/bash -c "source /opt/ros/humble/setup.bash && tail -f /dev/null"
```

`tail -f /dev/null` 只是維持容器常駐，真正的工作命令由 `docker compose exec` 啟動。`working_dir` 讓進入後位於 ROS workspace。X11 mount 和 `DISPLAY` 是 GUI 支援；純 CLI 不依賴它。

### direct Agent

`microros-agent` 把 Linux `/dev/ttyACM0` 映射成容器 `/dev/ttySTM32`，再以 460800 baud 開啟 serial transport。它使用 profile `direct-agent`，不指定 profile 時不會啟動。

### Windows bridge Agent

Windows 的 COM port 通常不能直接以 Linux device 傳入容器，因此使用：

```text
STM32 VCP <-> Windows COM6 <-> PowerShell/pyserial <-> TCP 8888
           <-> socat pseudo-TTY /dev/ttySTM32 <-> micro_ros_agent
```

`microros-agent-bridge` 啟動時安裝/執行 `socat`，建立 pseudo-TTY，再讓 Agent 以為自己讀的是 serial device。

## 安全啟動開發容器

在專案根目錄的 PowerShell：

```powershell
docker compose up -d ros2-dev
docker compose ps
docker compose exec ros2-dev bash
```

在容器內：

```bash
source /opt/ros/humble/setup.bash
cd /workspace/Software/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep doghead
```

`--symlink-install` 對 Python package 很方便：多數 `.py` 修改可由 symlink 反映；但改了 `setup.py`、message、CMake 或 package metadata 仍應 rebuild。

### 一次性建置與測試容器

不需要長時間保留測試容器時，建議在專案根目錄執行：

```powershell
.\scripts\run_ros2_tests.ps1
```

這個腳本會建立一次性的 ROS 2 container，依序執行 build、test 和 test-result；全部成功後自動刪除該 container。若測試失敗，container 會保留，方便查看 log 和重現問題。測試容器名稱會使用 `doghead_ros2_test_*`，不會刪除長時間使用的 `doghead_ros2_dev`。

## Overlay workspace

ROS 環境是疊加的：

```text
/opt/ros/humble       underlay：官方 ROS
install/              overlay：本專案 build 結果
```

順序必須是先 underlay 再 overlay：

```bash
source /opt/ros/humble/setup.bash
source /workspace/Software/ros2_ws/install/setup.bash
```

如果先 source 舊的 `install` 再 clean build，可能造成 CMake/Python 路徑混亂。乾淨 shell 中 build 最容易重現。

## Compose 常用命令

```powershell
docker compose config
docker compose up -d ros2-dev
docker compose logs -f --tail 100 ros2-dev
docker compose exec ros2-dev bash
docker compose stop ros2-dev
docker compose down
```

`stop` 保留 container；`down` 移除 Compose 建立的 container/network，但不會刪除 bind-mounted 專案檔。不要隨便加 `-v`，因為它會刪除 named volumes。

啟動 direct Agent：

```powershell
docker compose --profile direct-agent up -d microros-agent
```

啟動 bridge Agent：

```powershell
.\scripts\start_vcp_bridge.ps1 -ComPort COM6 -BaudRate 460800 -TcpPort 8888
docker compose --profile compose-bridge-agent up -d microros-agent-bridge
docker compose logs -f microros-agent-bridge
```

請以 Device Manager 實際 port 取代 COM6。兩種 Agent 路徑擇一，避免兩個程序同時占用同一 transport。

## Networking 與 DDS

ROS 2 discovery 依賴 DDS/UDP multicast。`network_mode: host` 可減少 container NAT 與 multicast 的複雜度，但 Docker Desktop 的 host networking 行為仍與原生 Linux不同。若 node 看不到彼此，依序核對：

```bash
printenv ROS_DOMAIN_ID
printenv RMW_IMPLEMENTATION
ros2 daemon stop
ros2 daemon start
ros2 node list
```

同一個 ROS graph 的程序應使用相同 `ROS_DOMAIN_ID`。公司 VPN、防火牆與多張網卡都可能影響 multicast。

## 常見錯誤

| 症狀 | 原因 | 處理 |
|---|---|---|
| 容器內看不到剛改的檔案 | 不在 bind mount 或路徑錯 | `mount`、`pwd`、`ls /workspace` |
| `Package not found` | 未 build 或未 source overlay | build 後 `source install/setup.bash` |
| `/dev/ttySTM32` 不存在 | profile/bridge/socat 未成功 | 看 Compose log 與 TCP 8888 |
| serial permission denied | Linux device 權限 | 檢查 `ls -l /dev/ttyACM0`、群組/udev |
| GUI 無法開 | DISPLAY/X server | 先用 CLI；GUI 另處理 X11 |

## 本章實作

1. 用 `docker compose config` 找到 `/workspace` mount。
2. 進容器建立一個暫存文字檔，再回 Windows 確認它出現在專案中，然後手動刪除。
3. 分別在未 source 與 source overlay 的 shell 執行 `ros2 pkg prefix doghead_driver`，解釋差異。
