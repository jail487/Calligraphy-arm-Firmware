# 00：學習地圖與開發環境

## 本章目標

讀完後，你要能指出一條命令在哪個作業系統、程序、ROS node 與 FreeRTOS task 中執行，並知道一個終端機中的環境設定為何不會自動出現在另一個終端機。

## 先建立四層心智模型

| 世界 | 本專案中跑什麼 | 常用工具 | 最常見誤解 |
|---|---|---|---|
| Windows host | 專案檔、Docker Desktop、COM port、PowerShell bridge | PowerShell、Docker | 把 Windows 的 COM6 當作 Linux `/dev/tty*` |
| Linux container | ROS 2 Humble、colcon、Python nodes、micro-ROS Agent | bash、ros2、colcon | 忘了每個 shell 都要 `source` |
| ROS graph | nodes、topics、services、parameters、DDS discovery | `ros2 node/topic/service/...` | 把 topic 當成「函式呼叫」 |
| STM32/FreeRTOS | CommTask、ControlTask、ISR、driver、safety state | CubeIDE、SWD、UART | 以為 ROS callback 可直接保證 1 ms deadline |

Docker container 是 Windows 上的一個隔離 Linux 使用者空間；ROS graph 是跨程序的通訊拓樸；STM32 是另一台處理器。三者並不是同一層。

## 倉庫導覽

```text
Calligraphy-arm-Firmware/
├─ docker-compose.yml               容器服務定義
├─ scripts/start_vcp_bridge.ps1     Windows COM ↔ TCP bridge
├─ Software/ros2_ws/                ROS 2 workspace
│  └─ src/
│     ├─ doghead_interfaces/        自訂 msg
│     ├─ doghead_driver/            host 安全與 MCU adapter
│     ├─ doghead_motion/            IK 與軌跡目標
│     ├─ doghead_hil/               telemetry 驗證與 fault scenarios
│     └─ doghead_bringup/           launch 與 YAML 設定
├─ Firmware/doghead_arm/
│  ├─ Core/Src/freertos.c           task 建立與週期
│  ├─ Application/                  控制、安全、運動邏輯
│  └─ Middleware/MicroROS/          micro-ROS entity 與 transport
└─ docs/                            工程文件與本教材
```

不要把 `Software/ros2_ws/build`、`install`、`log` 當原始碼。它們分別是編譯暫存、安裝空間與日誌，可由 `colcon build` 重新產生。

## 前置知識的最小集合

你不必先精通 Linux，但應熟悉：

```bash
pwd                 # 現在在哪裡
ls                  # 列檔案
cd path             # 切換目錄
env | sort          # 查看環境變數
ps aux              # 查看程序
```

ROS 2 Python 程式需要懂 class、callback 與 list；FreeRTOS 程式需要懂 C/C++ pointer、`volatile`、中斷與 race condition。遇到不懂的 API，先問三件事：誰呼叫、在哪個 context 呼叫、是否可能阻塞。

## 建議的學習節奏

每章採相同循環：

1. 先畫資料流，不急著改 code。
2. 用 CLI 觀察現況。
3. 只做不驅動馬達的實驗。
4. 刻意製造一個可復原錯誤。
5. 從 log 與 counter 解釋結果。

## 環境自我檢查

在 PowerShell：

```powershell
docker version
docker compose version
Get-ChildItem .\Software\ros2_ws\src
```

進入 ROS 開發容器後：

```bash
printenv ROS_DISTRO
ros2 --help
python3 --version
colcon --help
```

`ROS_DISTRO` 預期為 `humble`。若 `ros2` 找不到，執行：

```bash
source /opt/ros/humble/setup.bash
```

`source` 是把腳本對「目前這個 shell」所做的環境變數修改載入進來；關掉 shell 後就消失，因此每個新 shell 都要做。

## 本章檢查題

1. `docker compose up`、`colcon build`、`Robot_ControlTask_Tick()` 分別在哪一個世界執行？
2. 為什麼你在終端 A source workspace 後，終端 B 還是找不到 package？
3. `install/` 被刪除後，真正的原始碼是否消失？為什麼？

