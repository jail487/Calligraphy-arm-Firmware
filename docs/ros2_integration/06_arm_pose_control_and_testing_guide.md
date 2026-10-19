# DogHead 機械手臂姿態控制與測試操作指南

本文檔提供 DogHead 書法機械手臂完整的**上位機連線建立**、**平行四邊形雙軸校準**、**三軸運動學姿態控制**與**故障排除**操作指南。

---

## 1. 系統通訊架構

```text
[ Windows 實體主機 ]
   STM32F446RE (ST-Link USB VCP 460800 8N1)
       │
       ▼
   python .\scripts\win_vcp_bridge.py  ──(TCP 8888 本地轉接)──┐
                                                               │
[ Docker 容器環境 ]                                            │
   microros-agent-bridge (micro-ROS Agent + socat PTY) ◀───────┘
       │ DDS 主題傳輸
       ▼
   ros2-dev 容器 (arm_calibration_tuner.py / kinematic_feedback_control.py)
```

---

## 2. 測試連線標準三步驟（請開啟 3 個終端機視窗）

### 步驟 1：啟動 Windows VCP 串口轉接橋（終端 1：Windows PowerShell）

在專案根目錄直接執行 Python 橋接器：

```powershell
python .\scripts\win_vcp_bridge.py
```

> [!TIP]
> - 腳本具備**自動掃描**功能，會自動尋找 ST-Link 虛擬串口並在 `8888` 埠口啟動 TCP 服務。
> - 若欲手動指定串口，可加入參數：`python .\scripts\win_vcp_bridge.py -c COM6 -b 460800 -p 8888`。
> - **為何直接跑 Python**：Windows 系統預設會阻擋 `.ps1` 腳本執行，直接執行 Python 本體可避免權限錯誤。

---

### 步驟 2：啟動 micro-ROS Agent 容器（終端 2：Windows PowerShell）

開啟第 2 個 PowerShell 視窗，啟動 micro-ROS 代理容器：

```powershell
docker compose --profile compose-bridge-agent up -d microros-agent-bridge
docker compose logs -f microros-agent-bridge
```

> [!NOTE]
> 當畫面出現 `Session established` 或沒有紅色報錯時，即代表 STM32 韌體與 Docker micro-ROS Agent 已建立穩定的 DDS 通訊鏈路。

---

### 步驟 3：進入 ROS 2 容器並執行測試腳本（終端 3：Windows PowerShell）

開啟第 3 個 PowerShell 視窗，進入 ROS 2 開發容器：

```powershell
docker compose exec ros2-dev bash
```

進入容器後，先載入 ROS 2 環境變數：

```bash
source /opt/ros/humble/setup.bash
source /workspace/Software/ros2_ws/install/setup.bash
```

接著即可依據您的測試目的選擇對應的控制腳本（見下節）。

---

## 3. 核心控制與測試腳本使用說明

所有控制腳本皆位於 `/workspace/Software/ros2_ws/`。

### 🎮 腳本 A：平行四邊形雙軸 (X, Z) 幾何校準主控台（推薦校準使用）

專門用於 2-DOF 平行四邊形連桿幾何調諧、零點偏移補償與直角座標閉環控制。

```bash
python3 /workspace/Software/ros2_ws/arm_calibration_tuner.py
```

#### 選單功能對照表：
| 選號 | 功能名稱 | 說明 |
|---|---|---|
| `[0]` | 🚀 快速前往書寫預備區 | 將末端平滑移至前方的基準位置（$X=15.0\text{ cm}, Z=5.0\text{ cm}$）。 |
| `[1]` | 🕹️ 末端直角座標 Jog 微調 | 進入 2D 直角座標微調模式（支援鍵盤快捷鍵）。 |
| `[2]` | 測試 A：水平等高平移測試 | 末端沿 $X$ 軸走直線（$12\text{ cm} \rightarrow 18\text{ cm}$），維持 $Z=5\text{ cm}$ 高度不變。 |
| `[3]` | 測試 B：垂直升降直線測試 | 末端沿 $Z$ 軸走垂直線（$2\text{ cm} \rightarrow 8\text{ cm}$），維持 $X=15\text{ cm}$ 前後距離不變。 |
| `[4]` | 微調 J1 (大臂) 零點 Offset | 補償大臂安裝的微小初始角度偏差（步進 0.1° / 0.5°）。 |
| `[5]` | 微調 J2 (小臂) 零點 Offset | 補償小臂安裝的微小初始角度偏差（步進 0.1° / 0.5°）。 |
| `[6]` | 微調連桿幾何長度 | 微調大臂長度 $R1$ 與小臂長度 $R2$（預設皆為 $113.9\text{ mm}$）。 |
| `[7]` | 回到標準 Home 姿態 | 將關節角重置至基準姿態（$J1=111^\circ, J2=180^\circ$）。 |
| `[8]` | 調整最大移動速度上限 | 調整關節最高轉速（度/秒），保障調校安全。 |
| `[9]` | ⚡ 馬達轉速直接測試 | 直接以低轉速（$5 \sim 30\text{ RPM}$）點動單顆馬達。 |
| `[q]` | 離開程式 | 安全停機並退出。 |

#### `[1]` 模式鍵盤微調指令：
- `w`：末端向前（$+X$）伸長 1 cm
- `s`：末端向後（$-X$）縮回 1 cm
- `r`：末端向上（$+Z$）抬高 1 cm
- `f`：末端向下（$-Z$）壓低 1 cm
- `w 2` / `f 0.5`：自訂單次步長（如向前 2 cm、向下 0.5 cm）
- `goto 16 6`：指定直達絕對座標（$X=16\text{ cm}, Z=6\text{ cm}$）
- `b` 或 `q`：返回主選單

---

### 🦾 腳本 B：全手臂三軸閉環空間姿態主控台

當底座旋轉軸（J3）連線就緒，欲進行完整三維空間 $(X, Y, Z)$ 姿態控制時使用：

```bash
python3 /workspace/Software/ros2_ws/kinematic_feedback_control.py
```

- **Joint PTP 模式**：分別指定 J1（大臂）、J2（小臂）、J3（基座）的目標角度。
- **Cartesian PTP 模式**：輸入空間座標 $(X, Y, Z)$，系統自動使用五次多項式平滑曲線（Quintic PVA）規劃軌跡並平順運轉。

---

### ⚙️ 腳本 C：多軸與單軸輔助測試工具

- **三軸同步轉速旋轉測試**（可指定目標馬達 RPM）：
  ```bash
  python3 /workspace/Software/ros2_ws/spin_three_motors.py 15.0
  ```
- **單軸轉速點動測試**（語法：`python3 doghead_jog.py <軸 0..2> <RPM>`）：
  ```bash
  python3 /workspace/Software/ros2_ws/doghead_jog.py 0 50.0
  ```
- **手動編碼器讀數與轉向驗證**：
  ```bash
  python3 /workspace/Software/ros2_ws/verify_encoder_feedback.py
  ```

---

## 4. 常見問題與排除（FAQ）

### Q1：執行 `.ps1` 提示「因為這個系統上已停用指令碼執行...」
- **原因**：Windows 預設 Execution Policy 限制。
- **解決方法**：直接改跑 Python 指令：
  ```powershell
  python .\scripts\win_vcp_bridge.py
  ```
  若仍想跑 PowerShell 腳本，指令前需加上 Bypass 參數：
  ```powershell
  powershell -ExecutionPolicy Bypass -File .\scripts\start_vcp_bridge.ps1 -ComPort COM6
  ```

### Q2：`win_vcp_bridge.py` 提示 `error: argument -p/--port: invalid int value: 'COM6'`
- **原因**：參數輸入錯誤。在 `win_vcp_bridge.py` 中：
  - `-c` / `--com` 代表串口（例如 `COM6`）
  - `-p` / `--port` 代表 TCP Port（例如 `8888`）
- **解決方法**：直接執行 `python .\scripts\win_vcp_bridge.py`（讓系統自動偵測 COM 埠）。

### Q3：Python 腳本卡在「等待 MCU 回授訊號中...」
1. 檢查 ST-Link USB 線是否插好。
2. 檢查終端 1（`win_vcp_bridge.py`）是否顯示 Connected。
3. 檢查終端 2（`docker compose logs -f microros-agent-bridge`）是否有持續傳輸封包。
4. 按一下 STM32 開發板上的 **黑色 RESET 按鈕**，橋接器會自動重新同步。

---
