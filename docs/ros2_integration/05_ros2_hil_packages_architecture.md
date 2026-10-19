# ROS 2 驅動、運動、HIL 與啟動套件架構

> **目前實作狀態：** 以 ROS 2 Humble 為目標；5 個套件、31 個測試結果皆通過。
> 已在硬體驗證可靠傳遞的 `/doghead/mcu/session_control`，以及「輸出關閉」的命令影子流程。

**文件類型：** 架構與介面規格  
**版本：** v1.1  
**日期：** 2026-07-31

---

## 1. 目的與安全邊界

本文件說明書法機械臂 `doghead_arm` 的最小 ROS 2 套件組合。它採取「遙測優先」策略：先確認 MCU 能穩定回報狀態，再驗證主機端的關節 PVA（位置、速度、加速度）命令資料流；目前驗證流程不會啟用馬達輸出。

所有主機端套件均遵守下列規則：

- MCU 協定具有版本欄位，現行 `protocol_version = 1`。
- 不提供直接 PWM、MCU 端笛卡兒座標 JSON，或繞過安全檢查的致動器路徑。
- Agent 重新連線、協定不相容或 MCU 故障時，必須解除武裝（disarm）。
- 馬達輸出的「雙鑰匙硬閘門」必須同時成立：`armed == True`、`shadow_mode == False`、`actuator_enabled == True`。
- 以版本化 session 交接：運動節點收到驅動端 session 變更後，若已解除武裝就回到 `session_id = 0`。

```text
Software/ros2_ws/src/
├── doghead_interfaces/  自訂 ROS 2 訊息介面
├── doghead_driver/      MCU 通訊、遙測轉換、武裝安全狀態機
├── doghead_motion/      SI 單位、三軸運動學、PVA 軌跡與 session 交接
├── doghead_hil/         遙測驗證、追蹤指標與 HIL 情境測試
└── doghead_bringup/     啟動檔、安全限制與 rosbag2 錄製設定
```

## 2. 套件職責

### 2.1 `doghead_driver`

節點：`doghead_driver_node`

- 訂閱 `/doghead/mcu/state`（`McuState`），轉換後以 50 Hz 發布 `/joint_states`。
- 訂閱 `/doghead/mcu/diagnostics`（`McuDiagnostics`），判斷重連、記憶體與健康狀態。
- 接收 `/doghead/command/joint_setpoint`（`McuJointSetpoint`）與 `/doghead/command/motor_test`（`McuMotorTestCommand`），通過檢查後才轉發給 MCU。
- 發布 `/doghead/mcu/joint_setpoint`、`/doghead/mcu/motor_test_command`、`/doghead/driver/status` 與 `/doghead/driver/session_state`。
- 以可靠 QoS 定期發布 `/doghead/mcu/session_control`（`McuSessionControl`）：武裝時綁定非零 session，解除武裝時清除 session。
- 服務 `/doghead/driver/arm` 會建立新的 session；`/doghead/driver/disarm` 與 `/doghead/driver/stop` 會立即使 session 失效。

驅動端會拒絕 NaN/Inf、錯誤陣列長度、超出三軸軟限制、非遞增序號與超過 `valid_for_ms` 的命令。發布 `sensor_msgs/JointState` 前，也會把 NumPy `float32` 明確轉為原生 Python `float`，避免 ROS Python 型別轉換問題。

### 2.2 `doghead_motion`

節點：`doghead_motion_node`

- `DogheadKinematics` 提供三軸關節式機械臂的正向／逆向運動學。尺寸為 $L_1=0.20$ m、$L_2=0.18$ m、TCP z 偏移 $L_3=0.10$ m。
- 工作空間距離限制為 $D \in [0.02, 0.38]$ m；目前確認的關節軟限制為 J1=`80–180°`、J2=`0–180°`、J3=`-180–180°`。
- 不可達目標與奇異點會被明確回報，不會產生猜測性的命令。
- 訂閱 `/doghead/driver/session_state`，維護 `active_session_id` 與 `driver_armed`。驅動端解除武裝、重連或協定不相容時，立即清為 `0` 與 `False`。
- 未武裝或 session 無效時，只輸出 `enable_mask = 0`、`session_id = 0` 的安全 setpoint。
- `JointPvaTrajectoryGenerator` 使用 S 曲線／五次插值，以符合速度與加速度限制。
- 訂閱 `/doghead/motion/ef_reference`（`EndEffectorReference`），唯一的 setpoint 輸出為 `/doghead/command/joint_setpoint`（`McuJointSetpoint`）。

### 2.3 `doghead_hil`

- `telemetry_validator_node` 監看 50 Hz 的 `/doghead/mcu/state`，計算到達時間抖動並偵測非有限數值。
- `hil_scenario_runner_node` 執行 `telemetry_only`、`command_timeout`、`stale_command`、`nan_injection` 等自動化安全情境。
- `TrackingMetricsAnalyzer` 計算位置 RMSE、最大絕對誤差、速度 RMSE、遙測抖動與命令時效，並把 Markdown 報告發布至 `/doghead/hil/telemetry_report`。
- 所有 HIL 情境預設 `actuator_enabled = False`，亦即 `enable_mask = 0`。

### 2.4 `doghead_bringup`

- `telemetry_only.launch.py`：影子模式的驅動端與遙測驗證器。
- `shadow_mode.launch.py`：影子模式驅動端、運動節點與遙測驗證器。
- `hil_test.launch.py`：再加上 HIL 情境執行器。
- `config/safety_limits.yaml` 固定安全限制與預設值：`shadow_mode: true`、`actuator_enabled: false`。
- `config/rosbag2_profile.yaml` 定義遙測、診斷與 setpoint topic 的錄製集合。

## 3. 驗證與測試

每個套件都有可重現的 `pytest` 單元測試：

- `doghead_driver/test/test_safety_validator.py`：含雙鑰匙硬閘門。
- `doghead_motion/test/test_kinematics.py` 與 `test_trajectory.py`：運動學與軌跡限制。
- `doghead_motion/test/test_motion_session_handoff.py`：session 交接與失效。
- `doghead_hil/test/test_tracking_metrics.py` 與 `test_scenarios.py`：指標與安全情境。
- `doghead_bringup/test/test_bringup_config.py`：啟動與安全設定。

完整的目前成果、容器指令與初學者說明，請從 [文件導覽](../README.md) 進入。
