# 01：系統架構與一筆命令的旅程

## 分工原則

書法手臂同時需要「容易開發的高階運算」與「固定時間內完成的底層控制」。本專案採兩級控制：

| 層級 | 適合負責 | 不適合負責 |
|---|---|---|
| ROS 2/Linux | IK、軌跡請求、狀態整合、HIL、記錄、視覺化 | 直接保證每 1 ms 準時更新 PWM |
| STM32/FreeRTOS | 1 kHz feedback/control、TTL、硬限位、最終輸出 gate | 大型 UI、複雜檔案與網路服務 |

Linux 即使平均很快，也可能因排程、page fault 或 I/O 暫停；MCU 的工作較單純、可控制記憶體與中斷，適合做最後安全決策。

## ROS 2 package 與 node

「package」是程式碼、相依套件與安裝規則的發布單位；「node」是執行中的 ROS 參與者。一個 package 可以提供多個 node。

| package | executable/node | 主要責任 |
|---|---|---|
| `doghead_interfaces` | 無常駐 node | 產生雙方共用的 message type |
| `doghead_motion` | `motion_node` | 末端目標、IK、PVA setpoint、session handoff |
| `doghead_driver` | `driver_node` | host 驗證、arm/disarm、轉發、`/joint_states` |
| `doghead_hil` | `telemetry_validator`、`scenario_runner` | rate/jitter 檢查與錯誤注入 |
| `doghead_bringup` | launch files | 同時啟動上述 nodes 並載入 YAML |

## 命令方向

```text
/doghead/motion/ef_reference
       |
       v
motion_node --IK/PVA--> /doghead/command/joint_setpoint
       |                              |
       | session_state                v
       +----------------------- driver_node --validate/gate-->
                                      /doghead/mcu/joint_setpoint
                                                  |
                                             micro-ROS
                                                  |
                                             STM32 mailbox
                                                  |
                                            ControlTask 1 kHz
```

回授方向：

```text
encoder -> STM32 control snapshot
        -> McuState (50 Hz)
        -> driver_node
        -> sensor_msgs/JointState
        -> /joint_states
```

韌體也直接發布 `/joint_states`。這代表同時啟用 firmware publisher 與 driver republisher 時，可能有兩個 publisher；除錯時務必用 `ros2 topic info /joint_states --verbose` 確認來源。這不是 topic 名稱衝突錯誤，但會讓訂閱者收到兩股資料。

## 為何要有 driver 中間層

如果 motion node 直接控制 MCU，任一上層 bug 都可能送出危險值。driver 像防火牆，依序檢查：

- 是否已 armed，session 是否相同。
- `protocol_version` 是否為 1。
- `sequence` 是否比上一筆新，並處理 32-bit wrap-around。
- `valid_for_ms` 是否在 `(0, 1000]`。
- 三個陣列是否各有 3 項，是否為有限數值而非 NaN/Inf。
- position、velocity、acceleration 是否在 envelope。
- host 端兩把鑰匙是否同時成立。

即使 host 驗證出錯，MCU 還會再驗一次，並由 `UltimateGate_ApplyActuatorCommand()` 做最後判斷。這稱為 defense in depth。

## Session、sequence 與 TTL

這三者解決不同問題：

| 欄位 | 防止什麼 | 例子 |
|---|---|---|
| session ID | 舊控制世代的命令復活 | 斷線再連後，舊 queue 中的命令不能沿用 |
| sequence | 重送、亂序 | 收到 10 後拒絕 5 |
| TTL | 新鮮度不足 | 控制串流停了 250 ms 後不能繼續使用最後一筆 |

它們不能互相取代。sequence 很新但可能已經延遲數秒；TTL 未過但可能屬於上一個 arm session。

## Fail-closed 狀態轉換

預設狀態是 disarmed。Agent 連線只代表可以要求 arm，不代表自動 arm。若發生 MCU reboot、Agent disconnect/reconnect、fault、NaN/Inf、timeout、session mismatch，系統會讓 session 歸零、清 mailbox、reset 插補/PID 狀態並停止輸出。

`enable_mask` 的 bit 0、1、2 對應三個 axis。實際啟用條件可近似寫成：

```text
axis_output =
  driver_armed
  AND NOT host_shadow_mode
  AND host_actuator_enabled
  AND firmware_hardware_output_enabled
  AND MCU_safety_can_drive
  AND NOT MCU_faulted
  AND command_axis_bit
```

任一項為 false 都應停機。

## 時間尺度

| 路徑 | 週期/限制 | 目的 |
|---|---|---|
| ControlTask | 1 ms / 1 kHz | feedback、插補、PID、安全與輸出 |
| CommTask loop | 5 ms delay | ping、executor 與 ROS transport work |
| McuState | 20 ms / 50 Hz | host 遙測 |
| McuDiagnostics | 1000 ms / 1 Hz | counter、heap、stack、transport health |
| 預設 command TTL | 250 ms | 上層停止更新後 fail-closed |
| Agent ping | 約 1 s | 監測 transport/session |

「1 kHz 控制」不是 ROS topic 必須 1 kHz。ROS 傳較低頻的 setpoint，MCU 在每個 tick 插補與控制，才能把網路 jitter 隔離在即時迴圈之外。

## 對照原始碼

- host 資料流：`doghead_motion/motion_node.py`、`doghead_driver/driver_node.py`。
- host 規則：`doghead_driver/safety_validator.py`。
- task：`Core/Src/freertos.c`。
- mailbox/session/TTL：`Application/Src/robot_interface.cpp`。
- 1 kHz 控制與 ultimate gate：`Application/Src/mainpp.cpp`。
- micro-ROS lifecycle：`Middleware/MicroROS/Src/microros_app.c`。

## 本章檢查題

1. Agent 已連線但 driver 未 armed 時，motion node 的 `session_id` 與 `enable_mask` 是多少？
2. 為何 command mailbox 長度 1 並使用 overwrite，反而適合 setpoint？
3. sequence 不能代替 TTL 的原因是什麼？

