# 03：ROS 2 核心觀念與 CLI

## ROS 2 是什麼

ROS 2 是機器人軟體的通訊與工具生態，不是作業系統，也不是馬達控制演算法。節點透過 middleware（本專案通常是 DDS）交換有型別的資料。節點不必知道對方的 IP 或 process ID，只需對 topic 名稱、type 與 QoS 達成契約。

## Node、topic、service、parameter

| 機制 | 模型 | 適合 | 本專案例子 |
|---|---|---|---|
| topic | 非同步資料流 | sensor、state、持續 command | `/doghead/mcu/state` |
| service | request/response | 短時間控制動作 | `/doghead/driver/arm` |
| parameter | node 設定 | 啟動或低頻設定 | `shadow_mode` |
| action | 可回饋、可取消的長任務 | 導航、長軌跡 | 本專案目前未使用 |

不要用 service 傳 50 Hz sensor stream；不要假設 topic publisher 知道 subscriber 是否處理成功。topic 是解耦資料流。

## 用 CLI 探索，不靠猜

啟動 shadow-mode nodes 後：

```bash
ros2 launch doghead_bringup shadow_mode.launch.py
```

另一個已 source 的 shell：

```bash
ros2 node list
ros2 node info /doghead_driver_node
ros2 topic list -t
ros2 service list -t
ros2 param list /doghead_driver_node
```

深入 topic：

```bash
ros2 topic info /doghead/mcu/state --verbose
ros2 interface show doghead_interfaces/msg/McuState
ros2 topic echo /doghead/driver/status
ros2 topic hz /doghead/mcu/state
ros2 topic bw /doghead/mcu/state
```

若沒有 MCU publisher，`hz` 會等待，這不代表 CLI 壞掉。

## Message type 與 IDL

`McuJointSetpoint.msg` 是跨語言契約：

```text
uint16 protocol_version
uint32 session_id
uint32 sequence
uint16 valid_for_ms
uint8 enable_mask
float32[3] position_rad
float32[3] velocity_rad_s
float32[3] acceleration_rad_s2
```

`rosidl_generate_interfaces()` 會為 Python、C、C++ 與 type support 產生程式。這就是同一個 `.msg` 能被 Python driver 與 STM32 C code 使用的原因。

修改 message 後必須：

1. 同步修改 canonical ROS package 與 micro-ROS static library 內的 interface source。
2. 重建 ROS workspace。
3. 重新產生/重建 micro-ROS 靜態函式庫與 firmware。
4. 將 `protocol_version` 視相容性需求調整。

只改 host 的 `.msg` 而不更新 MCU，可能編譯都成功但 wire layout 不一致。

## Publisher 與 subscription callback

Python node 的典型生命週期：

```python
rclpy.init()
node = DogheadDriverNode()
rclpy.spin(node)       # executor 等待並派送 callback
node.destroy_node()
rclpy.shutdown()
```

`create_subscription(..., callback, qos)` 註冊 callback；message 到達後由 executor 呼叫。callback 應短小、避免長時間 blocking。ROS executor 的 callback 排程不是 FreeRTOS 1 kHz deadline 保證。

本專案 driver 在 `_on_mcu_state()`：

1. 更新 safety state。
2. 檢查 telemetry 是否皆為 finite。
3. 轉成 `sensor_msgs/JointState`。
4. 發布 `/joint_states`。

## QoS：不只是 queue depth

DDS QoS 描述傳輸行為。最重要的初學者屬性：

- **Reliability**：Reliable 會重試與確認；Best Effort 可以掉包。
- **History/depth**：保留最近幾筆。
- **Durability**：晚加入者是否取得舊資料。
- **Deadline/lifespan**：期望週期與資料壽命；本專案 TTL 主要是應用層欄位，不等同 DDS lifespan。

本專案用 Best Effort 傳 50 Hz `McuState`，因為舊 sensor frame 不值得阻塞 UART 等待 ACK；下一 frame 很快會來。session control 與 command 則用 Reliable，因為少量控制訊息遺失較重要。

Publisher 與 subscriber 的 QoS 必須相容。常見症狀是「topic 名稱與 type 都對，但 echo 沒資料」。使用：

```bash
ros2 topic info /doghead/mcu/state --verbose
```

查看 endpoint QoS。

## Service：arm 不是馬達立即轉動

```bash
ros2 service call /doghead/driver/arm std_srvs/srv/Trigger '{}'
ros2 service call /doghead/driver/disarm std_srvs/srv/Trigger '{}'
ros2 service call /doghead/driver/stop std_srvs/srv/Trigger '{}'
```

`arm` 只有在 driver 已從 McuState 確認 Agent connected 時才成功，並建立新的 session ID。它仍不會越過 `shadow_mode`、`actuator_enabled` 與 firmware gate。

## Parameter 與 YAML

程式先 `declare_parameter()`，launch 再由 `safety_limits.yaml` 提供值。查看實際值：

```bash
ros2 param get /doghead_driver_node shadow_mode
ros2 param get /doghead_driver_node actuator_enabled
ros2 param dump /doghead_driver_node
```

注意：程式只會讀取它實際宣告/取得的 key。YAML 中有 `limits.*` 不代表 Python driver 一定使用那些值；目前 `driver_node.py` 建立 `DriverSafetyState(DEFAULT_LIMITS)`，並未把 YAML 的 `limits` 傳入。這是閱讀 ROS parameter 時很重要的原則：**以程式實際 get 的值為準，而不是以 YAML 中存在為準**。

## Launch

launch file 是「建立哪些 process、傳哪些 parameter、如何輸出 log」的 Python 描述。`shadow_mode.launch.py` 同時啟動 driver、motion 與 telemetry validator。launch 不會自動啟動 Docker 或燒錄 firmware。

```bash
ros2 launch doghead_bringup shadow_mode.launch.py
ros2 launch doghead_bringup hil_test.launch.py
```

後者多啟動 scenario runner，預設仍是 `telemetry_only` 且 actuator disabled。

## 發布安全的測試資料

在沒有 arm session 時，直接 command 會被拒絕；可用它觀察安全 log：

```bash
ros2 topic pub --once /doghead/command/joint_setpoint \
  doghead_interfaces/msg/McuJointSetpoint \
  "{protocol_version: 1, session_id: 0, sequence: 1, valid_for_ms: 250, enable_mask: 0, position_rad: [2.0, 1.0, 0.0], velocity_rad_s: [0.0, 0.0, 0.0], acceleration_rad_s2: [0.0, 0.0, 0.0]}"
```

預期 driver 顯示 DISARMED 拒絕，而不是轉發。這是一個好的 negative test。

## 本章檢查題

1. Best Effort 適合 McuState 的原因是什麼？
2. `valid_for_ms` 和 DDS lifespan 有什麼不同？
3. YAML 中出現的值為何不一定已被 node 使用？

