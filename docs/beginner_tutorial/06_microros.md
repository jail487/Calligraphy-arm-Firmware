# 06：micro-ROS 與跨系統通訊

## 為什麼 STM32 不直接跑完整 ROS 2

完整 ROS 2/DDS 通常假設較多 RAM、動態記憶體、POSIX 與網路能力。micro-ROS 在 MCU 上提供 `rcl/rclc` API，底層使用 Micro XRCE-DDS client；Agent 在 Linux 端把 XRCE entities/data 橋接到一般 DDS graph。

```text
MCU application
  rclc publisher/subscription/executor
        |
  rmw_microxrcedds client
        |
  custom UART transport
======== serial/TCP boundary ========
  micro_ros_agent
        |
  ROS 2 DDS graph
```

Agent 不是單純文字 UART parser。它代表 MCU 參與 DDS world，建立 publisher/subscriber entities 並做序列化傳輸。

## 本專案的 transport

STM32 使用 USART2 460800 8N1。`MicroROS_Init()` 呼叫：

```c
rmw_uros_set_custom_transport(
    true,
    &huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);
```

`true` 表示 framing transport：XRCE 層在 byte stream 上處理 frame 邊界。open/close/read/write 是 porting boundary；Agent 或 UART 問題先判斷是 transport 沒 byte、XRCE session 沒成立，還是 ROS entity 建立失敗。

460800 8N1 每 byte 約需要 10 bits（start + 8 data + stop），理論上限約 46,080 bytes/s，實際還要扣 XRCE、CDR、message 與重試 overhead。50 Hz telemetry payload 若持續增大，必須做頻寬預算。

## micro-ROS state machine

`microros_app.c` 有四個狀態：

```text
WAIT_AGENT
   | ping success
   v
CREATE_ENTITIES --failure--> WAIT_AGENT
   | success
   v
RUNNING --ping/spin failure--> DESTROY_ENTITIES
                                 |
                                 v
                              WAIT_AGENT
```

### WAIT_AGENT

用 `rmw_uros_ping_agent(100, 1)` 探測 Agent。ping success 只證明 transport/XRCE 可回應，尚未代表全部 topic entities 已建好。

### CREATE_ENTITIES

依序建立 support、node、heartbeat publisher、JointState publisher、DogHead publishers/subscriptions 與 executor。code 記錄 entity stage、last failed stage 與 return code，方便知道失敗在 type support、記憶體、session 還是哪個 endpoint。

### RUNNING

executor 以 bounded spin 處理 session control、joint setpoint、motor test callbacks；CommTask 另外按週期發布 state/diagnostics。micro-ROS callback 只轉為固定大小 `RobotCommand_t`，驗證後寫 mailbox，不執行 PID。

### DESTROY_ENTITIES

斷線時先把 session 設為 0 並 invalid command，再 fini executor、subscriptions、publishers、node 與 support。這防止 reconnect 後舊命令復活。Agent 斷線時 entity destroy timeout 設為 0，避免 cleanup 長時間等已不存在的 Agent。

## Entity 與 executor

- **node**：entity 的 namespace/owner。
- **publisher/subscription**：資料 endpoint。
- **executor**：等待輸入並派送 callback。
- **support/context**：RCL/RMW 初始化狀態。

建立順序與銷毀逆序很重要。部分建立失敗後若重複對未初始化 handle `fini`，可能 crash；本專案用 initialized flags 與 zero-initialized handles 追蹤。

## 自訂 interface 為何要進 firmware build

MCU 編譯時需要由 `.msg` 產生的 C struct、init/fini、type support 與 CDR serialization metadata。因此 `micro_ros_stm32cubemx_utils/.../extra_packages/doghead_interfaces` 保存生成靜態 library 時使用的 interface source，必須與 `Software/ros2_ws/src/doghead_interfaces` 同步。只複製 generated header 不是可靠流程。

## Message callback 到 ControlTask

以 joint setpoint 為例：

1. Agent 從 DDS 收 message，透過 XRCE/UART 傳 MCU。
2. rclc executor 呼叫 joint setpoint callback。
3. callback 填 `RobotCommand_t`，mode 設 `CONTROL_MODE_JOINT_PVA`。
4. firmware 驗 protocol/session/sequence/TTL/finite/limits。
5. `xQueueOverwrite()` 寫入長度 1 mailbox，記錄 received tick。
6. 下一次 ControlTask 取最新 command。
7. 若 age 大於 TTL，清除 command 並增加 timeout counter。

ROS timestamp 與 MCU tick 可能不同步，所以 TTL 在 MCU 以「本地收到的 monotonic tick」計算，避免 wall-clock synchronization 問題。

## QoS 與受限 transport

Reliable 不是永遠更好。高頻 sensor 用 Reliable，packet loss 時重送舊 frame，可能增加 latency；Best Effort 允許跳過舊 frame。arm/session control 低頻且重要，Reliable 合理。桌面 DDS 能設定的所有 history/depth，也不一定在 MCU build 中都有足夠 memory pool。

## Windows bridge 的除錯分層

由下往上查：

1. Board 是否供電、firmware 是否正在跑。
2. Device Manager 是否有正確 COM port，是否被其他程式占用。
3. PowerShell bridge 是否 open COM6 與 listen TCP 8888。
4. Container 是否能連 `host.docker.internal:8888`。
5. `socat` 是否建立 `/dev/ttySTM32`。
6. Agent log 是否顯示 client/session/entities。
7. `ros2 node list` 是否看到 `/doghead_arm_node`。
8. topic 是否有 endpoints 與 data。

不要在第 8 層沒資料時立刻改 firmware；先找最後一個成功的邊界。

## 本章檢查題

1. micro-ROS Agent 與一般 serial-to-TCP bridge 的責任差在哪裡？
2. 為何斷線時要先清 session/mailbox，再重建 entities？
3. TTL 為何使用 MCU received tick 而不是 host wall clock？

