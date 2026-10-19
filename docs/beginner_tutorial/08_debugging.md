# 08：除錯、測試與常見陷阱

## 核心方法：找最後一個成功的邊界

```text
source/build -> node -> endpoint/QoS -> Agent session -> TCP/PTY
 -> Windows COM -> UART -> CommTask -> mailbox -> ControlTask -> gate -> motor
```

每層先問：輸入有沒有到、輸出有沒有出、error counter 是否增加。

## 症狀導向診斷

### `ros2 run` 找不到 package/executable

```bash
echo $AMENT_PREFIX_PATH
ros2 pkg prefix doghead_driver
ls install/doghead_driver/lib/doghead_driver
```

常見原因：未 build、未 source overlay、build 用不同 Python/ROS distro、entry point 改後沒 rebuild。

### node 存在但 topic 沒資料

```bash
ros2 node info /doghead_arm_node
ros2 topic info /doghead/mcu/state --verbose
ros2 topic echo /doghead/mcu/state --qos-reliability best_effort
```

publisher 為 0 時往 entity/Agent 查；publisher 有但無 data，查 QoS、publish return、transport bandwidth。

### Agent 一直 ping/reconnect

檢查雙方 460800 8N1、COM 是否被占用、bridge TCP、UART ORE/NE/FE、DMA overflow、TX timeout、entity failed stage、CommTask stack/heap。亂碼通常是 baud/framing/clock；完全沒 byte 多半是 port/pin/DMA/IRQ/bridge。

### telemetry rate 不穩

`ros2 topic hz` 測 host 到達率，不是 MCU 精確週期。分解 ControlTask deadline、CommTask starvation、UART 飽和、Reliable 重送、DDS/container scheduling；同時看 MCU cycle、transport counter 與 ROS arrival stats。

### arm 失敗

看 `/doghead/driver/status` 的 AgentConnected、fault、boot/reconnect、protocol。不要移除 `if not agent_connected`；用受控 mock 或修真正連線。

### command 被拒絕

依 validator 順序查：armed/session → protocol → sequence → TTL → array length → finite → envelope。保存完整 reason。

### build 成功但 HardFault

編譯成功不代表 runtime 正確。查 task stack、對齊、generated sequence/string 初始化、micro-ROS pool、ISR priority、scheduler 前呼叫 RTOS API、DMA buffer lifetime。保存 CFSR/HFSR/BFAR/MMFAR，不要只停在 handler。

## 測試金字塔

| 層 | 本專案例子 | 能證明 |
|---|---|---|
| 純函式 unit | kinematics、validator、trajectory | 數學與規則 |
| host package | pytest、launch config | package 整合 |
| firmware host logic | robot_interface tests | session/TTL/mailbox |
| shadow-mode HIL | Agent+MCU+ROS | transport 與時序 |
| powered test | 單軸低速 | 電氣/機械/實際控制 |

## 每次實驗的證據模板

```text
版本：source revision / firmware build / protocol
環境：ROS_DISTRO / image / COM / baud
設定：shadow / actuator / compile-time gate
步驟：確切命令
預期：topic / rate / state / counter
實際：log 與 CLI output
安全：馬達電源、E-stop
結論：通過/失敗、下一個邊界
```

## 常見陷阱

1. 有 heartbeat 不代表所有 type/subscription 都正常。
2. Reliable 可能以 latency 換完整，不一定適合高頻 sensor。
3. priority 越高不代表 WCET 越短，還可能餓死通訊。
4. mutex 用錯 context、鎖太久仍會出問題。
5. arm 不等於 enable；本案是多層 AND gate。
6. YAML 有值不等於 node 已使用。
7. ROS time 不等於 MCU time。
8. 50 Hz telemetry 不能證明 1 kHz deadline。

## 建議改善（不是目前既有功能）

- 將 YAML limits/max TTL 注入 `DriverSafetyState` 並加測試。
- 避免 `/joint_states` 雙 publisher，或 remap raw/filtered state。
- 為 motion 建 timer-based trajectory stream 與 preemption。
- 文件化 boot ID、WCET、stack/heap、UART counters 的來源與單位。
- 建固定版本含 socat 的 Agent image，不在 startup `apt-get`。
- 自動收集 rosbag、diagnostics 與 fault test evidence。

