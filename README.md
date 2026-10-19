# DogHead 書法機械手臂韌體專案

DogHead 是以 STM32F446RE、FreeRTOS 與 micro-ROS 建構的三軸書法機械手臂
控制平台。上位機軟體以 ROS 2 Humble 容器為正式執行環境；目前已完成僅遙測與
輸出關閉的命令影子 HIL，尚未授權實體馬達運轉。

## 現在做到哪裡

| 項目 | 狀態 |
|---|---|
| 韌體與 ROS 訊息協定 | 完成並可重複建置 |
| micro-ROS Agent / USART2 transport | 實機連線完成，460800 8N1 |
| 50 Hz telemetry | 通過，約 49.2–49.3 Hz |
| ROS 2 Humble 測試 | 5 個套件，31 個測試結果，0 個失敗 |
| Session／sequence／TTL／版本驗證 | 實機命令影子流程通過 |
| Agent disconnect/reconnect fail-closed | 通過；舊 session 不會復活 |
| 實體馬達控制 | 尚未授權，硬體 gate 維持 0 |

Nidec `24H702U030` 的 13-pin 馬達規格書已補足 STMP 頻率輸入、CW/CCW、Encoder
輸出與 24 V／5 V 電源基線；三軸 encoder PPR、減速比、方向與機械零點也已依原始
韌體設定由使用者確認。Phase 3 前仍需確認有效頻率範圍、soft limits、E-Stop、
driver fault output 與 powered-load 測試條件。5 V 馬達邏輯與 STM32 3.3 V GPIO、
以及 5 V Encoder 與 STM32 TIM encoder 的相容性，均已由使用者實測確認。
目前馬達軸命令上限設定為 `5000 RPM`，第一次帶電 bring-up 仍限制為 `150 RPM`；
Joint 初始保守限制為 `10°/s` 與 `20°/s²`。依目前硬體限制，停止方式是將速度命令
設為 `0 Hz`；這不是獨立硬體 E-Stop，現場供電、負載與測試程序由使用者自行管理。
目前 `START` 已由硬體固定為 5 V、停用 MCU 控制；因此 E-Stop 必須走切斷 `VM` 或
其他獨立硬體停機路徑，不能依賴韌體把 `START` 拉低。

## 系統輪廓

```text
ROS 2 Humble container
  doghead_interfaces / doghead_driver / doghead_motion / doghead_hil / doghead_bringup
                   │ DDS
micro-ROS Agent container
                   │ socat pseudo-TTY / TCP 8888
Windows pyserial bridge
                   │ COM6, USART2 460800 8N1
STM32F446RE + FreeRTOS
  CommTask: micro-ROS state machine and 50 Hz telemetry
  ControlTask: 1 kHz control path and ultimate actuator gate
```

高頻 `/doghead/mcu/state` 與 `/joint_states` 使用 Best-Effort QoS，避免 UART
reliable ACK 拉長週期；diagnostics、command 與 `McuSessionControl` 使用 Reliable。

## 安全設計

- Firmware compile-time actuator gate 預設為 0。
- Driver 必須同時滿足 armed、非 shadow mode、actuator enabled 才可能要求輸出。
- MCU 未收到有效、非零的 session control 時，所有 command 都 fail-closed。
- protocol、session、sequence、TTL、有限值、axis/mask 與 joint envelope 採整包驗證。
- disarm、初始化失敗、entity destroy、Agent disconnect 都會清除 session 與 mailbox。
- firmware motor-driver boundary 再執行一次 ultimate safety gate。

## 初學者建議閱讀順序

1. [完整學習筆記](docs/project_management/05_motion_control_hil_learning_report.md)：
   ROS 2、FreeRTOS、micro-ROS、QoS、session 與除錯方法。
2. [HIL 執行計畫](docs/project_management/04_motion_control_hil_execution_plan.md)：
   interface contract、phase gate 與 acceptance criteria。
3. [ROS 2 package 架構](docs/ros2_integration/05_ros2_hil_packages_architecture.md)。
4. [micro-ROS 狀態機](docs/ros2_integration/01_microros_architecture_and_state_machine.md)。
5. [Windows VCP / Agent 操作](docs/ros2_integration/02_wsl2_usb_and_agent_setup_guide.md)。
6. [static library 產生流程](docs/ros2_integration/micro_ros_header_generation.md)。

早期 audit 文件保留歷史決策，若內容與上述文件衝突，以 HIL 執行計畫與學習筆記
的 2026-07-31 狀態為準。

## ROS 2 Humble clean build

正式驗證應在 `osrf/ros:humble-desktop-full` 或等價的 Humble container 執行：

```bash
cd Software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
colcon test
colcon test-result --all --verbose
```

本機 workspace 若含其他環境產生的 `build/`、`install/`、`log/`，建議使用乾淨
container workspace，避免把 ABI 或 Python environment 問題誤認成 source bug。

## 啟動 Windows VCP bridge 與 Agent

目前 NUCLEO 的 ST-Link VCP 在 Windows 為 COM6：

```powershell
.\scripts\start_vcp_bridge.ps1 -ComPort COM6 -BaudRate 460800 -TcpPort 8888
```

Bridge 模式下不要把同一個 ST-Link VCP attach 給 WSL2。若 COM port 或 board
編號不同，請依本機實際值修改參數。

## Firmware build 與 flash

可用 STM32CubeIDE 匯入 `Firmware/doghead_arm`；自動化流程也可直接使用
CubeIDE bundled GNU Arm toolchain 建置 `Firmware/doghead_arm/Debug`，再由
STM32CubeProgrammer 透過 SWD 寫入 `.elf`。

建置成功不等於 HIL 成功。燒錄後仍應確認：

- `/doghead/mcu/state` 與 diagnostics 可讀；
- `agent_connected=true`；
- UART/DMA/transport error counters 未增加；
- `enable_mask=0`、RPM/PWM command 為 0；
- CFSR 無 HardFault 記錄。

## 主要目錄

```text
Firmware/doghead_arm/                 STM32 + FreeRTOS + micro-ROS firmware
Software/ros2_ws/src/doghead_interfaces  ROS interface definitions
Software/ros2_ws/src/doghead_driver      host safety/session driver
Software/ros2_ws/src/doghead_motion      FK/IK and motion generation
Software/ros2_ws/src/doghead_hil         telemetry and fault scenarios
Software/ros2_ws/src/doghead_bringup     launch/configuration package
scripts/                                  Windows VCP bridge utilities
docs/                                     architecture, audit and tutorials
```

## 目前禁止事項

- 未確認硬體規格前，不得把 actuator hardware gate 改為 1。
- 不得把 command 被接受誤解為允許馬達運轉。
- 不得用 ROS 2/Linux 排程直接取代 MCU 1 kHz 閉迴路控制。
- 不得跳過 session、TTL、sequence 或 firmware hard limits。
