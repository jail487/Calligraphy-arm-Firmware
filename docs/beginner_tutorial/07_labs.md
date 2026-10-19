# 07：安全的端到端實驗

本章從完全不需硬體開始。除非另有說明，保持馬達電源斷開、host `shadow_mode=true`、`actuator_enabled=false`、firmware `ACTUATOR_HARDWARE_OUTPUT_ENABLED=0`。

## Lab 0：建立乾淨 workspace

PowerShell：

```powershell
docker compose up -d ros2-dev
docker compose exec ros2-dev bash
```

container：

```bash
cd /workspace/Software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --all --verbose
```

通過標準：build 成功、test result 無 failure。若生成目錄由另一環境建立而出錯，先保存 log，再確認路徑後移除 `build/install/log` 並 clean build；不要刪 `src`。

## Lab 1：只啟動 host nodes

終端 A：

```bash
source /opt/ros/humble/setup.bash
source /workspace/Software/ros2_ws/install/setup.bash
ros2 launch doghead_bringup shadow_mode.launch.py
```

終端 B：

```bash
source /opt/ros/humble/setup.bash
source /workspace/Software/ros2_ws/install/setup.bash
ros2 node list
ros2 topic list -t
ros2 service list -t
ros2 param get /doghead_driver_node shadow_mode
ros2 param get /doghead_driver_node actuator_enabled
```

預期 nodes：driver、motion、telemetry validator；兩個參數預期為 true/false。

## Lab 2：驗證「未連 MCU 不能 arm」

```bash
ros2 service call /doghead/driver/arm std_srvs/srv/Trigger '{}'
ros2 topic echo /doghead/driver/status
```

沒有有效 McuState 時，預期 arm 失敗。再送一個合法格式但無 session 的 setpoint：

```bash
ros2 topic pub --once /doghead/command/joint_setpoint \
  doghead_interfaces/msg/McuJointSetpoint \
  "{protocol_version: 1, session_id: 0, sequence: 1, valid_for_ms: 250, enable_mask: 0, position_rad: [2.0, 1.0, 0.0], velocity_rad_s: [0.0, 0.0, 0.0], acceleration_rad_s2: [0.0, 0.0, 0.0]}"
```

預期 driver 拒絕 DISARMED command。

## Lab 3：單元測試看懂安全規則

```bash
cd /workspace/Software/ros2_ws
pytest -q src/doghead_driver/test/test_safety_validator.py
pytest -q src/doghead_motion/test/test_kinematics.py
pytest -q src/doghead_motion/test/test_trajectory.py
```

用 `pytest -vv` 閱讀 case 名稱，把 arrange/act/assert 對回 production function。安全測試應涵蓋 NaN、Inf、TTL、stale sequence、session mismatch 與 limit。

## Lab 4：IK 正例與反例

先監看：

```bash
ros2 topic echo /doghead/motion/status
```

發布目標：

```bash
ros2 topic pub --once /doghead/motion/ef_reference \
  doghead_interfaces/msg/EndEffectorReference \
  "{mode: 0, position: {x: -0.20, y: 0.20, z: 0.05}, linear_velocity: {x: 0.0, y: 0.0, z: 0.0}, linear_acceleration: {x: 0.0, y: 0.0, z: 0.0}}"
```

依 limits，結果可能接受或以明確原因拒絕。再送明顯超出 `L1+L2=0.38 m` 的點：

```bash
ros2 topic pub --once /doghead/motion/ef_reference \
  doghead_interfaces/msg/EndEffectorReference \
  "{mode: 0, position: {x: 2.0, y: 0.0, z: 2.0}}"
```

預期 `Unreachable`。未 armed 時，即使 IK accepted，setpoint 仍為 session 0、mask 0。

## Lab 5：接上 MCU telemetry，但仍不驅動

PowerShell A：

```powershell
.\scripts\start_vcp_bridge.ps1 -ComPort COM6 -BaudRate 460800 -TcpPort 8888
```

PowerShell B：

```powershell
docker compose --profile compose-bridge-agent up -d microros-agent-bridge
docker compose logs -f microros-agent-bridge
```

ROS container：

```bash
ros2 node list
ros2 topic info /doghead/mcu/state --verbose
ros2 topic hz /doghead/mcu/state
ros2 topic echo /doghead/mcu/diagnostics --once
```

通過標準：Agent 建立 entities、state 接近 50 Hz、diagnostics 約 1 Hz、錯誤 counters 不持續成長、enable/PWM/RPM 保持安全值。

## Lab 6：Session handoff，仍保持 shadow mode

確認硬體輸出 gate 關閉後：

```bash
ros2 service call /doghead/driver/arm std_srvs/srv/Trigger '{}'
ros2 topic echo /doghead/driver/session_state
```

觀察 session 從 0 變非 0；driver armed，但 shadow true、actuator false，轉發給 MCU 的 mask 應為 0。再 disarm：

```bash
ros2 service call /doghead/driver/disarm std_srvs/srv/Trigger '{}'
```

session 應歸零；再次 arm 的 ID 應不同。

## Lab 7：Fault injection

```bash
ros2 launch doghead_bringup hil_test.launch.py
```

預設是 `telemetry_only`。要測其他 scenario，先停止原 runner，再啟動：

```bash
ros2 run doghead_hil scenario_runner --ros-args \
  -p scenario_name:=stale_command \
  -p actuator_enabled:=false
```

同理測 `nan_injection`、`command_timeout`。每次先 disarm/re-arm 取得乾淨 session，記錄輸入、預期拒絕層、log 與 diagnostic counter。

## Lab 8：斷線與 reconnect

保持 shadow mode，先 arm，停止 bridge 或 Agent。預期 driver disarm、MCU session 清零、mailbox invalidated；Agent 恢復後不可自動重用舊 session，必須重新 arm。

## 實體輸出前的獨立審查

這不屬於新手 Lab。至少需要實體 E-stop、可斷電驅動回路、卸載測試、方向與 encoder sign 驗證、limit switch、單軸低速、current limit、第二人覆核與 gate 變更記錄。不可只把 YAML 改成 false/true 就上電。

