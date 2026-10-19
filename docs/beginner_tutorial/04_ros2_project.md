# 04：逐檔閱讀本專案 ROS 2 程式

## Workspace 與 package 結構

`colcon` 從 workspace 的 `src/` 尋找 package。Python package 通常有 `package.xml`、`setup.py`、`setup.cfg`、resource marker 與 Python module；interface/bringup 使用 `ament_cmake`，有 `CMakeLists.txt`。

```text
src/doghead_driver/
├─ package.xml                  ROS metadata/dependencies
├─ setup.py                     Python 安裝與 console_scripts
├─ setup.cfg                    script 安裝位置
├─ resource/doghead_driver      ament index marker
└─ doghead_driver/
   ├─ driver_node.py
   └─ safety_validator.py
```

`setup.py` 的：

```python
'driver_node = doghead_driver.driver_node:main'
```

讓 `ros2 run doghead_driver driver_node` 找到 `main()`。

## `doghead_interfaces`

這個 package 沒有 node，只產生 types。`CMakeLists.txt` 的流程是：

1. 找 `ament_cmake`、`rosidl_default_generators`。
2. 列出七個 `.msg`。
3. 以 `rosidl_generate_interfaces()` 產生程式。
4. export runtime dependency。

幾個重要 message：

| message | 角色 |
|---|---|
| `EndEffectorReference` | 上層末端 XYZ 參考值，SI 單位 |
| `McuJointSetpoint` | 三軸 position/velocity/acceleration 與安全 metadata |
| `McuSessionControl` | host 告知 MCU 新 session 是否 armed |
| `McuState` | 50 Hz 控制與量測快照 |
| `McuDiagnostics` | 1 Hz deadline、UART、heap、stack counters |
| `DriverSessionState` | driver 對 motion 宣告 active session |

固定長度 `[3]` 可避免任意長度配置，對 MCU 記憶體可預測性有利。

## `doghead_motion`

### 資料進入

`/doghead/motion/ef_reference` callback 取出公尺制 `(x,y,z)`，呼叫 `DogheadKinematics.inverse_kinematics()`。IK 先檢查 finite、工作空間半徑、餘弦定理、兩種 elbow 解與 joint limits。

核心幾何：

```text
r = hypot(x, y)
z_eff = z + L3
D² = r² + z_eff²
cos(q3) = (D² - L1² - L2²) / (2 L1 L2)
q1 = atan2(y, x)
```

接近邊界時浮點誤差可能讓 `cos(q3)` 稍微超出 `[-1,1]`，因此 code 先 clamp 再 `sqrt`。這是數值程式的重要防護。

### Session handoff

motion 訂閱 `/doghead/driver/session_state`。未 armed 時仍可計算 IK，但輸出：

```text
session_id = 0
enable_mask = 0
```

armed 且 protocol 相符時，才沿用 driver session 並要求 `0x07`。這不代表馬達必然輸出；driver/firmware gate 仍會再次判斷。

### 一個值得注意的現況

`sample_trajectory(..., progress=1.0)` 目前只產生終點 sample，而不是持續 timer 驅動的完整軌跡串流。教材中的「trajectory」要理解為目前的 PVA endpoint setpoint 產生器；若要平滑 host-side streaming，未來需加入固定週期 timer、progress 與 preemption 設計。

## `doghead_driver`

driver 是 host 的 trust boundary。

### Telemetry callback

`_on_mcu_state()` 先讓 `DriverSafetyState.on_mcu_state()` 檢查 protocol、boot ID、Agent connection 與 fault。任何 telemetry vector 含 NaN/Inf 時立即 disarm 並丟棄，避免把無效值送入 `/joint_states`。

### Command callback

`_on_command_joint_setpoint()` 通過 validator 後，重新建一個 outgoing message，不盲目原樣轉發。最終 host gate：

```python
armed and (not shadow_mode) and actuator_enabled
```

不成立時強制 `enable_mask=0`。

### Safety state

每次成功 arm，`_session_epoch` 遞增；disarm 將 active `session_id` 清為 0，但不把 epoch 歸零，因此下一次不會重用 session 1。sequence 使用 modular comparison：

```python
diff = (new_seq - current_seq) & 0xFFFFFFFF
newer = 0 < diff < 0x80000000
```

這可正確處理 `0xFFFFFFFF -> 0` 附近的 wrap-around。

### 目前參數限制

driver 宣告 `max_ttl_ms`，但 validator 呼叫 `validate_ttl()` 時使用 module default；YAML `limits` 也沒有注入 safety state。現值剛好與 defaults 對齊，但未來若只改 YAML，驗證行為可能不變。修改安全參數時要配合測試驗證，不要只看 YAML。

## `doghead_hil`

`telemetry_validator` 用到達時間估算頻率/jitter，適合觀察 end-to-end health，但不是 MCU 內部 1 kHz deadline 的證明。`scenario_runner` 提供：

- `telemetry_only`：不送動作命令。
- `command_timeout`：送三筆後停止，驗證 TTL。
- `stale_command`：交替送 sequence 10、5。
- `nan_injection`：送 NaN，驗證拒絕。

scenario 預設等待 armed session，且 `actuator_enabled=false` 時 mask 為 0。

## `doghead_bringup`

launch 將多個 node 組合成可重現的啟動 profile；YAML 集中設定 joint names、link lengths、rate/jitter 與 safety defaults。檢查 launch 最有效的方法不是只讀檔，而是：

```bash
ros2 launch doghead_bringup shadow_mode.launch.py --show-args
ros2 node list
ros2 param dump /doghead_driver_node
```

## 編譯與測試

乾淨 shell：

```bash
cd /workspace/Software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --all --verbose
```

只編一個 package：

```bash
colcon build --packages-select doghead_driver
colcon test --packages-select doghead_driver
```

interface 改動會影響下游，通常用：

```bash
colcon build --packages-up-to doghead_bringup
```

### Motion trajectory and timing update (2026-08-04)

The second review follow-up also documents that the MCU-state subscription uses
Best-Effort QoS to match the firmware publisher, EF commands wait for fresh
measured state, trajectory preemption starts from the last generated sample,
and safety limits are resolved from the bringup YAML. Driver telemetry replay
and silence are fail-closed, while the HIL CSV includes applied PVA and output
provenance fields.

The EF callback now starts a persistent joint trajectory from the latest measured MCU position. `doghead_motion` publishes PVA samples at `trajectory_rate_hz` (default 100 Hz) until the endpoint, instead of publishing only a `progress=1.0` endpoint. Firmware interpolation advances from the position target even when target velocity and acceleration are zero. `current_joint_pos` is therefore a measured-state cache, not an optimistic endpoint assignment.

`doghead_hil` applies the configured `expected_rate_hz`, `rate_tolerance_hz`, and `max_jitter_ms` gates. Its report distinguishes jitter standard deviation, maximum period, and maximum period deviation. A deterministic CSV can be evaluated with:

```bash
ros2 run doghead_hil tracking_report telemetry.csv --expected-rate-hz 50 --max-jitter-ms 10
```

## 閱讀練習

1. 從 `setup.py` 找出三個 Python package 共提供幾個 executable。
2. 從 `motion_node.py` 找出未 armed 和 armed 的 mask 差異。
3. 追蹤 `/doghead/mcu/state` 到 `/joint_states` 的每一行轉換。
4. 找出 YAML 看似可調、但目前 code 沒有實際取用的參數。
