# micro-ROS 自訂介面標頭與靜態函式庫產生流程

> **目前實作狀態：** 靜態函式庫已包含 `McuSessionControl`，並以 ROS 2 Humble 與 STM32F446 的 `softfp` ABI 重建。任何 `.msg`、ROS 發行版、編譯器 ABI 或建置設定變動後，都必須重新產生並完成 HIL 驗證。

**文件類型：** 技術指南／整合協定  
**適用範圍：** `doghead_interfaces`、`micro_ros_stm32cubemx_utils` 與 STM32 micro-ROS 建置邊界  
**日期：** 2026-07-31

## 1. 能力開關與編譯邊界

`microros_app.h` 用明確的能力開關決定是否編譯 DogHead 自訂介面。目的在於維持裸機工具鏈穩定、避免 `main()` 前 HardFault，也避免引用未驗證的產物：

```c
#if defined(MICROROS_ENABLE_DOGHEAD_INTERFACES) && (MICROROS_ENABLE_DOGHEAD_INTERFACES == 1)
  #if defined(__has_include)
    #if __has_include(<doghead_interfaces/msg/mcu_state.h>)
      #define MICROROS_HAS_DOGHEAD_INTERFACES 1
    #else
      #define MICROROS_HAS_DOGHEAD_INTERFACES 0
    #endif
  #else
    #define MICROROS_HAS_DOGHEAD_INTERFACES 1
  #endif
#else
  #ifndef MICROROS_HAS_DOGHEAD_INTERFACES
    #define MICROROS_HAS_DOGHEAD_INTERFACES 0
  #endif
#endif
```

已驗證的 HIL 預設值為 `MICROROS_ENABLE_TOPIC_TELEMETRY=1` 與 `MICROROS_ENABLE_DOGHEAD_INTERFACES=1`。標頭存在時，韌體會發布完整遙測並建立 session／命令訂閱；任一開關設為 `0` 是刻意保留的 ping-only 診斷模式，不是正常應用設定。

舊的預設關閉做法只適用於 T22 之前：當時尚未重建適合 Humble 的自然 GCC attribute 函式庫。若放回未驗證 archive，可能重新引發建構子或 ABI 的 `main()` 前故障；因此「重建函式庫 + 韌體 HIL 驗證」必須視為同一個步驟。

## 2. 何時需要重建

出現下列任一變動時，重建 C 標頭與 `libmicroros.a`：

- `Software/ros2_ws/src/doghead_interfaces/msg/` 中的 `.msg` 定義。
- ROS 2 發行版（本專案是 Humble）。
- STM32 編譯器、FPU 選項或 ABI（特別是 `softfp`）。
- `colcon.meta`、micro-ROS builder 或額外 package 設定。

## 3. 產生步驟

### 步驟 1：在 Humble 容器建置介面套件

```bash
cd /workspace/Software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select doghead_interfaces
source install/setup.bash
```

### 步驟 2：產生 micro-ROS 型別支援

在 `micro_ros_setup` 工作區或對應 Docker 容器中執行：

```bash
ros2 run micro_ros_setup create_firmware_ws.sh freertos stm32f4
ros2 run micro_ros_setup configure_firmware.sh [configuration_name]
# 把 doghead_interfaces 套件目錄加入 extra packages 清單
ros2 run micro_ros_setup build_firmware.sh
```

### 步驟 3：確認產物與 include 路徑

產生樹中應有下列 C 標頭：

```text
build/doghead_interfaces/rosidl_generator_c/doghead_interfaces/msg/
├── mcu_state.h
├── mcu_diagnostics.h
├── mcu_joint_setpoint.h
├── mcu_motor_test_command.h
└── mcu_session_control.h
```

靜態函式庫與標頭必須一起放入 STM32 專案使用的 `libmicroros` 目錄；不要只複製標頭或只更換 archive。實際路徑與建置方式請依本專案的 [工具 README](../../Firmware/doghead_arm/micro_ros_stm32cubemx_utils/README.md) 核對。

## 4. Topic 與訊息核對

目前協定的主要 topic 為：

- `/doghead/mcu/joint_setpoint`
- `/doghead/mcu/motor_test_command`
- `/doghead/mcu/session_control`
- `/doghead/mcu/state`
- `/doghead/mcu/diagnostics`
- `/joint_states`

在 MCU 訂閱命令之前，確認主機與韌體對每個 `.msg` 使用同一版型別支援。`McuSessionControl` 遺漏、兩端版本不同或 ABI 不符時，都不應用「只要能連線即可」來判定系統正確；必須重新建置並以輸出關閉 HIL 驗證。
