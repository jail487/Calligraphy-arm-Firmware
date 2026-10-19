# Firmware Development & Bringup Rules

This file defines the strict rules and best practices for developing and reviewing embedded firmware, particularly for motor control and safety-critical Bringup testing. All agents MUST follow these rules when modifying firmware code.

## 1. Safety Gate Pattern
- **Ultimate Gate Enforcement**: Before any direct call to actuator/hardware APIs (e.g., `Motor_SetSpeed`, `Motor_Start`), there MUST be an "Ultimate Gate". 
- **Validation**: The Ultimate Gate must violently validate the outputs before casting or applying. All floating-point outputs must be checked for NaN/Inf (`std::isfinite`).
- **Sanity Limits**: Actuator commands must be clamped or checked against absolute hardcoded safety limits (e.g., Bringup max RPM).
- **Safety State Check**: The global safety state (e.g., `RobotSafety_CanDrive()`) MUST be checked exactly one line before the hardware API call.

## 2. RTOS Timeout & Stale Command Handling
- **Timebase**: NEVER use simple `g_current_time_ms` counters updated by interrupts for critical timeouts. Use native RTOS tick mechanisms (e.g., `xTaskGetTickCount()`) with proper type (`TickType_t`).
- **Stale Command Annihilation**: When a command times out in a queue, do not just return or ignore it. You MUST explicitly clear the cached command (`memset` to 0) and reset any "has_command" flags. This prevents stale commands from resurrecting after a 32-bit tick wrap-around (e.g., 49.7 days).

## 3. Loop Deadlock Prevention
- **No Infinite Polling in Control Loops**: Inside critical control loops (`Robot_Loop` or RTOS tasks), if a timeout or fault occurs, trigger the fault state and `return` immediately. NEVER use `while(1)` or `continue` to trap the thread, as this starves the RTOS scheduler and prevents new commands from clearing the fault.

## 4. Strict Actuator Isolation for Testing
- **Physical Isolation**: In test modes, do not just set the speed of unused axes to 0. You MUST physically disable them (e.g., `Motor_Stop()` or pulling the EN pin low). Use an `enable_mask` to enforce this.

## 5. Command Semantics Separation
- **No Field Reuse**: Never reuse a communication field for a different semantic meaning (e.g., using `target_theta1_deg` to pass `test_rpm`). 
- **Dedicated Fields**: If a new mode is introduced (like Test Mode), add dedicated fields (`test_axis`, `test_rpm`) to the command structure. Ensure validation logic strictly branches based on the active mode so that normal validation (like joint angle limits) does not accidentally block valid test commands.

## 6. Toolchain Independence
- **Avoid Modern C++ in C Core**: Unless explicitly instructed that the toolchain supports it (e.g., C++17), avoid using standard library functions that are version-specific (like `std::clamp` from `<algorithm>`). Use standard `if/else` bounds checking to ensure warning-free compilation on all bare-metal/RTOS toolchains.

## 7. Documentation Management
- **Centralized Documentation**: All markdown (`.md`) documentation files MUST be placed under the `docs/` directory. Do not leave loose `.md` files in the root or source code directories (except for the root `README.md` and third-party/vendor library documents like `micro_ros_stm32cubemx_utils/*.md` which must be exempted and kept intact).
- **Proper Categorization**: When creating a new `.md` file, place it into an appropriate subfolder within `docs/` based on its type (e.g., `ros2_integration`, `project_management`, `firmware_audit`).
- **Prune Obsolete Files**: Documents that are too old or no longer hold reference value should be directly deleted to maintain workspace cleanliness.

## 8. Automated CLI Build & Flash Workflow
- **Auto-Flash after Firmware Modification**: Whenever an agent modifies any embedded firmware files (`Firmware/doghead_arm/**`), the agent MUST proactively invoke the CLI build & flash tool (`scripts/build_and_flash.ps1` or `scripts/flash_stm32.ps1`) to compile and download the latest binary directly to the connected STM32 target.

## 9. Autonomous Telemetry Inspection & Proactive Debugging
- **Autonomous Data Sampling**: When diagnosing issues (such as encoder counts, homing states, angles, and telemetry timeouts), agents MUST proactively sample live ROS 2 topics (`/joint_states`, `/doghead/mcu/state`, etc.) or container logs via CLI without asking the user to read or copy-paste data.
- **Autonomous Fix & Flash**: Agents should autonomously identify discrepancies, apply code fixes, and trigger CLI flashing directly to maximize debugging efficiency.

## 10. Single Source of Truth (SSoT) & Zero Magic Numbers
- **Hardware & Geometry SSoT**: All mechanical parameters (link lengths, gear ratios, encoder PPR, switch angles, standby/home angles, safety limits) must be defined in central configuration files (`Firmware/doghead_arm/Application/Inc/robot_config.h` in Firmware, and mirrored via `Software/ros2_ws/robot_constants.py` in Software).
- **No Scattered Magic Numbers**: Never hardcode angles (like `111.0`, `180.0`, `45.0`, `90.0`), tolerances, or communication axis IDs inline in state machines, algorithms, or CLI menus. Always use descriptive constants/enums (e.g. `HOME_STANDBY_THETA3_DEG`, `CMD_AXIS_SYSTEM_HOMING`).
- **Unified Hardware Command Interface**: Hardware processes (like homing, calibration, estop) MUST be driven by the firmware state machine. The software side must ONLY send commands (e.g. `CMD_AXIS_SYSTEM_HOMING`) and poll the state machine (`safety_state == ROBOT_STATE_READY`), NEVER re-implementing fake/virtual hardware sequences via open-loop trajectory commands.
