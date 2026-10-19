#ifndef TEST_SCENARIOS_H
#define TEST_SCENARIOS_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_config.h" // For BRINGUP_MAX_MOTOR_RPM (joint-space RPM cap)

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================================
// 測試模式變數
// ==========================================================
// Test Mode 切換現在全由 mainpp.cpp 透過 Queue 指令傳入
extern volatile int32_t test_mode_selection;

// ==========================================================
// 馬達命令結構 (供 Test Mode 使用)
// ==========================================================
typedef struct {
    int32_t rpm[3];
    uint8_t enable_mask;
} MotorCommand_t;

// Build the dedicated motor-test output.  A disabled command is a valid
// safety-stop event and must never fall through to a legacy test_mode field.
//
// Unit contract:
//   requested_rpm  = joint/output-shaft RPM (NOT motor shaft RPM).
//   The caller (mainpp.cpp Robot_Loop) derives this value from
//   cmd.motor_test.target_motor_rpm, which is also joint/output-shaft RPM.
//   Motor_SetSpeed() receives the same joint RPM and applies gear ratio
//   internally exactly once via Joint_RPM_To_Command_Freq_Hz().
//
// Cap: BRINGUP_MAX_MOTOR_RPM (robot_config.h) is the bringup ceiling in
//   joint RPM, consistent with the host safety_validator bringup_motor_rpm_max.
static inline bool Robot_TestMode_BuildCommand(bool enable,
                                               uint8_t selected_axis,
                                               int32_t requested_rpm,
                                               MotorCommand_t *out) {
    if (!out) return false;

    out->rpm[0] = 0;
    out->rpm[1] = 0;
    out->rpm[2] = 0;
    out->enable_mask = 0U;

    if (!enable) {
        return true;
    }
    if (selected_axis >= 3U) {
        return false;
    }

    // Clamp to the bringup joint-RPM ceiling.  Using BRINGUP_MAX_MOTOR_RPM
    // keeps the MCU cap consistent with the host validator (bringup_motor_rpm_max)
    // and avoids silently truncating valid commands at the old legacy ±100 cap.
    const int32_t rpm_cap = (int32_t)BRINGUP_MAX_MOTOR_RPM;
    if (requested_rpm > rpm_cap) requested_rpm = rpm_cap;
    if (requested_rpm < -rpm_cap) requested_rpm = -rpm_cap;
    out->rpm[selected_axis] = requested_rpm;
    out->enable_mask = (uint8_t)(1U << selected_axis);
    return true;
}

#ifdef __cplusplus
}
#endif

#endif // TEST_SCENARIOS_H
