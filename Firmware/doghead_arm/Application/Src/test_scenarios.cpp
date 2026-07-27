#include "test_scenarios.h"
#include "robot_config.h"
#include "safety_manager.h"

// ==========================================================
// 測試模式變數定義
// ==========================================================
volatile int32_t test_target_rpm_m1 = 0;  
volatile int32_t test_target_rpm_m2 = 0;
volatile int32_t test_target_rpm_m3 = 0;

volatile int32_t test_mode_selection = 0;
volatile int32_t mode1_target_rpm = 0;
volatile bool enable_square_test = false;

// ==========================================================
// API 實作
// ==========================================================
extern "C" void Robot_SetTestMode(bool enable) {
    // 統一使用 test_mode_selection 作為唯一模式來源
    // false -> PID 模式 (0)
    // true  -> 測試模式 (1: 定速模式)
    test_mode_selection = enable ? 1 : 0;
    if (enable) {
        RobotSafety_RequestRun();
    }
    if (enable) {
        ik_mode_enabled = false;  // 測試模式下關閉運動學控制
    }
}

extern "C" void Robot_SetTestSpeed(int32_t rpm_motor1, int32_t rpm_motor2, int32_t rpm_motor3) {
    if (rpm_motor1 > 1710 || rpm_motor1 < -1710 ||
        rpm_motor2 > 1710 || rpm_motor2 < -1710 ||
        rpm_motor3 > 1710 || rpm_motor3 < -1710) {
        RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
        return;
    }
    // 統一寫入 FreeRTOS 控制任務使用的目標變數
    test_target_rpm_m1 = rpm_motor1;
    test_target_rpm_m2 = rpm_motor2;
    test_target_rpm_m3 = rpm_motor3;
}
