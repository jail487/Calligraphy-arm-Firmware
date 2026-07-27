#ifndef TEST_SCENARIOS_H
#define TEST_SCENARIOS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================================
// 測試模式變數
// ==========================================================
extern volatile int32_t test_target_rpm_m1;
extern volatile int32_t test_target_rpm_m2;
extern volatile int32_t test_target_rpm_m3;

extern volatile int32_t test_mode_selection;
extern volatile int32_t mode1_target_rpm;
extern volatile bool enable_square_test;

// ==========================================================
// API 宣告
// ==========================================================
void Robot_SetTestMode(bool enable);
void Robot_SetTestSpeed(int32_t rpm_motor1, int32_t rpm_motor2, int32_t rpm_motor3);

#ifdef __cplusplus
}
#endif

#endif // TEST_SCENARIOS_H
