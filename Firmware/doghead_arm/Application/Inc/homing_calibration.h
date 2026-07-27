#ifndef HOMING_CALIBRATION_H
#define HOMING_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================================
// 歸零校正變數
// ==========================================================
extern volatile bool robot_homing_active;
extern volatile bool robot_homing_done;
extern volatile bool homing_joint1_done;
extern volatile bool homing_joint2_done;
extern volatile bool homing_joint3_done;
extern volatile bool homing_trigger_pin10;
extern volatile bool homing_trigger_pin11;
extern volatile bool homing_trigger_pin12;
extern volatile bool bypass_homing; // 設為 true 時可直接跳過歸零校正流程

// 微動開關校正後對應的固定角度
extern const float HOME_THETA1_DEG;
extern const float HOME_THETA2_DEG;
extern const float HOME_THETA3_DEG;

// 歸零搜尋轉速 (馬達本體 RPM，負值代表反轉朝低角度搜尋)
extern volatile int32_t home_search_rpm_m1;
extern volatile int32_t home_search_rpm_m2;
extern volatile int32_t home_search_rpm_m3;

// ==========================================================
// API 宣告
// ==========================================================
void Robot_HandleHomeSwitch(uint16_t gpio_pin);
bool Robot_IsHomingActive(void);
bool Robot_IsHomingDone(void);
void Robot_HomingStart(void);

#ifdef __cplusplus
}
#endif

#endif // HOMING_CALIBRATION_H
