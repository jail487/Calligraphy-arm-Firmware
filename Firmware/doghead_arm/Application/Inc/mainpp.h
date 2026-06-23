/**
 * @file mainpp.h
 * @brief C 與 C++ 的橋接介面 (Main C++)
 */
#ifndef MAINPP_H
#define MAINPP_H

#include <stdint.h>   // 提供 int32_t 等類型
#include <stdbool.h>  // 提供 bool 類型 (C99)

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================================
// 全域控制與回授變數 (供 Live Expressions 與 FreeRTOS 使用)
// 實體定義於 robot_arm_core.cpp
// ==========================================================
extern volatile int32_t test_target_rpm_m1;
extern volatile int32_t test_target_rpm_m2;
extern volatile int32_t test_target_rpm_m3;

extern volatile int32_t test_mode_selection;
extern volatile int32_t mode1_target_rpm;

extern volatile float pid_target_theta1;
extern volatile float pid_target_theta2;
extern volatile float pid_target_theta3;

extern volatile float pose_x;
extern volatile float pose_y;
extern volatile float pose_z;

extern volatile float real_theta1;
extern volatile float real_theta2;
extern volatile float real_theta3;

extern volatile float real_x;
extern volatile float real_y;
extern volatile float real_z;

extern volatile float real_rpm1;
extern volatile float real_rpm2;
extern volatile float real_rpm3;

extern volatile float pid_output_rpm1;
extern volatile float pid_output_rpm2;
extern volatile float pid_output_rpm3;

extern volatile bool robot_homing_active;
extern volatile bool robot_homing_done;
extern volatile bool homing_joint1_done;
extern volatile bool homing_joint2_done;
extern volatile bool homing_joint3_done;
extern volatile bool homing_trigger_pin10;
extern volatile bool homing_trigger_pin11;
extern volatile bool homing_trigger_pin12;

// IK/PID 迴圈中實際使用的目標角度 (Degree)
extern volatile float target_angle1_deg;
extern volatile float target_angle2_deg;
extern volatile float target_angle3_deg;

// ----------------------------------------------------------

// 初始化機器人 (馬達、PID)
void Robot_Init(void);

// 外部微動開關校正：下降沿時呼叫，停止對應馬達並把當前角度校正為指定值
void Robot_HandleHomeSwitch(uint16_t gpio_pin);

// 供 FreeRTOS 控制任務判斷是否仍在歸零流程中
bool Robot_IsHomingActive(void);
bool Robot_IsHomingDone(void);

// 設定測試模式 (true=純速度控制, false=位置軌跡控制)
void Robot_SetTestMode(bool enable);

// 設定3顆馬達的目標角度
void Robot_SetTargetAngles3(float theta1, float theta2, float theta3);

// 設定目標角度 (Legacy for 2 motors)
void Robot_SetTargetAngles(float theta1_deg, float theta2_deg);

// 機器人主迴圈 (需定期呼叫)
// dt_seconds: 距離上次呼叫的時間差 (秒)，例如 1ms = 0.001f
void Robot_Loop(float dt_seconds);

// 設定目標位置 (使用運動學解算)
void Robot_SetTargetPosition(float x, float y, float z);

// 測試模式下直接設定馬達速度 (RPM)
void Robot_SetTestSpeed(int32_t rpm_motor1, int32_t rpm_motor2, int32_t rpm_motor3);

// PID 控制迴圈（含前饋，不含運動學）
void Robot_PID_Loop(float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif // MAINPP_H
