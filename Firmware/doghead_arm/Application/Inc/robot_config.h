#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#ifndef HOST_TEST
#include "main.h"
#endif

// ==========================================================
// 歸零校正腳位設定
// ==========================================================
#define JOINT1_HOME_PIN   GPIO_PIN_10
#define JOINT2_HOME_PIN   GPIO_PIN_12
#define JOINT3_HOME_PIN   GPIO_PIN_11
#define HOME_SWITCH_PORT  GPIOA

// User-confirmed software limits matching the current mechanical envelope.
// Joint 1 cannot exceed 111.0 deg (Home Reset angle is the mechanical upper limit)
#define JOINT1_MIN_DEG 0.0f
#define JOINT1_MAX_DEG 111.0f
// Joint 2 cannot exceed 180.0 deg (Home Reset angle is the mechanical upper limit)
#define JOINT2_MIN_DEG 105.0f
#define JOINT2_MAX_DEG 180.0f
#define JOINT3_MIN_DEG 0.0f
#define JOINT3_MAX_DEG 90.0f

// ==========================================================
// 歸零微動開關與待命姿態 (Single Source of Truth)
// ==========================================================
// 微動開關觸發時的實體角度
#define HOME_SWITCH_THETA1_DEG 111.0f
#define HOME_SWITCH_THETA2_DEG 180.0f
#define HOME_SWITCH_THETA3_DEG 90.0f

// 歸零完成後，各軸回退抵達的待命角度 (Standby Pose)
#define HOME_STANDBY_THETA1_DEG 111.0f
#define HOME_STANDBY_THETA2_DEG 180.0f
#define HOME_STANDBY_THETA3_DEG 45.0f   // 底座預設回退至 45° 待命 (正前方對稱軸，書寫正方形中心正對向)
#define HOME_STANDBY_TOLERANCE_DEG 0.5f // 抵達待命角度之容許誤差 (deg)

// ==========================================================
// 遠端通訊控制通道定義 (McuMotorTestCommand.axis)
// ==========================================================
typedef enum {
    CMD_AXIS_MOTOR_ALL         = 0,
    CMD_AXIS_MOTOR_J1          = 1,
    CMD_AXIS_MOTOR_J2          = 2,
    CMD_AXIS_MOTOR_J3          = 3,

    CMD_AXIS_PID_BASE          = 10,
    CMD_AXIS_PID_ALL           = 10,
    CMD_AXIS_PID_J1            = 11,
    CMD_AXIS_PID_J2            = 12,
    CMD_AXIS_PID_J3            = 13,

    CMD_AXIS_FEEDFORWARD_BASE  = 20,
    CMD_AXIS_FEEDFORWARD_ALL   = 20,
    CMD_AXIS_FEEDFORWARD_J1    = 21,
    CMD_AXIS_FEEDFORWARD_J2    = 22,
    CMD_AXIS_FEEDFORWARD_J3    = 23,

    CMD_AXIS_SYSTEM_HOMING     = 99  // 遠端觸發 STM32 硬體微動開關全自動歸零
} RemoteCommandAxis_t;

#include "actuator_config.h"

// ==========================================================
// Joint PVA & Shadow Mode Configuration
// ==========================================================
#ifndef JOINT_PVA_SHADOW_MODE_ENABLED
#define JOINT_PVA_SHADOW_MODE_ENABLED 0
#endif

// Strict, Planner-synchronized Joint PVA Dynamic Limits:
// - Max Motor RPM Cap: 5000.0f (Motor physical spec rated max RPM before gearbox reduction).
#define JOINT_PVA_MAX_VEL_RAD_S     4.593f // 5000 RPM / 114 * (2*pi/60)
#define JOINT_PVA_MAX_ACCEL_RAD_S2  15.000f // Scaling acceleration limit accordingly
#define BRINGUP_MAX_MOTOR_RPM       5000.0f

#define DEG_TO_RAD_F 0.017453292519943295f
#define RAD_TO_DEG_F 57.29577951308232f

#define JOINT1_MIN_RAD (JOINT1_MIN_DEG * DEG_TO_RAD_F)
#define JOINT1_MAX_RAD (JOINT1_MAX_DEG * DEG_TO_RAD_F)
#define JOINT2_MIN_RAD (JOINT2_MIN_DEG * DEG_TO_RAD_F)
#define JOINT2_MAX_RAD (JOINT2_MAX_DEG * DEG_TO_RAD_F)
#define JOINT3_MIN_RAD (JOINT3_MIN_DEG * DEG_TO_RAD_F)
#define JOINT3_MAX_RAD (JOINT3_MAX_DEG * DEG_TO_RAD_F)

// ==========================================================
// 馬達 GPIO 腳位與連接埠設定
// ==========================================================
#ifndef HOST_TEST
#include "motor_config.h"
#endif

#ifdef __cplusplus
#include "pid_controller.hpp"

// C++ Class Instances
extern PositionController joint1_pid;
extern PositionController joint2_pid;
extern PositionController joint3_pid;
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================================
// 全域控制與回授變數
// ==========================================================
typedef struct {
    volatile float real_theta1;
    volatile float real_theta2;
    volatile float real_theta3;

    volatile float real_rpm1;
    volatile float real_rpm2;
    volatile float real_rpm3;

    volatile float real_vel1; // 實測關節角速度 (deg/s)
    volatile float real_vel2; // 實測關節角速度 (deg/s)
    volatile float real_vel3; // 實測關節角速度 (deg/s)

    volatile float pid_output_rpm1;
    volatile float pid_output_rpm2;
    volatile float pid_output_rpm3;

    volatile float traj_vel1; // 路徑規劃輸出的角速度1 (deg/s)
    volatile float traj_vel2; // 路徑規劃輸出的角速度2 (deg/s)
    volatile float traj_vel3; // 路徑規劃輸出的角速度3 (deg/s)
} RobotState_t;

extern RobotState_t robot_state;

// ==========================================================
// API 宣告
// ==========================================================
bool Robot_AreJointTargetsValid(float theta1_deg, float theta2_deg, float theta3_deg);
void Robot_SetPidGains(uint8_t axis, float kp, float ki, float kd, float kv, float ka);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CONFIG_H

