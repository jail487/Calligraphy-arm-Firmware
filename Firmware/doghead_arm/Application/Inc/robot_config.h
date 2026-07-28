#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

// ==========================================================
// 機構參數設定 (單位: mm)
// ==========================================================
#define LINK_L1 113.9f   // 主動臂 (連接馬達)
#define LINK_L2 113.9f   // 從動臂 (連接末端)
#define MOTOR_DIST_D 0.f // 兩馬達間距

// ==========================================================
// 歸零校正腳位設定
// ==========================================================
#define JOINT1_HOME_PIN   GPIO_PIN_10
#define JOINT2_HOME_PIN   GPIO_PIN_12
#define JOINT3_HOME_PIN   GPIO_PIN_11
#define HOME_SWITCH_PORT  GPIOA

// Confirm these software limits against the mechanical hard stops before use.
#define JOINT1_MIN_DEG 80.0f
#define JOINT1_MAX_DEG 180.0f
#define JOINT2_MIN_DEG 0.0f
#define JOINT2_MAX_DEG 180.0f
#define JOINT3_MIN_DEG -180.0f
#define JOINT3_MAX_DEG 180.0f

// ==========================================================
// 馬達 GPIO 腳位與連接埠設定
// ==========================================================
#include "motor_config.h"


#ifdef __cplusplus
#include "kinematics.hpp"
#include "pid_controller.hpp"

// C++ Class Instances
extern FiveBarKinematics kinematics;
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

    volatile float real_x;
    volatile float real_y;
    volatile float real_z;

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

typedef struct {
    volatile float target_angle1_deg;
    volatile float target_angle2_deg;
    volatile float target_angle3_deg;

    volatile float pose_x;
    volatile float pose_y;
    volatile float pose_z;

    float target_theta1_deg;
    float target_theta2_deg;
    float target_theta3_deg;
} RobotTarget_t;

extern RobotState_t robot_state;
extern RobotTarget_t robot_target;

extern const float BASE_R_OFFSET;
extern bool ik_mode_enabled;

// ==========================================================
// API 宣告
// ==========================================================
void Robot_SetTargetAngles(float theta1_deg, float theta2_deg);
void Robot_SetTargetAngles3(float theta1, float theta2, float theta3);
void Robot_SetTargetPosition(float x, float y, float z);
bool Robot_AreJointTargetsValid(float theta1_deg, float theta2_deg, float theta3_deg);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CONFIG_H
