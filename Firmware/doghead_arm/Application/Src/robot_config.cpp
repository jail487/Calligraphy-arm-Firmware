#include "robot_config.h"
#include "trajectory_planner.h"
#include "nidec_motor_driver.h"
#include "safety_manager.h"
#include <cmath>

// ==========================================================
// 全域控制與回授變數定義
// ==========================================================
volatile float real_theta1 = 0.0f;
volatile float real_theta2 = 0.0f;
volatile float real_theta3 = 0.0f;

volatile float real_x = 0.0f;
volatile float real_y = 0.0f;
volatile float real_z = 0.0f;

volatile float real_rpm1 = 0.0f;
volatile float real_rpm2 = 0.0f;
volatile float real_rpm3 = 0.0f;

volatile float real_vel1 = 0.0f; // 實測關節角速度 (deg/s)
volatile float real_vel2 = 0.0f; // 實測關節角速度 (deg/s)
volatile float real_vel3 = 0.0f; // 實測關節角速度 (deg/s)

volatile float pid_output_rpm1 = 0.0f;
volatile float pid_output_rpm2 = 0.0f;
volatile float pid_output_rpm3 = 0.0f;

volatile float target_angle1_deg = 0.0f;
volatile float target_angle2_deg = 0.0f;
volatile float target_angle3_deg = 0.0f;

volatile float pose_x = 0.0f;
volatile float pose_y = 100.0f; // 預設停在前方半徑
volatile float pose_z = 0.0f;   // 預設Z高度

volatile float traj_vel1 = 0.0f;
volatile float traj_vel2 = 0.0f;
volatile float traj_vel3 = 0.0f;

const float BASE_R_OFFSET = 0.0f;
bool ik_mode_enabled = false;

float target_theta1_deg = 90.0f;
float target_theta2_deg = 0.0f;
float target_theta3_deg = 90.0f;

// ==========================================================
// C++ 類別實體定義
// ==========================================================
FiveBarKinematics kinematics(LINK_L1, LINK_L2, MOTOR_DIST_D);

// 參數: (Kp, Ki, Kd, Kv_速度前饋, Ka_加速度前饋, max_rpm)
PositionController joint1_pid(5.0f, 0.1f, 0.0f, 1.0f, 0.1f, 1710.0f);
PositionController joint2_pid(5.0f, 0.1f, 0.0f, 1.0f, 0.1f, 1710.0f);
PositionController joint3_pid(8.0f, 0.2f, 0.0f, 1.0f, 0.15f, 1710.0f);

// ==========================================================
// API 實作
// ==========================================================
extern "C" void Robot_SetTargetAngles(float theta1_deg, float theta2_deg) {
    if (!Robot_AreJointTargetsValid(theta1_deg, theta2_deg, target_theta3_deg)) {
        RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
        return;
    }
    target_theta1_deg = theta1_deg;
    target_theta2_deg = theta2_deg;
    RobotSafety_RequestRun();
}

extern "C" void Robot_SetTargetAngles3(float theta1, float theta2, float theta3) {
    if (!Robot_AreJointTargetsValid(theta1, theta2, theta3)) {
        RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
        return;
    }
    target_theta1_deg = theta1;
    target_theta2_deg = theta2;
    target_theta3_deg = theta3;
    RobotSafety_RequestRun();
}

extern "C" void Robot_SetTargetPosition(float x, float y, float z) {
    pose_x = x;
    pose_y = y;
    pose_z = z;
    ik_mode_enabled = true;
    RobotSafety_RequestRun();
}

extern "C" bool Robot_AreJointTargetsValid(float theta1_deg, float theta2_deg, float theta3_deg) {
    return std::isfinite(theta1_deg) && std::isfinite(theta2_deg) && std::isfinite(theta3_deg) &&
           theta1_deg >= JOINT1_MIN_DEG && theta1_deg <= JOINT1_MAX_DEG &&
           theta2_deg >= JOINT2_MIN_DEG && theta2_deg <= JOINT2_MAX_DEG &&
           theta3_deg >= JOINT3_MIN_DEG && theta3_deg <= JOINT3_MAX_DEG;
}
