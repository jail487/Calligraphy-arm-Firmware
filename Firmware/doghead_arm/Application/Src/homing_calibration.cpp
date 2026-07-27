#include "homing_calibration.h"
#include "main.h"
#include "nidec_motor_driver.h"
#include "robot_config.h"
#include "trajectory_planner.h"
#include "safety_manager.h"

// ==========================================================
// 歸零校正變數定義
// ==========================================================
volatile bool robot_homing_active = true;
volatile bool robot_homing_done = false;
volatile bool homing_joint1_done = false;
volatile bool homing_joint2_done = false;
volatile bool homing_joint3_done = false;
volatile bool homing_trigger_pin10 = false;
volatile bool homing_trigger_pin11 = false;
volatile bool homing_trigger_pin12 = false;
volatile bool bypass_homing = false; // 設為 true 時可直接跳過歸零校正流程

// 微動開關校正後對應的固定角度
const float HOME_THETA1_DEG = 115.0f;
const float HOME_THETA2_DEG = 180.0f;
const float HOME_THETA3_DEG = 0.0f; // 標準坐標系下面向正前方為 0°

// 歸零搜尋轉速 (馬達本體 RPM，負值代表反轉朝低角度搜尋)
volatile int32_t home_search_rpm_m1 = 500;
volatile int32_t home_search_rpm_m2 = 500;
volatile int32_t home_search_rpm_m3 = 500;

// ==========================================================
// API 實作
// ==========================================================
extern "C" void Robot_HandleHomeSwitch(uint16_t gpio_pin) {
  // 讀取外部 GPIO PA10/PA11/PA12 校正開關狀態
  // 三個腳位均在 RESET (Low) 時觸發
  if (HAL_GPIO_ReadPin(HOME_SWITCH_PORT, gpio_pin) != GPIO_PIN_RESET) {
    return;
  }

  if (gpio_pin == JOINT1_HOME_PIN) {
    if (homing_joint1_done) {
      return;
    }
    Motor_Stop(&motor_joint_13pin);
    Motor_CalibrateAngle(&motor_joint_13pin, HOME_THETA1_DEG);
    real_theta1 = HOME_THETA1_DEG;
    target_theta1_deg = HOME_THETA1_DEG;
    target_angle1_deg = HOME_THETA1_DEG;
    pid_output_rpm1 = 0.0f;
    homing_joint1_done = true;
  } else if (gpio_pin == JOINT2_HOME_PIN) {
    if (homing_joint2_done) {
      return;
    }
    Motor_Stop(&motor_joint_13pin_2);
    Motor_CalibrateAngle(&motor_joint_13pin_2, HOME_THETA2_DEG);
    real_theta2 = HOME_THETA2_DEG;
    target_theta2_deg = HOME_THETA2_DEG;
    target_angle2_deg = HOME_THETA2_DEG;
    pid_output_rpm2 = 0.0f;
    homing_joint2_done = true;
  } else if (gpio_pin == JOINT3_HOME_PIN) {
    if (homing_joint3_done) {
      return;
    }
    Motor_Stop(&motor_joint_13pin_3);
    Motor_CalibrateAngle(&motor_joint_13pin_3, HOME_THETA3_DEG);
    real_theta3 = HOME_THETA3_DEG;
    target_theta3_deg = HOME_THETA3_DEG;
    target_angle3_deg = HOME_THETA3_DEG;
    pid_output_rpm3 = 0.0f;
    homing_joint3_done = true;
  }

  if (homing_joint1_done && homing_joint2_done && homing_joint3_done) {
    robot_homing_active = false;
    robot_homing_done = true;
    Motor_Stop(&motor_joint_13pin);
    Motor_Stop(&motor_joint_13pin_2);
    Motor_Stop(&motor_joint_13pin_3);

    // 設定歸零完成時的預設目標姿態座標為歸零位置 (65.764, 0.0, 103.228)
    pose_x = 65.764f;
    pose_y = 0.0f;
    pose_z = 103.228f;

    // 重置 PID 控制器，設定初始位置為歸零點，以防止微分突刺與積分殘留
    joint1_pid.reset(HOME_THETA1_DEG);
    joint2_pid.reset(HOME_THETA2_DEG);
    joint3_pid.reset(HOME_THETA3_DEG);

    // 重置軌跡規劃器起點與終點至當前歸零角度，防止從 0° 規劃到歸零角導致暴衝
    traj_joint1.setTarget(HOME_THETA1_DEG, HOME_THETA1_DEG, 90.0f, 180.0f);
    traj_joint2.setTarget(HOME_THETA2_DEG, HOME_THETA2_DEG, 90.0f, 180.0f);
    traj_joint3.setTarget(HOME_THETA3_DEG, HOME_THETA3_DEG, 90.0f, 180.0f);

    // 啟動逆運動學模式
    ik_mode_enabled = false;
    RobotSafety_HomingComplete();
  }
}

extern "C" bool Robot_IsHomingActive(void) {
  if (bypass_homing) {
    return false;
  }
  return robot_homing_active && RobotSafety_GetState() == ROBOT_STATE_HOMING;
}

extern "C" bool Robot_IsHomingDone(void) { return robot_homing_done; }

extern "C" void Robot_HomingStart(void) {
  robot_homing_active = true;
  robot_homing_done = false;
  homing_joint1_done = false;
  homing_joint2_done = false;
  homing_joint3_done = false;
  RobotSafety_RequestHoming();
}
