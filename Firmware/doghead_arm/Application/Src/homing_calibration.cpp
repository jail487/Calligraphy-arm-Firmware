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

const uint32_t homing_timeout_ms = 20000U;
const float homing_max_travel_deg = 270.0f;

static uint32_t homing_start_ms = 0;
static float homing_start_theta1 = 0.0f;
static float homing_start_theta2 = 0.0f;
static float homing_start_theta3 = 0.0f;

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
    robot_state.real_theta1 = HOME_THETA1_DEG;
    robot_target.target_theta1_deg = HOME_THETA1_DEG;
    robot_target.target_angle1_deg = HOME_THETA1_DEG;
    robot_state.pid_output_rpm1 = 0.0f;
    homing_joint1_done = true;
  } else if (gpio_pin == JOINT2_HOME_PIN) {
    if (homing_joint2_done) {
      return;
    }
    Motor_Stop(&motor_joint_13pin_2);
    Motor_CalibrateAngle(&motor_joint_13pin_2, HOME_THETA2_DEG);
    robot_state.real_theta2 = HOME_THETA2_DEG;
    robot_target.target_theta2_deg = HOME_THETA2_DEG;
    robot_target.target_angle2_deg = HOME_THETA2_DEG;
    robot_state.pid_output_rpm2 = 0.0f;
    homing_joint2_done = true;
  } else if (gpio_pin == JOINT3_HOME_PIN) {
    if (homing_joint3_done) {
      return;
    }
    Motor_Stop(&motor_joint_13pin_3);
    Motor_CalibrateAngle(&motor_joint_13pin_3, HOME_THETA3_DEG);
    robot_state.real_theta3 = HOME_THETA3_DEG;
    robot_target.target_theta3_deg = HOME_THETA3_DEG;
    robot_target.target_angle3_deg = HOME_THETA3_DEG;
    robot_state.pid_output_rpm3 = 0.0f;
    homing_joint3_done = true;
  }

  if (homing_joint1_done && homing_joint2_done && homing_joint3_done) {
    robot_homing_active = false;
    robot_homing_done = true;
    Motor_Stop(&motor_joint_13pin);
    Motor_Stop(&motor_joint_13pin_2);
    Motor_Stop(&motor_joint_13pin_3);

    // 設定歸零完成時的預設目標姿態座標為歸零位置 (65.764, 0.0, 103.228)
    robot_target.pose_x = 65.764f;
    robot_target.pose_y = 0.0f;
    robot_target.pose_z = 103.228f;

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
  
  homing_start_ms = HAL_GetTick();
  homing_start_theta1 = Motor_GetAngle(&motor_joint_13pin);
  homing_start_theta2 = Motor_GetAngle(&motor_joint_13pin_2);
  homing_start_theta3 = Motor_GetAngle(&motor_joint_13pin_3);
}

extern "C" void Robot_Homing_Tick(void) {
  if (!Robot_IsHomingActive()) return;

  // 先讀取編碼器，讓 Live Expressions 與回授保持更新
  Motor_Update(&motor_joint_13pin);
  Motor_Update(&motor_joint_13pin_2);
  Motor_Update(&motor_joint_13pin_3);

  robot_state.real_theta1 = Motor_GetAngle(&motor_joint_13pin);
  robot_state.real_theta2 = Motor_GetAngle(&motor_joint_13pin_2);
  robot_state.real_theta3 = Motor_GetAngle(&motor_joint_13pin_3);

  if ((HAL_GetTick() - homing_start_ms) > homing_timeout_ms) {
    RobotSafety_TriggerFault(ROBOT_FAULT_HOMING_TIMEOUT);
    return;
  }
  if (std::fabs(robot_state.real_theta1 - homing_start_theta1) > homing_max_travel_deg ||
      std::fabs(robot_state.real_theta2 - homing_start_theta2) > homing_max_travel_deg ||
      std::fabs(robot_state.real_theta3 - homing_start_theta3) > homing_max_travel_deg) {
    RobotSafety_TriggerFault(ROBOT_FAULT_HOMING_TRAVEL_LIMIT);
    return;
  }

  robot_state.real_rpm1 = Motor_GetVelocity(&motor_joint_13pin);
  robot_state.real_rpm2 = Motor_GetVelocity(&motor_joint_13pin_2);
  robot_state.real_rpm3 = Motor_GetVelocity(&motor_joint_13pin_3);

  // 換算成關節角速度 (deg/s) = RPM * 360 / 60 / 減速比 (114.0f) = RPM * 6.0f / 114.0f
  robot_state.real_vel1 = robot_state.real_rpm1 * 6.0f;
  robot_state.real_vel2 = robot_state.real_rpm2 * 6.0f;
  robot_state.real_vel3 = robot_state.real_rpm3 * 6.0f;

  // 如果開關已經被壓住，即使沒有邊沿中斷，也直接做校正
  if (!homing_joint1_done &&
      HAL_GPIO_ReadPin(HOME_SWITCH_PORT, JOINT1_HOME_PIN) == GPIO_PIN_RESET) {
    Robot_HandleHomeSwitch(JOINT1_HOME_PIN);
  }
  if (!homing_joint2_done &&
      HAL_GPIO_ReadPin(HOME_SWITCH_PORT, JOINT2_HOME_PIN) == GPIO_PIN_RESET) {
    Robot_HandleHomeSwitch(JOINT2_HOME_PIN);
  }
  if (!homing_joint3_done &&
      HAL_GPIO_ReadPin(HOME_SWITCH_PORT, JOINT3_HOME_PIN) == GPIO_PIN_RESET) {
    Robot_HandleHomeSwitch(JOINT3_HOME_PIN);
  }

  // 只讓尚未校正完成的軸持續慢速移動，已校正完成的軸則完全關閉 (Pull Enable Low)
  if (!homing_joint1_done) {
    Motor_Start(&motor_joint_13pin);
    Motor_SetSpeed(&motor_joint_13pin, home_search_rpm_m1);
  } else {
    Motor_Stop(&motor_joint_13pin);
  }

  if (!homing_joint2_done) {
    Motor_Start(&motor_joint_13pin_2);
    Motor_SetSpeed(&motor_joint_13pin_2, home_search_rpm_m2);
  } else {
    Motor_Stop(&motor_joint_13pin_2);
  }

  if (!homing_joint3_done) {
    Motor_Start(&motor_joint_13pin_3);
    Motor_SetSpeed(&motor_joint_13pin_3, home_search_rpm_m3);
  } else {
    Motor_Stop(&motor_joint_13pin_3);
  }

  robot_state.real_rpm1 = Motor_GetVelocity(&motor_joint_13pin);
  robot_state.real_rpm2 = Motor_GetVelocity(&motor_joint_13pin_2);
  robot_state.real_rpm3 = Motor_GetVelocity(&motor_joint_13pin_3);
}
