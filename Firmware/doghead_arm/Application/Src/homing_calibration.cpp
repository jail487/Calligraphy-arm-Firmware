#include "homing_calibration.h"
#include "hardware_io.h"
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
volatile bool bypass_homing = false; // 設為 false 啟用實體微動開關自動歸零校正流程

// 微動開關校正角度直接使用 homing_calibration.h / robot_config.h 定義之 SSoT 巨集

// 歸零搜尋轉速 (Joint RPM，調慢為 12 RPM 確保平穩安全觸發)
volatile int32_t home_search_rpm_m1 = 12;
volatile int32_t home_search_rpm_m2 = 12;
volatile int32_t home_search_rpm_m3 = 12;

const uint32_t homing_timeout_ms = 30000U;
const float homing_max_travel_deg = 270.0f;

#if (ACTUATOR_HARDWARE_OUTPUT_ENABLED != 0)
static uint32_t homing_start_ms = 0;
static float homing_start_theta1 = 0.0f;
static float homing_start_theta2 = 0.0f;
static float homing_start_theta3 = 0.0f;
#endif

// ==========================================================
// API 實作
// ==========================================================
extern "C" void Robot_HandleHomeSwitch(uint16_t gpio_pin) {
  // 已經改由 Task 根據鎖存事件呼叫，不再使用 HAL_GPIO_ReadPin

  if (gpio_pin == JOINT1_HOME_PIN || gpio_pin == 0) {
    if (!homing_joint1_done || gpio_pin == 0) {
      Motor_Stop(&motor_joint_13pin);
      Motor_CalibrateAngle(&motor_joint_13pin, HOME_THETA1_DEG);
      robot_state.real_theta1 = HOME_THETA1_DEG;
      robot_state.pid_output_rpm1 = 0.0f;
      homing_joint1_done = true;
    }
  }
  if (gpio_pin == JOINT2_HOME_PIN || gpio_pin == 0) {
    if (!homing_joint2_done || gpio_pin == 0) {
      Motor_Stop(&motor_joint_13pin_2);
      Motor_CalibrateAngle(&motor_joint_13pin_2, HOME_THETA2_DEG);
      robot_state.real_theta2 = HOME_THETA2_DEG;
      robot_state.pid_output_rpm2 = 0.0f;
      homing_joint2_done = true;
    }
  }
  if (gpio_pin == JOINT3_HOME_PIN || gpio_pin == 0) {
    if (!homing_joint3_done || gpio_pin == 0) {
      Motor_Stop(&motor_joint_13pin_3);
      Motor_CalibrateAngle(&motor_joint_13pin_3, HOME_THETA3_DEG);
      robot_state.real_theta3 = HOME_THETA3_DEG;
      robot_state.pid_output_rpm3 = 0.0f;
      homing_joint3_done = true;
    }
  }

  if (homing_joint1_done && homing_joint2_done && homing_joint3_done) {
    robot_homing_active = false;
    robot_homing_done = true;
    Motor_Stop(&motor_joint_13pin);
    Motor_Stop(&motor_joint_13pin_2);
    Motor_Stop(&motor_joint_13pin_3);

    // 重置 PID 控制器，設定初始位置為歸零點，以防止微分突刺與積分殘留
    joint1_pid.reset(HOME_THETA1_DEG);
    joint2_pid.reset(HOME_THETA2_DEG);
    joint3_pid.reset(HOME_THETA3_DEG);

    RobotSafety_HomingComplete();
  }
}

extern "C" bool Robot_IsHomingActive(void) {
#if (ACTUATOR_HARDWARE_OUTPUT_ENABLED == 0)
  return false;
#else
  if (bypass_homing) {
    return false;
  }
  return robot_homing_active && RobotSafety_GetState() == ROBOT_STATE_HOMING;
#endif
}

extern "C" bool Robot_IsHomingDone(void) { return robot_homing_done; }

extern "C" void Robot_HomingStart(void) {
#if (ACTUATOR_HARDWARE_OUTPUT_ENABLED == 0)
  homing_joint1_done = false;
  homing_joint2_done = false;
  homing_joint3_done = false;
  robot_homing_active = false;
  robot_homing_done = false;
  return;
#else
  if (bypass_homing) {
    homing_joint1_done = true;
    homing_joint2_done = true;
    homing_joint3_done = true;
    robot_homing_active = false;
    robot_homing_done = true;
    return;
  }

  homing_start_ms = HW_GetMonotonicTimeMs();
  homing_start_theta1 = robot_state.real_theta1;
  homing_start_theta2 = robot_state.real_theta2;
  homing_start_theta3 = robot_state.real_theta3;
  
  // 恢復三軸 (J1, J2, J3) 開機同時歸零搜尋
  homing_joint1_done = false;
  homing_joint2_done = false;
  homing_joint3_done = false;
  robot_homing_active = true;
  robot_homing_done = false;
#endif
}

static inline void HomingGate_MotorStart(Motor_t* motor, int32_t rpm) {
  if (RobotSafety_CanHomingDrive()) {
    Motor_Start(motor);
    Motor_SetSpeed(motor, rpm);
  } else {
    Motor_Stop(motor);
  }
}

extern "C" void Robot_Homing_Tick(void) {
#if (ACTUATOR_HARDWARE_OUTPUT_ENABLED == 0)
  return;
#else
  if (robot_homing_done || bypass_homing) {
    if (bypass_homing) {
       Robot_HandleHomeSwitch(0); // 假觸發以完成剩餘重置動作
    }
    return;
  }

  if (HW_GetMonotonicTimeMs() - homing_start_ms > homing_timeout_ms) {
    robot_homing_active = false;
    RobotSafety_TriggerFault(ROBOT_FAULT_HOMING_TIMEOUT);
    return;
  }
  
  // 先讀取編碼器，讓 Live Expressions 與回授保持更新
  Motor_Update(&motor_joint_13pin);
  Motor_Update(&motor_joint_13pin_2);
  Motor_Update(&motor_joint_13pin_3);

  robot_state.real_theta1 = Motor_GetAngle(&motor_joint_13pin);
  robot_state.real_theta2 = Motor_GetAngle(&motor_joint_13pin_2);
  robot_state.real_theta3 = Motor_GetAngle(&motor_joint_13pin_3);

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

  if (homing_joint1_done && homing_joint2_done && homing_joint3_done) {
    Robot_HandleHomeSwitch(0); // 若三軸皆已完成（可能由中斷觸發），呼叫以收尾
  }

  // 如果開關已經被壓住，即使沒有邊沿中斷，也直接做校正
  if (!homing_joint1_done && HW_IsHomeSwitchTriggered(HW_JOINT_1)) {
    Robot_HandleHomeSwitch(JOINT1_HOME_PIN);
  }
  if (!homing_joint2_done && HW_IsHomeSwitchTriggered(HW_JOINT_2)) {
    Robot_HandleHomeSwitch(JOINT2_HOME_PIN);
  }
  if (!homing_joint3_done && HW_IsHomeSwitchTriggered(HW_JOINT_3)) {
    Robot_HandleHomeSwitch(JOINT3_HOME_PIN);
  }

  // 只讓尚未校正完成的軸持續慢速移動，已校正完成的軸則完全關閉 (Pull Enable Low)
  if (!homing_joint1_done) {
    HomingGate_MotorStart(&motor_joint_13pin, home_search_rpm_m1);
  } else {
    Motor_Stop(&motor_joint_13pin);
  }

  if (!homing_joint2_done) {
    HomingGate_MotorStart(&motor_joint_13pin_2, home_search_rpm_m2);
  } else {
    Motor_Stop(&motor_joint_13pin_2);
  }

  if (!homing_joint3_done) {
    HomingGate_MotorStart(&motor_joint_13pin_3, home_search_rpm_m3);
  } else {
    Motor_Stop(&motor_joint_13pin_3);
  }

  robot_state.real_rpm1 = Motor_GetVelocity(&motor_joint_13pin);
  robot_state.real_rpm2 = Motor_GetVelocity(&motor_joint_13pin_2);
  robot_state.real_rpm3 = Motor_GetVelocity(&motor_joint_13pin_3);
#endif
}
