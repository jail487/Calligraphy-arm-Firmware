/**
 * @file nidec_motor_driver.c
 * @brief Nidec 馬達驅動實作 (v2.0)
 * @details 支援 24H702U030 (Freq) 與 24H220Q231 (PWM)，含編碼器與減速比計算
 */

#include "nidec_motor_driver.h"
#include "motor_parameters.h"
#include "joint_units.h"
#include <math.h>
#include <stdlib.h>

// The driver repeats the runtime actuator gate because Motor_Update() runs
// before the application loop's final gate.  Weak host fallbacks keep the
// standalone C unit tests linkable; firmware links the strong C++ functions.
extern bool RobotSafety_CanDrive(void);
extern bool RobotSafety_CanHomingDrive(void);
#if defined(HOST_TEST) && defined(__GNUC__)
__attribute__((weak)) bool RobotSafety_CanDrive(void) { return false; }
__attribute__((weak)) bool RobotSafety_CanHomingDrive(void) { return false; }
#endif

static bool Motor_PhysicalOutputPermitted(void) {
#if (ACTUATOR_HARDWARE_OUTPUT_ENABLED == 0)
  return false;
#else
  return RobotSafety_CanDrive() || RobotSafety_CanHomingDrive();
#endif
}

// Software baseline: the 1 kHz control task refreshes the estimator at 10 ms
// windows.  Actual loaded-speed adequacy still requires physical HIL data.
#define MOTOR_VELOCITY_UPDATE_INTERVAL_S 0.010f
#define MOTOR_VELOCITY_FILTER_NEW_WEIGHT 0.30f

// ==========================================================
// 私有輔助函式 (Helper)
// ==========================================================

static uint32_t Get_Timer_Input_Clock(TIM_HandleTypeDef *htim) {
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  if (pclk1 == 0) {
    pclk1 = 45000000U;
  }
  return pclk1 * 2;
}

float Motor_ComputeVelocityRpmFromPulses(int64_t pulse_delta,
                                         float dt_seconds,
                                         float encoder_ppr,
                                         float gear_ratio) {
  if (!isfinite(dt_seconds) || dt_seconds <= 0.0f ||
      !isfinite(encoder_ppr) || encoder_ppr <= 0.0f ||
      !isfinite(gear_ratio) || gear_ratio <= 0.0f) {
    return 0.0f;
  }

  const float pulses_per_motor_rev = encoder_ppr * 4.0f;
  if (!isfinite(pulses_per_motor_rev) || pulses_per_motor_rev <= 0.0f) {
    return 0.0f;
  }

  const float output_revs =
      (float)pulse_delta / (pulses_per_motor_rev * gear_ratio);
  const float rpm = (output_revs / dt_seconds) * 60.0f;
  return isfinite(rpm) ? rpm : 0.0f;
}

// ==========================================================
// 類型 1: 13-Pin 馬達實作 (頻率控制)
// ==========================================================
static void Impl_Freq13Pin_SetSpeed(Motor_t *self, int32_t target_rpm) {
  if (target_rpm == 0) {
    __HAL_TIM_SET_COMPARE(self->config.htim_pwm, self->config.tim_channel, 0);
    HAL_TIM_PWM_Stop(self->config.htim_pwm, self->config.tim_channel);
    return;
  }

  bool cw = (target_rpm >= 0);
  if (self->config.inverted) {
    cw = !cw;
  }
  // Low = CW, High = CCW
  HAL_GPIO_WritePin(self->config.dir_port, self->config.dir_pin,
                    cw ? GPIO_PIN_RESET : GPIO_PIN_SET);

  uint32_t abs_rpm = (uint32_t)(target_rpm >= 0 ? target_rpm : -target_rpm);
  float gear_ratio = (self->config.gear_ratio > 0.0f) ? self->config.gear_ratio : 1.0f;
  float ppr = 400.0f; // MOTOR_PULSES_PER_REV
  uint32_t target_freq = (uint32_t)Joint_RPM_To_Command_Freq_Hz((float)abs_rpm, gear_ratio, ppr);
  if (target_freq < 100)
    target_freq = 100;

  uint32_t timer_clk = Get_Timer_Input_Clock(self->config.htim_pwm);
  uint32_t target_arr_full = timer_clk / target_freq;
  uint32_t psc = 0;

  // 為了相容 16-bit Timer (如 TIM3, TIM12)，如果 ARR 超過最大值 (65535)，調整 Prescaler
  if (target_arr_full > 65535) {
    psc = target_arr_full / 65535;
    target_arr_full = timer_clk / ((psc + 1) * target_freq);
  }
  uint32_t arr = (target_arr_full > 0) ? target_arr_full - 1 : 0;

  if (self->config.htim_pwm->Instance->ARR != arr ||
      self->config.htim_pwm->Instance->PSC != psc) {
    __HAL_TIM_SET_PRESCALER(self->config.htim_pwm, psc);
    __HAL_TIM_SET_AUTORELOAD(self->config.htim_pwm, arr);
    HAL_TIM_GenerateEvent(self->config.htim_pwm, TIM_EVENTSOURCE_UPDATE);
  }
  __HAL_TIM_SET_COMPARE(self->config.htim_pwm, self->config.tim_channel, (arr + 1) / 2);
  HAL_TIM_PWM_Start(self->config.htim_pwm, self->config.tim_channel);
}

static void Impl_Freq13Pin_SetEnable(Motor_t *self, bool enable) {
  // START Pin: High = Enable (若啟用腳位存在才寫入)
  if (self->config.enable_port != NULL) {
    HAL_GPIO_WritePin(self->config.enable_port, self->config.enable_pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  if (enable) {
    HAL_TIM_PWM_Start(self->config.htim_pwm, self->config.tim_channel);
  } else {
    HAL_TIM_PWM_Stop(self->config.htim_pwm, self->config.tim_channel);
  }
}

const MotorOps_t Ops_Freq13Pin = {.set_speed = Impl_Freq13Pin_SetSpeed,
                                  .set_enable = Impl_Freq13Pin_SetEnable};

// ==========================================================
// 類型 2: 8-Pin 馬達實作 (PWM Duty 控制)
// ==========================================================
static void Impl_PWM8Pin_SetSpeed(Motor_t *self, int32_t target_rpm) {
  bool cw = (target_rpm >= 0);
  if (self->config.inverted) {
    cw = !cw;
  }
  // 假設 High = CW
  HAL_GPIO_WritePin(self->config.dir_port, self->config.dir_pin,
                    cw ? GPIO_PIN_SET : GPIO_PIN_RESET);

  self->current_rpm_cmd = target_rpm >= 0 ? target_rpm : -target_rpm;
}

static void Impl_PWM8Pin_SetEnable(Motor_t *self, bool enable) {
  // BRAKE Pin: Low = Brake (Stop), High = Run (若啟用腳位存在才寫入)
  if (self->config.enable_port != NULL) {
    HAL_GPIO_WritePin(self->config.enable_port, self->config.enable_pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  if (enable) {
    HAL_TIM_PWM_Start(self->config.htim_pwm, self->config.tim_channel);
  } else {
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(self->config.htim_pwm);
    __HAL_TIM_SET_COMPARE(self->config.htim_pwm, self->config.tim_channel,
                          period);

    self->speed_pid_integral = 0.0f;
    self->speed_pid_prev_error = 0.0f;
  }
}

const MotorOps_t Ops_PWM8Pin = {.set_speed = Impl_PWM8Pin_SetSpeed,
                                .set_enable = Impl_PWM8Pin_SetEnable};

// ==========================================================
// 3. 公開 API 實作
// ==========================================================

void Motor_Init(Motor_t *motor) {
  if (motor->type == MOTOR_TYPE_FREQ_13PIN) {
    motor->ops = &Ops_Freq13Pin;
  } else if (motor->type == MOTOR_TYPE_PWM_8PIN) {
    motor->ops = &Ops_PWM8Pin;
  }

  motor->is_enabled = false;
  motor->current_rpm_cmd = 0;

  if (motor->config.htim_pwm != NULL) {
    if (motor->type == MOTOR_TYPE_FREQ_13PIN) {
      uint32_t timer_clk = Get_Timer_Input_Clock(motor->config.htim_pwm);
      uint32_t target_freq = 100;
      uint32_t target_arr_full = timer_clk / target_freq;
      uint32_t psc = 0;

      if (target_arr_full > 65535) {
        psc = target_arr_full / 65535;
        target_arr_full = timer_clk / ((psc + 1) * target_freq);
      }
      uint32_t initial_arr = (target_arr_full > 0) ? target_arr_full - 1 : 0;

      __HAL_TIM_SET_PRESCALER(motor->config.htim_pwm, psc);
      __HAL_TIM_SET_AUTORELOAD(motor->config.htim_pwm, initial_arr);
      __HAL_TIM_SET_COMPARE(motor->config.htim_pwm, motor->config.tim_channel,
                            initial_arr / 2);
    }

    HAL_TIM_GenerateEvent(motor->config.htim_pwm, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(motor->config.htim_pwm, TIM_FLAG_UPDATE);
  }

  motor->total_pulse_count = 0;
  motor->last_counter_val = 0;
  motor->raw_velocity_rpm = 0.0f;
  motor->measured_velocity_rpm = 0.0f;
  motor->prev_pulse_count = 0;
  motor->last_update_time_ms = HAL_GetTick();
  motor->speed_pid_integral = 0.0f;
  motor->speed_pid_prev_error = 0.0f;
  motor->current_duty_cycle = 0.0f;
  motor->angle_offset_deg = 0.0f;

  if (motor->type == MOTOR_TYPE_PWM_8PIN) {
    if (motor->config.speed_kp == 0.0f && motor->config.speed_ki == 0.0f) {
      motor->config.speed_kp = 0.0001f;
      motor->config.speed_ki = 0.00005f;
      motor->config.speed_kd = 0.0f;
      motor->config.speed_integral_limit = 0.3f;
    }
  }

  if (motor->config.htim_encoder != NULL) {
    HAL_TIM_Encoder_Start(motor->config.htim_encoder, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(motor->config.htim_encoder, 0);
  }

  if (motor->ops && motor->ops->set_enable) {
    motor->ops->set_enable(motor, false);
  }
}

void Motor_Update(Motor_t *motor) {
  if (motor == NULL) return;
  if (motor->config.htim_encoder == NULL)
    return;

  // Safety can transition between two 1 kHz iterations.  Force the physical
  // output low here as well, so an old PWM compare value cannot remain active
  // until the application-level final gate runs again.
  if (!Motor_PhysicalOutputPermitted() && motor->is_enabled) {
    motor->is_enabled = false;
    motor->current_rpm_cmd = 0;
    motor->current_duty_cycle = 0.0f;
    if (motor->config.htim_pwm != NULL) {
      __HAL_TIM_SET_COMPARE(motor->config.htim_pwm, motor->config.tim_channel, 0U);
    }
    if (motor->ops && motor->ops->set_speed) {
      motor->ops->set_speed(motor, 0);
    }
    if (motor->ops && motor->ops->set_enable) {
      motor->ops->set_enable(motor, false);
    }
  }

  uint32_t current_cnt = __HAL_TIM_GET_COUNTER(motor->config.htim_encoder);
  uint32_t period =
      __HAL_TIM_GET_AUTORELOAD(motor->config.htim_encoder);

  int32_t delta = (int32_t)current_cnt - (int32_t)motor->last_counter_val;

  if (delta > (int32_t)(period / 2)) {
    delta -= (period + 1);
  } else if (delta < -(int32_t)(period / 2)) {
    delta += (period + 1);
  }

  motor->total_pulse_count += delta;
  motor->last_counter_val = current_cnt;

  uint32_t current_time = HAL_GetTick();
  float dt = (current_time - motor->last_update_time_ms) / 1000.0f;

  if (dt >= MOTOR_VELOCITY_UPDATE_INTERVAL_S) {
    int64_t pulse_delta = motor->total_pulse_count - motor->prev_pulse_count;
    float raw_rpm = Motor_ComputeVelocityRpmFromPulses(
        pulse_delta, dt, motor->config.encoder_ppr, motor->config.gear_ratio);

    motor->raw_velocity_rpm = raw_rpm;
    motor->measured_velocity_rpm =
        (1.0f - MOTOR_VELOCITY_FILTER_NEW_WEIGHT) *
            motor->measured_velocity_rpm +
        MOTOR_VELOCITY_FILTER_NEW_WEIGHT * raw_rpm;

    motor->prev_pulse_count = motor->total_pulse_count;
    motor->last_update_time_ms = current_time;

    // A stale is_enabled flag must not refresh PWM while SAFE, FAULT, or
    // ESTOP is active.
    if (motor->type == MOTOR_TYPE_PWM_8PIN && motor->is_enabled &&
        Motor_PhysicalOutputPermitted()) {
      float target_rpm = (float)abs(motor->current_rpm_cmd);
      float current_rpm = (float)fabs(motor->measured_velocity_rpm);
      float error = target_rpm - current_rpm;
      float p_out = motor->config.speed_kp * error;

      motor->speed_pid_integral += motor->config.speed_ki * error * dt;
      if (motor->speed_pid_integral > motor->config.speed_integral_limit)
        motor->speed_pid_integral = motor->config.speed_integral_limit;
      if (motor->speed_pid_integral < -motor->config.speed_integral_limit)
        motor->speed_pid_integral = -motor->config.speed_integral_limit;

      float deadzone = 0.12f;
      float ff_duty = 0.0f;
      if (target_rpm > 10.0f) {
        ff_duty = deadzone + (1.0f - deadzone) *
                                 (target_rpm / (float)motor->config.max_rpm);
      }

      float duty_out = ff_duty + p_out + motor->speed_pid_integral;
      if (duty_out > 1.0f)
        duty_out = 1.0f;
      if (duty_out < 0.0f)
        duty_out = 0.0f;

      motor->current_duty_cycle = duty_out;

      uint32_t period_pwm = __HAL_TIM_GET_AUTORELOAD(motor->config.htim_pwm);
      uint32_t ccr_val = (uint32_t)(period_pwm * (1.0f - duty_out));
      __HAL_TIM_SET_COMPARE(motor->config.htim_pwm, motor->config.tim_channel,
                            ccr_val);
    }
  }
}

float Motor_GetAngle(Motor_t *motor) {
  if (motor->config.gear_ratio == 0.0f || motor->config.encoder_ppr == 0.0f) {
    return 0.0f;
  }

  float pulses_per_motor_rev = motor->config.encoder_ppr * 4.0f;
  float output_revs = (float)motor->total_pulse_count /
                      (pulses_per_motor_rev * motor->config.gear_ratio);
  float angle_deg = output_revs * 360.0f;

  if (motor->config.encoder_inverted) {
    angle_deg = -angle_deg;
  }

  return angle_deg + motor->angle_offset_deg;
}

void Motor_ResetEncoder(Motor_t *motor) {
  motor->total_pulse_count = 0;
  motor->prev_pulse_count = 0;
  motor->raw_velocity_rpm = 0.0f;
  motor->measured_velocity_rpm = 0.0f;
  motor->last_update_time_ms = HAL_GetTick();
  if (motor->config.htim_encoder != NULL) {
    __HAL_TIM_SET_COUNTER(motor->config.htim_encoder, 0);
    motor->last_counter_val = 0;
  }
}

void Motor_CalibrateAngle(Motor_t *motor, float target_deg) {
  if (motor->config.gear_ratio == 0.0f || motor->config.encoder_ppr == 0.0f) {
    motor->angle_offset_deg = target_deg;
    return;
  }

  float pulses_per_motor_rev = motor->config.encoder_ppr * 4.0f;
  float raw_output_revs = (float)motor->total_pulse_count /
                          (pulses_per_motor_rev * motor->config.gear_ratio);
  float raw_angle_deg = raw_output_revs * 360.0f;
  if (motor->config.encoder_inverted) {
    raw_angle_deg = -raw_angle_deg;
  }

  motor->angle_offset_deg = target_deg - raw_angle_deg;
}

void Motor_SetAngleOffset(Motor_t *motor, float offset_deg) {
  motor->angle_offset_deg = offset_deg;
}

float Motor_GetVelocity(Motor_t *motor) {
  if (motor == NULL || !isfinite(motor->measured_velocity_rpm)) {
    return 0.0f;
  }
  if (motor->config.encoder_inverted) {
    return -motor->measured_velocity_rpm;
  }
  return motor->measured_velocity_rpm;
}

float Motor_GetRawVelocity(Motor_t *motor) {
  if (motor == NULL || !isfinite(motor->raw_velocity_rpm)) {
    return 0.0f;
  }
  if (motor->config.encoder_inverted) {
    return -motor->raw_velocity_rpm;
  }
  return motor->raw_velocity_rpm;
}

void Motor_SetSpeed(Motor_t *motor, int32_t rpm) {
  if (motor == NULL) return;

#if (ACTUATOR_HARDWARE_OUTPUT_ENABLED == 0)
  rpm = 0;
#else
  if (rpm != 0 && !Motor_PhysicalOutputPermitted()) {
    rpm = 0;
  }
#endif

  float gear_ratio = (motor->config.gear_ratio > 0.0f) ? motor->config.gear_ratio : 1.0f;
  if (motor->config.max_rpm > 0) {
    float max_joint_rpm = (float)motor->config.max_rpm / gear_ratio;
    if (fabsf((float)rpm) > max_joint_rpm) {
      rpm = (rpm >= 0) ? (int32_t)max_joint_rpm : -(int32_t)max_joint_rpm;
    }
  }
  motor->current_rpm_cmd = rpm;

#if (ACTUATOR_HARDWARE_OUTPUT_ENABLED != 0)
  if (motor->is_enabled && motor->ops && motor->ops->set_speed) {
    motor->ops->set_speed(motor, rpm);
  }
#else
  if (motor->ops && motor->ops->set_speed) {
    motor->ops->set_speed(motor, 0);
  }
#endif
}

void Motor_Start(Motor_t *motor) {
  if (motor == NULL) return;

#if (ACTUATOR_HARDWARE_OUTPUT_ENABLED != 0)
  if (!Motor_PhysicalOutputPermitted()) {
    motor->is_enabled = false;
    if (motor->ops && motor->ops->set_speed) {
      motor->ops->set_speed(motor, 0);
    }
    if (motor->ops && motor->ops->set_enable) {
      motor->ops->set_enable(motor, false);
    }
    return;
  }
  motor->is_enabled = true;
  if (motor->ops && motor->ops->set_enable) {
    motor->ops->set_enable(motor, true);
  }
  Motor_SetSpeed(motor, motor->current_rpm_cmd);
#else
  motor->is_enabled = false;
  if (motor->ops && motor->ops->set_enable) {
    motor->ops->set_enable(motor, false);
  }
  motor->current_rpm_cmd = 0;
  if (motor->ops && motor->ops->set_speed) {
    motor->ops->set_speed(motor, 0);
  }
#endif
}

#ifdef HOST_TEST
static volatile uint32_t g_motor_stop_count = 0U;

void Motor_ResetStopCallCount(void) {
  g_motor_stop_count = 0U;
}

uint32_t Motor_GetStopCallCount(void) {
  return g_motor_stop_count;
}
#endif

void Motor_Stop(Motor_t *motor) {
  if (motor == NULL) return;
#ifdef HOST_TEST
  g_motor_stop_count++;
#endif
  motor->is_enabled = false;
  Motor_SetSpeed(motor, 0);
  if (motor->ops && motor->ops->set_enable) {
    motor->ops->set_enable(motor, false);
  }
}
