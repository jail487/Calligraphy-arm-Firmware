/**
 * @file nidec_motor_driver.c
 * @brief Nidec 馬達驅動實作 (v2.0)
 * @details 支援 24H702U030 (Freq) 與 24H220Q231 (PWM)，含編碼器與減速比計算
 */

#include "nidec_motor_driver.h"
#include <math.h>
#include <stdlib.h>

// ==========================================================
// 全域馬達物件實體化
// ==========================================================
Motor_t motor_joint_13pin;
Motor_t motor_joint_13pin_2;
Motor_t motor_joint_13pin_3;

// ==========================================================
// 私有輔助函式 (Helper)
// ==========================================================

static uint32_t Get_Timer_Input_Clock(TIM_HandleTypeDef *htim) {
  // 簡化假設：使用 APB1 Timer (時脈 = PCLK1 * 2)
  return HAL_RCC_GetPCLK1Freq() * 2;
}

// ==========================================================
// 類型 1: 13-Pin 馬達實作 (頻率控制)
// ==========================================================
static void Impl_Freq13Pin_SetSpeed(Motor_t *self, int32_t target_rpm) {
  if (target_rpm == 0) {
    __HAL_TIM_SET_COMPARE(self->config.htim_pwm, self->config.tim_channel, 0);
    return;
  }

  bool cw = (target_rpm >= 0);
  if (self->config.inverted) {
    cw = !cw;
  }
  // 假設 Low = CW
  HAL_GPIO_WritePin(self->config.dir_port, self->config.dir_pin,
                    cw ? GPIO_PIN_RESET : GPIO_PIN_SET);

  uint32_t abs_rpm = (uint32_t)(target_rpm >= 0 ? target_rpm : -target_rpm);
  if (abs_rpm > self->config.max_rpm)
    abs_rpm = self->config.max_rpm;

  // Freq = RPM * 400 / 60
  uint32_t target_freq = (abs_rpm * 400) / 60;
  if (target_freq < 100)
    target_freq = 100;

  uint32_t timer_clk = Get_Timer_Input_Clock(self->config.htim_pwm);
  uint32_t target_arr_full = timer_clk / target_freq;
  uint32_t psc = 0;

  // 為了相容 16-bit Timer (如 TIM3, TIM12)，如果 ARR 超過最大值
  // (65535)，必須調整 Prescaler
  if (target_arr_full > 65535) {
    psc = target_arr_full / 65535;
    target_arr_full = timer_clk / ((psc + 1) * target_freq);
  }
  uint32_t arr = (target_arr_full > 0) ? target_arr_full - 1 : 0;

  __HAL_TIM_SET_PRESCALER(self->config.htim_pwm, psc);
  __HAL_TIM_SET_AUTORELOAD(self->config.htim_pwm, arr);
  __HAL_TIM_SET_COMPARE(self->config.htim_pwm, self->config.tim_channel,
                        arr / 2);

  // 只有在尚未啟動時才呼叫 Start，避免重複呼叫造成 PWM 抖動
  if (self->is_enabled && self->config.htim_pwm->State != HAL_TIM_STATE_BUSY) {
    HAL_TIM_PWM_Start(self->config.htim_pwm, self->config.tim_channel);
  }
}

static void Impl_Freq13Pin_SetEnable(Motor_t *self, bool enable) {
  // START Pin: High = Enable
  HAL_GPIO_WritePin(self->config.enable_port, self->config.enable_pin,
                    enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

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
  // BRAKE Pin: Low = Brake (Stop), High = Run
  HAL_GPIO_WritePin(self->config.enable_port, self->config.enable_pin,
                    enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

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
  motor->measured_velocity_rpm = 0.0f;
  motor->prev_pulse_count = 0;
  motor->last_update_time_ms = HAL_GetTick();
  motor->speed_pid_integral = 0.0f;
  motor->speed_pid_prev_error = 0.0f;
  motor->current_duty_cycle = 0.0f;
  motor->angle_offset_deg = 0.0f;

  if (motor->type == MOTOR_TYPE_PWM_8PIN) {
    motor->config.speed_kp = 0.0001f;
    motor->config.speed_ki = 0.00005f;
    motor->config.speed_kd = 0.0f;
    motor->config.speed_integral_limit = 0.3f;
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
  if (motor->config.htim_encoder == NULL)
    return;

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

  if (dt >= 0.01f) {
    int64_t pulse_delta = motor->total_pulse_count - motor->prev_pulse_count;
    float pulses_per_motor_rev = motor->config.encoder_ppr * 4.0f;
    float output_revs =
        (float)pulse_delta / (pulses_per_motor_rev * motor->config.gear_ratio);
    float raw_rpm = (output_revs / dt) * 60.0f;

    motor->measured_velocity_rpm =
        0.7f * motor->measured_velocity_rpm + 0.3f * raw_rpm;

    motor->prev_pulse_count = motor->total_pulse_count;
    motor->last_update_time_ms = current_time;

    if (motor->type == MOTOR_TYPE_PWM_8PIN && motor->is_enabled) {
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
  if (motor->config.encoder_inverted) {
    return -motor->measured_velocity_rpm;
  }
  return motor->measured_velocity_rpm;
}

void Motor_SetSpeed(Motor_t *motor, int32_t rpm) {
  if (motor == NULL) return;
  if (rpm > (int32_t)motor->config.max_rpm) rpm = (int32_t)motor->config.max_rpm;
  if (rpm < -(int32_t)motor->config.max_rpm) rpm = -(int32_t)motor->config.max_rpm;
  motor->current_rpm_cmd = rpm;
  if (motor->is_enabled && motor->ops && motor->ops->set_speed) {
    motor->ops->set_speed(motor, rpm);
  }
}

void Motor_Start(Motor_t *motor) {
  motor->is_enabled = true;
  if (motor->ops && motor->ops->set_enable) {
    motor->ops->set_enable(motor, true);
  }
  Motor_SetSpeed(motor, motor->current_rpm_cmd);
}

void Motor_Stop(Motor_t *motor) {
  motor->is_enabled = false;
  Motor_SetSpeed(motor, 0);
  if (motor->ops && motor->ops->set_enable) {
    motor->ops->set_enable(motor, false);
  }
}
