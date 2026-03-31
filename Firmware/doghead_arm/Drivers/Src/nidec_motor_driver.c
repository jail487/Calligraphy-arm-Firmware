/**
 * @file nidec_motor_driver.c
 * @brief Nidec 馬達驅動實作 (v2.0)
 * @details 支援 24H702U030 (Freq) 與 24H220Q231 (PWM)，含編碼器與減速比計算
 */

#include "nidec_motor_driver.h"
#include <math.h>
#include <stdlib.h>
// ==========================================================
// 外部 Timer Handle 參照
// 已移至 main.h 中
// ==========================================================

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
    // 假設 Low = CW
    HAL_GPIO_WritePin(self->config.dir_port, self->config.dir_pin,
                      cw ? GPIO_PIN_RESET : GPIO_PIN_SET);

    uint32_t abs_rpm = (uint32_t)(cw ? target_rpm : -target_rpm);
    if (abs_rpm > self->config.max_rpm) abs_rpm = self->config.max_rpm;

    // Freq = RPM * 400 / 60
    uint32_t target_freq = (abs_rpm * 400) / 60;
    if (target_freq < 100) target_freq = 100;

    uint32_t timer_clk = Get_Timer_Input_Clock(self->config.htim_pwm);
    uint32_t target_arr_full = timer_clk / target_freq;
    uint32_t psc = 0;

    // 為了相容 16-bit Timer (如 TIM3, TIM12)，如果 ARR 超過最大值 (65535)，必須調整 Prescaler
    if (target_arr_full > 65535) {
        psc = target_arr_full / 65535;
        target_arr_full = timer_clk / ((psc + 1) * target_freq);
    }
    uint32_t arr = (target_arr_full > 0) ? target_arr_full - 1 : 0;

    __HAL_TIM_SET_PRESCALER(self->config.htim_pwm, psc);
    __HAL_TIM_SET_AUTORELOAD(self->config.htim_pwm, arr);
    __HAL_TIM_SET_COMPARE(self->config.htim_pwm, self->config.tim_channel, arr / 2);

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

const MotorOps_t Ops_Freq13Pin = {
    .set_speed  = Impl_Freq13Pin_SetSpeed,
    .set_enable = Impl_Freq13Pin_SetEnable
};


// ==========================================================
// 類型 2: 8-Pin 馬達實作 (PWM Duty 控制)
// ==========================================================
static void Impl_PWM8Pin_SetSpeed(Motor_t *self, int32_t target_rpm) {
    bool cw = (target_rpm >= 0);
    // 假設 High = CW
    HAL_GPIO_WritePin(self->config.dir_port, self->config.dir_pin,
                      cw ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // [New Architecture] 僅記錄指令，透過 Motor_Update 進行 PID 閉迴圈控制
    self->current_rpm_cmd = target_rpm * (cw ? 1 : -1); 
    
    // 首次呼叫先不輸出 PWM (Initial State)，由 Control Loop 接手
    // 但為了向前相容，可以先給一個 Feedforward value (Open Loop Estimate)
    // 這裡我們選擇：不直接寫 PWM，而是讓 PID Loop 在下一次 Update 時處理
    // 這樣可以避免兩個地方同時寫入計時器造成衝突
}

static void Impl_PWM8Pin_SetEnable(Motor_t *self, bool enable) {
    // BRAKE Pin: Low = Brake (Stop), High = Run
    HAL_GPIO_WritePin(self->config.enable_port, self->config.enable_pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (enable) {
        HAL_TIM_PWM_Start(self->config.htim_pwm, self->config.tim_channel);
    } else {
        // Disable 時將 PWM Duty 設為 0
        // [Fix] Inverted Logic: Stop = Speed 0 = Duty 0 = Motor Low.
        // HW Inverts: Motor Low = MCU High (100% Duty).
        uint32_t period = __HAL_TIM_GET_AUTORELOAD(self->config.htim_pwm);
        __HAL_TIM_SET_COMPARE(self->config.htim_pwm, self->config.tim_channel, period);
        
        // [New] 重置 PID 積分項，避免重啟時暴衝
        self->speed_pid_integral = 0.0f;
        self->speed_pid_prev_error = 0.0f;
    }
}

const MotorOps_t Ops_PWM8Pin = {
    .set_speed  = Impl_PWM8Pin_SetSpeed,
    .set_enable = Impl_PWM8Pin_SetEnable
};


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

    // [Fix] 針對 Auto-reload preload (ARPE) 的修正
    // 確保 Shadow Register 的值能立即更新到 Active Register，解決啟動時馬達不動的問題
    if (motor->config.htim_pwm != NULL) {
        // 若為頻率控制馬達 (13-Pin)，ARR 決定頻率，需初始化為有效值 (如最低頻率)
        if (motor->type == MOTOR_TYPE_FREQ_13PIN) {
            uint32_t timer_clk = Get_Timer_Input_Clock(motor->config.htim_pwm);
            // 設定初始頻率為 100Hz (參考 SetSpeed 邏輯)
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
            // 設定 Duty Cycle 50%
            __HAL_TIM_SET_COMPARE(motor->config.htim_pwm, motor->config.tim_channel, initial_arr / 2);
        }
        
        // 關鍵：產生 Update Event (UG)
        // 強制將 Preload Register (ARR, CCRx) 的值載入 Active Register
        // 使用 HAL 庫標準函式替代直接暫存器操作
        HAL_TIM_GenerateEvent(motor->config.htim_pwm, TIM_EVENTSOURCE_UPDATE);
        
        // 清除因 UG 觸發的 Update Interrupt Flag
        __HAL_TIM_CLEAR_FLAG(motor->config.htim_pwm, TIM_FLAG_UPDATE);
    }

    // 初始化編碼器狀態
    motor->total_pulse_count = 0;
    motor->last_counter_val = 0;
    
    // 初始化速度反饋變數
    motor->measured_velocity_rpm = 0.0f;
    motor->prev_pulse_count = 0;
    motor->last_update_time_ms = HAL_GetTick();

    // 初始化 PID 狀態
    motor->speed_pid_integral = 0.0f;
    motor->speed_pid_prev_error = 0.0f;
    motor->current_duty_cycle = 0.0f;

    // [New] 設定預設 PID 參數 (可調)
    if (motor->type == MOTOR_TYPE_PWM_8PIN) {
        motor->config.speed_kp = 0.0001f;     // 每 RPM 誤差增加 0.01% Duty
        motor->config.speed_ki = 0.00005f;    // 積分項
        motor->config.speed_kd = 0.0f;
        motor->config.speed_integral_limit = 0.3f; // 積分上限佔 30% Duty
    }

    // 啟動硬體 Timer 的 Encoder Mode
    if (motor->config.htim_encoder != NULL) {
        HAL_TIM_Encoder_Start(motor->config.htim_encoder, TIM_CHANNEL_ALL);
        // 重置硬體計數器為 0 (或中間值，視習慣而定，這裡設 0)
        __HAL_TIM_SET_COUNTER(motor->config.htim_encoder, 0);
    }

    if (motor->ops && motor->ops->set_enable) {
        motor->ops->set_enable(motor, false);
    }
}

// [重要功能] 讀取編碼器並處理溢位 (Wrap-around)
void Motor_Update(Motor_t *motor) {
    if (motor->config.htim_encoder == NULL) return;

    // 1. 讀取當前硬體計數器
    uint32_t current_cnt = __HAL_TIM_GET_COUNTER(motor->config.htim_encoder);
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(motor->config.htim_encoder); // 也就是 ARR

    // 2. 計算與上次的變化量 (Delta)
    int32_t delta = (int32_t)current_cnt - (int32_t)motor->last_counter_val;

    // 3. 處理溢位 (Handle Overflow/Underflow)
    // 假設變化量不會瞬間超過半圈 (ARR/2)。如果超過，代表發生了溢位。
    // 例如 ARR=65535, 從 65530 變成 5 -> delta = -65525 -> 實際是 +10
    if (delta > (int32_t)(period / 2)) {
        delta -= (period + 1);
    } else if (delta < -(int32_t)(period / 2)) {
        delta += (period + 1);
    }

    // 4. 更新累計值與記錄
    motor->total_pulse_count += delta;
    motor->last_counter_val = current_cnt;
    
    // 5. [新增] 計算速度反饋
    uint32_t current_time = HAL_GetTick();  // 取得系統時間 (ms)
    float dt = (current_time - motor->last_update_time_ms) / 1000.0f;  // 轉換為秒
    
    // 增加時間間隔門檻，避免 dt 太小導致速度計算暴衝
    // 10ms 計算一次速度比較穩定
    if (dt >= 0.01f) {  
        // 計算脈衝變化量
        int64_t pulse_delta = motor->total_pulse_count - motor->prev_pulse_count;
        
        // STM32 Encoder Mode x4: 馬達轉一圈 = PPR * 4 脈衝
        float pulses_per_motor_rev = motor->config.encoder_ppr * 4.0f;
        
        // 馬達軸轉數 (未經過減速比)
        // 注意：這裡計算的是「輸出軸」轉速，所以要除以減速比
        // 如果要算馬達軸轉速，就不除以 gear_ratio
        float output_revs = (float)pulse_delta / (pulses_per_motor_rev * motor->config.gear_ratio);
        
        // 轉換為 RPM (revolutions per minute)
        float raw_rpm = (output_revs / dt) * 60.0f;
        
        // [New] 加入低通濾波 (LPF) 以平滑速度數值
        // Alpha = 0.3 (30% 新值, 70% 舊值)
        // 解決低解析度編碼器在低速時的數值跳動問題
        motor->measured_velocity_rpm = 0.7f * motor->measured_velocity_rpm + 0.3f * raw_rpm;
        
        // 儲存當前值供下次使用
        motor->prev_pulse_count = motor->total_pulse_count;
        motor->last_update_time_ms = current_time;

        // 6. [New] 閉迴圈速度控制 (僅 8-Pin PWM 模式)
        // 透過 PID 調整 PWM Duty，解決非線性與負載效應問題
        if (motor->type == MOTOR_TYPE_PWM_8PIN && motor->is_enabled) {
            float target_rpm = (float)abs(motor->current_rpm_cmd);
            float current_rpm = (float)fabs(motor->measured_velocity_rpm);
            
            // 計算誤差 (總是正值，方向由 GPIO 控制)
            float error = target_rpm - current_rpm;
            
            // 比例項 (P)
            float p_out = motor->config.speed_kp * error;
            
            // 積分項 (I)
            motor->speed_pid_integral += motor->config.speed_ki * error * dt;
            // 積分限幅
            if (motor->speed_pid_integral > motor->config.speed_integral_limit) 
                motor->speed_pid_integral = motor->config.speed_integral_limit;
            if (motor->speed_pid_integral < -motor->config.speed_integral_limit) 
                motor->speed_pid_integral = -motor->config.speed_integral_limit;

            // 前饋控制 (Feedforward) - 基礎 Duty Estimation
            // 保留原本的線性映射與死區補償作為基礎值
            float deadzone = 0.12f;
            float ff_duty = 0.0f;
            if (target_rpm > 10.0f) { // 設一個小門檻避免 0 速抖動
                 ff_duty = deadzone + (1.0f - deadzone) * (target_rpm / (float)motor->config.max_rpm);
            }

            // 總輸出 duty = FF + P + I
            float duty_out = ff_duty + p_out + motor->speed_pid_integral;
            
            // 輸出限幅 (0.0 ~ 1.0)
            if (duty_out > 1.0f) duty_out = 1.0f;
            if (duty_out < 0.0f) duty_out = 0.0f;
            
            motor->current_duty_cycle = duty_out;

            // 寫入硬體 PWM 暫存器
            uint32_t period = __HAL_TIM_GET_AUTORELOAD(motor->config.htim_pwm);
            // [Fix] Inverted Logic: MCU Low = Motor Run
            // Motor needs High Duty (Run). MCU Low corresponds to Motor High.
            // So we need Low Duty Output from MCU.
            // CCR high -> Low Output (Mode 1). No.
            // Mode 1: CNT < CCR ? High : Low.
            // We want 'duty_out' amount of HIGH at the Motor.
            // If HW Inverts: Motor High = MCU Low.
            // So we want 'duty_out' amount of LOW at MCU.
            // 'duty_out' amount of LOW means '1-duty_out' amount of High.
            // MCU generates '1-duty_out' High.
            // So CCR = Period * (1.0 - duty_out).
            uint32_t ccr_val = (uint32_t)(period * (1.0f - duty_out));
            __HAL_TIM_SET_COMPARE(motor->config.htim_pwm, motor->config.tim_channel, ccr_val);
        }
    }
}

float Motor_GetAngle(Motor_t *motor) {
    // 檢查除數是否為 0
    if (motor->config.gear_ratio == 0.0f || motor->config.encoder_ppr == 0.0f) {
        return 0.0f;
    }

    // STM32 Encoder Mode x4 模式 (上下數都計數)
    // 馬達轉一圈的 Pulse 數 = PPR * 4
    float pulses_per_motor_rev = motor->config.encoder_ppr * 4.0f;

    // 總輸出軸轉數 = 總 Pulse / (馬達每圈Pulse * 減速比)
    // [Modified] 移除負號，恢復正向計數 (假設 Pulse 增加代表正轉)
    float output_revs = (float)motor->total_pulse_count / (pulses_per_motor_rev * motor->config.gear_ratio);

    // 轉換為角度
    return output_revs * 360.0f;
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

float Motor_GetVelocity(Motor_t *motor) {
    return motor->measured_velocity_rpm;
}

void Motor_SetSpeed(Motor_t *motor, int32_t rpm) {
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

// ==========================================================
// 4. 系統配置函式
// ==========================================================

void Motor_System_Config(void) {

    // --- 13-Pin 馬達配置 ---
    motor_joint_13pin.type = MOTOR_TYPE_FREQ_13PIN;

    // 速度 Timer
    motor_joint_13pin.config.htim_pwm = &htim2;
    motor_joint_13pin.config.tim_channel = TIM_CHANNEL_1; // PA5
    // [新增] Encoder Timer (假設用 TIM4)
    motor_joint_13pin.config.htim_encoder = &htim1; // PA8, PA9
    motor_joint_13pin.config.enable_port = GPIOC;
    motor_joint_13pin.config.enable_pin = GPIO_PIN_4; // START
    motor_joint_13pin.config.dir_port = GPIOC;
    motor_joint_13pin.config.dir_pin = GPIO_PIN_1;    // DIR

    // 參數
    motor_joint_13pin.config.max_rpm = 6000;
    motor_joint_13pin.config.encoder_ppr = 100.0f;  // [已確認] 規格為 100 PPR
    motor_joint_13pin.config.gear_ratio = 1.0f;    // [已確認] 無減速機，比率 1:1

    Motor_Init(&motor_joint_13pin);

    // --- 13-Pin 馬達配置 ---
    motor_joint_13pin_2.type = MOTOR_TYPE_FREQ_13PIN;

    // 速度 Timer
    motor_joint_13pin_2.config.htim_pwm = &htim3; // PB4
    motor_joint_13pin_2.config.tim_channel = TIM_CHANNEL_1; // PB4

    // [新增] Encoder Timer (用 TIM4)
    motor_joint_13pin_2.config.htim_encoder = &htim4; // PA8, PA9

    // GPIO
    motor_joint_13pin_2.config.enable_port = GPIOC;
    motor_joint_13pin_2.config.enable_pin = GPIO_PIN_4; // BRAKE
    motor_joint_13pin_2.config.dir_port = GPIOC;
    motor_joint_13pin_2.config.dir_pin = GPIO_PIN_2;    // DIR

    // 參數
    motor_joint_13pin_2.config.max_rpm = 6000;
    motor_joint_13pin_2.config.encoder_ppr = 100.0f; // [已確認] 規格為 100 PPR
    motor_joint_13pin_2.config.gear_ratio = 1.0f;   // [已確認] 無減速機，比率 1:1

    Motor_Init(&motor_joint_13pin_2);

    // --- 第三顆 13-Pin 馬達配置 ---
    motor_joint_13pin_3.type = MOTOR_TYPE_FREQ_13PIN;

    // 速度 Timer (TIM13 Channel 1)
    motor_joint_13pin_3.config.htim_pwm = &htim12; 
    motor_joint_13pin_3.config.tim_channel = TIM_CHANNEL_1; 
    // Encoder Timer (TIM5)
    motor_joint_13pin_3.config.htim_encoder = &htim5; 

    // GPIO
    motor_joint_13pin_3.config.enable_port = GPIOC; 
    motor_joint_13pin_3.config.enable_pin = GPIO_PIN_0; 
    motor_joint_13pin_3.config.dir_port = GPIOC;    
    motor_joint_13pin_3.config.dir_pin = GPIO_PIN_3;                            
    // 參數
    motor_joint_13pin_3.config.max_rpm = 6000;
    motor_joint_13pin_3.config.encoder_ppr = 100.0f;
    motor_joint_13pin_3.config.gear_ratio = 1.0f;

    Motor_Init(&motor_joint_13pin_3);

}
