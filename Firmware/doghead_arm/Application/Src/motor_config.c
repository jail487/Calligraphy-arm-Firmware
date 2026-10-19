/**
 * @file motor_config.c
 * @brief 系統馬達腳位與硬體配置實作 (Application Layer)
 */

#include "motor_config.h"
#include "motor_parameters.h"

// ==========================================================
// 全域馬達物件實體化 (Application Layer)
// ==========================================================
Motor_t motor_joint_13pin;
Motor_t motor_joint_13pin_2;
Motor_t motor_joint_13pin_3;

void Motor_System_Config(void) {

    // --- 13-Pin 馬達配置 (Joint 1 / M1 硬體) ---
    motor_joint_13pin.type = MOTOR_TYPE_FREQ_13PIN;

    // 速度 Timer (M1 硬體)
    motor_joint_13pin.config.htim_pwm = &htim2;
    motor_joint_13pin.config.tim_channel = TIM_CHANNEL_1; // PA5 (M1 PWM)
    // Encoder Timer (M1 Encoder)
    motor_joint_13pin.config.htim_encoder = &htim1; // PA8, PA9 (M1 Encoder)
    motor_joint_13pin.config.enable_port = NULL; // START Pin 已由硬體固定為 5V；不由 MCU 控制
    motor_joint_13pin.config.enable_pin = 0;
    motor_joint_13pin.config.dir_port = M1_DIR_PORT;
    motor_joint_13pin.config.dir_pin = M1_DIR_PIN; // DIR

    // 參數
    motor_joint_13pin.config.max_rpm = 5000; // User-confirmed motor-shaft ceiling
    motor_joint_13pin.config.encoder_ppr = ENCODER_PPR_AXIS1;
    motor_joint_13pin.config.gear_ratio = GEAR_RATIO_AXIS1;
    motor_joint_13pin.config.inverted = true; // 設為 true 讓正速度指令朝後旋轉 (校正方向)
    motor_joint_13pin.config.encoder_inverted = false; // 修正：角度朝手臂後方旋轉時累加方向為正

    Motor_Init(&motor_joint_13pin);

    // --- 13-Pin 馬達配置 (Joint 2 / M2 硬體) ---
    motor_joint_13pin_2.type = MOTOR_TYPE_FREQ_13PIN;

    // 速度 Timer (M2 硬體)
    motor_joint_13pin_2.config.htim_pwm = &htim3;           // PB4
    motor_joint_13pin_2.config.tim_channel = TIM_CHANNEL_1; // PB4

    // Encoder Timer (用 TIM4)
    motor_joint_13pin_2.config.htim_encoder = &htim4; // PA8, PA9

    // GPIO
    motor_joint_13pin_2.config.enable_port = NULL; // START Pin 已由硬體固定為 5V；不由 MCU 控制
    motor_joint_13pin_2.config.enable_pin = 0;
    motor_joint_13pin_2.config.dir_port = M2_DIR_PORT;
    motor_joint_13pin_2.config.dir_pin = M2_DIR_PIN; // DIR

    // 參數
    motor_joint_13pin_2.config.max_rpm = 5000; // User-confirmed motor-shaft ceiling
    motor_joint_13pin_2.config.encoder_ppr = ENCODER_PPR_AXIS2;
    motor_joint_13pin_2.config.gear_ratio = GEAR_RATIO_AXIS2;
    motor_joint_13pin_2.config.inverted = false; // 設為 false 讓正向轉動朝手臂後方旋轉 (校正方向)
    motor_joint_13pin_2.config.encoder_inverted = false; // 預設編碼器方向正常

    Motor_Init(&motor_joint_13pin_2);

    // --- 第三顆 13-Pin 馬達配置 (Joint 3 / M3 硬體) ---
    motor_joint_13pin_3.type = MOTOR_TYPE_FREQ_13PIN;

    // 速度 Timer (M3 硬體)
    motor_joint_13pin_3.config.htim_pwm = &htim12;
    motor_joint_13pin_3.config.tim_channel = TIM_CHANNEL_1; // PB14 (M3 PWM)
    // Encoder Timer (M3 Encoder)
    motor_joint_13pin_3.config.htim_encoder = &htim5; // PA0, PA1 (M3 Encoder)

    // GPIO
    motor_joint_13pin_3.config.enable_port = NULL; // START Pin 已由硬體固定為 5V；不由 MCU 控制
    motor_joint_13pin_3.config.enable_pin = 0;
    motor_joint_13pin_3.config.dir_port = M3_DIR_PORT;
    motor_joint_13pin_3.config.dir_pin = M3_DIR_PIN;

    motor_joint_13pin_3.config.max_rpm = 5000;
    motor_joint_13pin_3.config.encoder_ppr = ENCODER_PPR_AXIS3;
    motor_joint_13pin_3.config.gear_ratio = GEAR_RATIO_AXIS3;
    motor_joint_13pin_3.config.inverted = true;  // 正向轉速 (+RPM) 驅動底座朝逆時針 (朝左) 旋轉
    motor_joint_13pin_3.config.encoder_inverted = false; // 正轉時 Timer 5 計數自然遞增，不需翻轉符號

    Motor_Init(&motor_joint_13pin_3);
}
