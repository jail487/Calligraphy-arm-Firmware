/**
 * @file motor_config.c
 * @brief 系統馬達腳位與硬體配置實作 (Application Layer)
 */

#include "motor_config.h"
#include "nidec_motor_driver.h"

void Motor_System_Config(void) {

    // --- 13-Pin 馬達配置 (Joint 1 / M1 硬體) ---
    motor_joint_13pin.type = MOTOR_TYPE_FREQ_13PIN;

    // 速度 Timer (M1 硬體)
    motor_joint_13pin.config.htim_pwm = &htim2;
    motor_joint_13pin.config.tim_channel = TIM_CHANNEL_1; // PA5 (M1 PWM)
    // Encoder Timer (M1 Encoder)
    motor_joint_13pin.config.htim_encoder = &htim1; // PA8, PA9 (M1 Encoder)
    motor_joint_13pin.config.enable_port = M1_ENABLE_PORT;
    motor_joint_13pin.config.enable_pin = M1_ENABLE_PIN; // START
    motor_joint_13pin.config.dir_port = M1_DIR_PORT;
    motor_joint_13pin.config.dir_pin = M1_DIR_PIN; // DIR

    // 參數
    motor_joint_13pin.config.max_rpm = 1710; // 0.25 rps * 114:1 = 1710 rpm (motor shaft)
    motor_joint_13pin.config.encoder_ppr = 100.0f; // [已確認] 規格為 100 PPR
    motor_joint_13pin.config.gear_ratio = 114.0f;  // 減速比 114:1
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
    motor_joint_13pin_2.config.enable_port = M2_ENABLE_PORT;
    motor_joint_13pin_2.config.enable_pin = M2_ENABLE_PIN; // BRAKE
    motor_joint_13pin_2.config.dir_port = M2_DIR_PORT;
    motor_joint_13pin_2.config.dir_pin = M2_DIR_PIN; // DIR

    // 參數
    motor_joint_13pin_2.config.max_rpm = 1710; // 0.25 rps * 114:1 = 1710 rpm (motor shaft)
    motor_joint_13pin_2.config.encoder_ppr = 100.0f; // [已確認] 規格為 100 PPR
    motor_joint_13pin_2.config.gear_ratio = 114.0f;  // 減速比 114:1
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
    motor_joint_13pin_3.config.enable_port = M3_ENABLE_PORT;
    motor_joint_13pin_3.config.enable_pin = M3_ENABLE_PIN;
    motor_joint_13pin_3.config.dir_port = M3_DIR_PORT;
    motor_joint_13pin_3.config.dir_pin = M3_DIR_PIN;

    // 參數
    motor_joint_13pin_3.config.max_rpm = 1710; // 0.25 rps * 114:1 = 1710 rpm (motor shaft)
    motor_joint_13pin_3.config.encoder_ppr = 100.0f;
    motor_joint_13pin_3.config.gear_ratio = 114.0f; // 減速比 114:1
    motor_joint_13pin_3.config.inverted = false;     // 預設第三軸方向取反
    motor_joint_13pin_3.config.encoder_inverted = false; // 修正：底座朝左旋轉時角度累加方向為正

    Motor_Init(&motor_joint_13pin_3);
}
