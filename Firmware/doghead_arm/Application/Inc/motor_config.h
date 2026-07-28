/**
 * @file motor_config.h
 * @brief 系統馬達腳位與硬體配置標頭檔 (Application Layer)
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "nidec_motor_driver.h"

// ==========================================================
// 全域馬達物件實體宣告 (Application Level Motor Instances)
// ==========================================================
extern Motor_t motor_joint_13pin;   // Joint 1 (M1 馬達物件)
extern Motor_t motor_joint_13pin_2; // Joint 2 (M2 馬達物件)
extern Motor_t motor_joint_13pin_3; // Joint 3 (M3 馬達物件)

// ==========================================================
// 馬達 GPIO 腳位與連接埠設定
// ==========================================================
// Motor 1 (Joint 1)
// #define M1_ENABLE_PORT     GPIOC        // Motor Enable 已取消
// #define M1_ENABLE_PIN      GPIO_PIN_4   // Disabled (Free)
#define M1_DIR_PORT        GPIOC
#define M1_DIR_PIN         GPIO_PIN_1

// Motor 2 (Joint 2)
// #define M2_ENABLE_PORT     GPIOC        // Motor Enable 已取消
// #define M2_ENABLE_PIN      GPIO_PIN_4   // Disabled (Free)
#define M2_DIR_PORT        GPIOC
#define M2_DIR_PIN         GPIO_PIN_2

// Motor 3 (Joint 3)
// #define M3_ENABLE_PORT     GPIOC        // Motor Enable 已取消
// #define M3_ENABLE_PIN      GPIO_PIN_0   // Disabled (Free)
#define M3_DIR_PORT        GPIOC
#define M3_DIR_PIN         GPIO_PIN_3

/**
 * @brief 系統層級的馬達腳位與硬體配置初始化
 * @note 請在系統初始化 (Robot_Init 或 main) 時呼叫此函式
 */
void Motor_System_Config(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONFIG_H

