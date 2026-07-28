/**
 * @file nidec_motor_driver.h
 * @brief Nidec 無刷馬達統一驅動介面 (v2.0 - 含編碼器與減速比)
 * @author Gemini Assistant
 */

#ifndef NIDEC_MOTOR_DRIVER_H
#define NIDEC_MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   // 包含 STM32 HAL Library 定義
#include <stdbool.h>

// ==========================================================
// 1. 列舉與結構定義
// ==========================================================

/**
 * @brief 馬達類型列舉
 */
typedef enum {
    MOTOR_TYPE_FREQ_13PIN, // 型號: 24H702U030 (頻率控制 STMP)
    MOTOR_TYPE_PWM_8PIN    // 型號: 24H220Q231 (PWM Duty控制, Low Active)
} MotorType_e;

struct Motor_s;

/**
 * @brief 馬達操作函式指標 (Virtual Function Table)
 */
typedef struct {
    void (*set_speed)(struct Motor_s *self, int32_t target_rpm);
    void (*set_enable)(struct Motor_s *self, bool enable);
} MotorOps_t;

/**
 * @brief 馬達硬體配置結構
 */
typedef struct {
    // --- 速度控制 Timer (PWM or Freq Gen) ---
    TIM_HandleTypeDef *htim_pwm;
    uint32_t tim_channel;

    // --- 編碼器 Timer (Encoder Mode) [新增] ---
    TIM_HandleTypeDef *htim_encoder;

    // --- 方向控制 GPIO (CW/CCW) ---
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;

    // --- 啟用/煞車 GPIO (START or BRAKE) ---
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;

    // --- 參數限制 ---
    uint32_t max_rpm;           // 馬達空載最大轉速

    // --- 機械參數 [新增] ---
    float gear_ratio;           // 減速比 (例如 50.0 代表 50:1 減速機)
    float encoder_ppr;          // 編碼器解析度 (Pulse Per Rev)，Nidec這兩顆通常是 100
    bool inverted;              // 是否方向反向 (true 代表給正速朝反方向轉)
    bool encoder_inverted;      // 是否編碼器回授反向 (true 代表回授角度與速度取反)

    // --- 速度 PID 參數 (僅 8-Pin PWM 模式使用) [新增] ---
    float speed_kp;             // 比例增益
    float speed_ki;             // 積分增益
    float speed_kd;             // 微分增益 (通常設為 0)
    float speed_integral_limit; // 積分輸出上限 (對應 Duty 0.0~1.0)
} MotorConfig_t;

/**
 * @brief 馬達物件主結構 (Object)
 */
typedef struct Motor_s {
    MotorType_e type;           // 馬達類型
    MotorConfig_t config;       // 硬體配置
    const MotorOps_t *ops;      // 操作介面

    // --- 內部狀態 (Runtime Status) ---
    int32_t current_rpm_cmd;    // 當前設定的轉速
    bool is_enabled;            // 是否已啟動

    // --- 編碼器狀態 [新增] ---
    volatile int64_t total_pulse_count; // 累計總脈衝數 (處理多圈與溢位)
    uint32_t last_counter_val;          // 上一次讀取的 Timer Counter 值
    
    // --- 速度反饋 [新增] ---
    float measured_velocity_rpm;        // 測量到的實際馬達軸速度 (RPM)
    int64_t prev_pulse_count;           // 上次的脈衝數 (用於速度計算)
    uint32_t last_update_time_ms;       // 上次更新時間 (ms)

    // --- 速度 PID 運算狀態 (僅 8-Pin PWM 模式使用) [新增] ---
    float speed_pid_integral;           // 積分累加值
    float speed_pid_prev_error;         // 上次誤差值
    float current_duty_cycle;           // 目前輸出的 PWM Duty (0.0 ~ 1.0)

    // --- 角度零點偏移 [新增] ---
    float angle_offset_deg;             // 回授角度偏移，Motor_GetAngle 會加上此值
} Motor_t;


// ==========================================================
// 2. 對外公開 API (Public Functions)
// ==========================================================


/**
 * @brief 初始化單一馬達物件
 */
void Motor_Init(Motor_t *motor);

/**
 * @brief [核心] 定期更新馬達狀態
 * @note **必須**在 main loop 或 SysTick 定期呼叫 (例如 1ms~10ms 一次)
 * 用於讀取編碼器變化量並計算總位置
 */
void Motor_Update(Motor_t *motor);

/**
 * @brief 設定馬達轉速 (輸入馬達軸轉速)
 * @param rpm 馬達軸目標轉速 (尚未經過減速機)。
 */
void Motor_SetSpeed(Motor_t *motor, int32_t rpm);

/**
 * @brief 取得輸出軸角度 (Output Shaft Angle)
 * @return 角度 (Degree)，已考慮減速比。
 * 例如：轉一圈回傳 360.0，反轉一圈回傳 -360.0
 */
float Motor_GetAngle(Motor_t *motor);

/**
 * @brief 取得輸出軸轉速 (Output Shaft Velocity)
 * @return 轉速 (RPM)，已考慮減速比。
 */
float Motor_GetVelocity(Motor_t *motor);

/**
 * @brief 重置編碼器數值
 * @note 將當前位置設為 0 度
 */
void Motor_ResetEncoder(Motor_t *motor);

/**
 * @brief 將目前位置校正為指定角度
 * @param target_deg 希望在當下位置對應到的角度
 */
void Motor_CalibrateAngle(Motor_t *motor, float target_deg);

/**
 * @brief 設定角度零點偏移
 * @note 呼叫後 Motor_GetAngle() 會回傳「實際角度 + offset」。
 */
void Motor_SetAngleOffset(Motor_t *motor, float offset_deg);

/**
 * @brief 啟動馬達 (Enable/Run)
 */
void Motor_Start(Motor_t *motor);

/**
 * @brief 停止馬達 (Disable/Brake)
 */
void Motor_Stop(Motor_t *motor);


#ifdef __cplusplus
}
#endif

#endif // NIDEC_MOTOR_DRIVER_H
