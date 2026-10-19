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

#include <stdint.h>
#include <stddef.h>
#ifndef HOST_TEST
#include "main.h"   // 包含 STM32 HAL Library 定義
#else
typedef struct {
    uint32_t ARR;
    uint32_t PSC;
} TIM_TypeDef;
typedef struct {
    TIM_TypeDef* Instance;
    uint32_t opaque;
    uint32_t State;
    uint32_t Counter;
    uint32_t AutoReload;
    uint32_t Compare;
} TIM_HandleTypeDef;
typedef struct { uint32_t opaque; } GPIO_TypeDef;
#define HAL_TIM_STATE_BUSY 2U
#define TIM_CHANNEL_ALL 0U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_EVENTSOURCE_UPDATE 0U
#define TIM_FLAG_UPDATE 0U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_0 0x0001U
#define GPIO_PIN_1 0x0002U
#define GPIO_PIN_2 0x0004U
#define GPIO_PIN_3 0x0008U
#define GPIO_PIN_4 0x0010U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_12 0x1000U
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim12;
extern GPIO_TypeDef *GPIOA;
extern GPIO_TypeDef *GPIOB;
extern GPIO_TypeDef *GPIOC;
static inline uint32_t HAL_RCC_GetPCLK1Freq(void) { return 0U; }
static inline uint32_t HAL_GetTick(void) { return 0U; }
static inline int HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel) { (void)htim; (void)channel; return 0; }
static inline int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel) { (void)htim; (void)channel; return 0; }
static inline int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t channel) { (void)htim; (void)channel; return 0; }
static inline void HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t event) { (void)htim; (void)event; }
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state) { (void)port; (void)pin; (void)state; }
#define __HAL_TIM_SET_COMPARE(htim, channel, value) ((void)((htim)->Compare = (value)))
#define __HAL_TIM_SET_PRESCALER(htim, value) ((void)(htim))
#define __HAL_TIM_SET_AUTORELOAD(htim, value) ((void)((htim)->AutoReload = (value)))
#define __HAL_TIM_GET_COUNTER(htim) ((htim)->Counter)
#define __HAL_TIM_GET_AUTORELOAD(htim) ((htim)->AutoReload)
#define __HAL_TIM_SET_COUNTER(htim, value) ((void)((htim)->Counter = (value)))
#define __HAL_TIM_CLEAR_FLAG(htim, flag) ((void)(htim))
#endif
#include <stdbool.h>
#include "actuator_config.h"


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
    /* Joint/output-shaft RPM stored by Motor_SetSpeed(); NOT motor shaft RPM.
     * Gear ratio is applied once inside Impl_Freq13Pin_SetSpeed(). */
    int32_t current_rpm_cmd;    // 當前設定的關節轉速 (Joint RPM)
    bool is_enabled;            // 是否已啟動

    // --- 編碼器狀態 [新增] ---
    volatile int64_t total_pulse_count; // 累計總脈衝數 (處理多圈與溢位)
    uint32_t last_counter_val;          // 上一次讀取的 Timer Counter 值
    
    // --- 速度反饋 [新增] ---
    float raw_velocity_rpm;             // 最近一次取樣的未濾波輸出軸速度 (RPM)
    float measured_velocity_rpm;        // 濾波後輸出軸速度 (RPM)
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
 * @brief Return the cumulative quadrature count maintained by Motor_Update().
 * This is intentionally distinct from last_counter_val, which is only the
 * modulo timer register snapshot.
 */
static inline int64_t Motor_GetTotalPulseCount(const Motor_t *motor) {
  return motor != NULL ? motor->total_pulse_count : 0;
}

/**
 * @brief 將編碼器脈衝差換算為減速機輸出軸 RPM。
 * @param pulse_delta 四倍頻後的累計脈衝差；正負代表方向。
 * @param dt_seconds 本次取樣時間差（秒）。
 * @param encoder_ppr 馬達軸每轉 PPR（未乘四倍頻）。
 * @param gear_ratio 馬達軸到輸出軸的減速比。
 * @return 輸出軸 RPM；參數無效或結果非有限時回傳 0。
 */
float Motor_ComputeVelocityRpmFromPulses(int64_t pulse_delta,
                                         float dt_seconds,
                                         float encoder_ppr,
                                         float gear_ratio);

/**
 * @brief Set target motor velocity in joint-space RPM.
 * @param motor Pointer to motor instance.
 * @param rpm Target joint angular velocity in RPM (joint output shaft).
 * Input is checked and capped against motor->config.max_rpm / gear_ratio.
 * Converts joint RPM to motor shaft RPM via gear_ratio, applying gear ratio exactly once.
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
 * @brief 取得最近一次未濾波的輸出軸速度估計值（RPM）。
 * @return 未濾波輸出軸 RPM；編碼器反向設定會套用相同方向修正。
 */
float Motor_GetRawVelocity(Motor_t *motor);

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

#ifdef HOST_TEST
/**
 * @brief [Host Test Spy] Reset Motor_Stop call counter.
 */
void Motor_ResetStopCallCount(void);

/**
 * @brief [Host Test Spy] Get Motor_Stop call counter.
 */
uint32_t Motor_GetStopCallCount(void);
#endif


#ifdef __cplusplus
}
#endif

#endif // NIDEC_MOTOR_DRIVER_H
