#ifndef JOINT_UNITS_H
#define JOINT_UNITS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/**
 * @brief Convert Joint RPM to Joint rad/s
 * @param joint_rpm Joint angular velocity in RPM
 * @return Joint angular velocity in rad/s, or 0.0f if invalid
 */
float Joint_RPM_To_Rad_S(float joint_rpm);

/**
 * @brief Convert Joint rad/s to Joint RPM
 * @param joint_rad_s Joint angular velocity in rad/s
 * @return Joint angular velocity in RPM, or 0.0f if invalid
 */
float Joint_Rad_S_To_RPM(float joint_rad_s);

/**
 * @brief Convert Joint RPM to Motor shaft RPM
 * @param joint_rpm Joint angular velocity in RPM
 * @param gear_ratio Reducer gear ratio (> 0.0f)
 * @return Motor shaft angular velocity in RPM, or 0.0f if invalid
 */
float Joint_RPM_To_Motor_RPM(float joint_rpm, float gear_ratio);

/**
 * @brief Convert Motor shaft RPM to Joint RPM
 * @param motor_rpm Motor shaft angular velocity in RPM
 * @param gear_ratio Reducer gear ratio (> 0.0f)
 * @return Joint angular velocity in RPM, or 0.0f if invalid
 */
float Motor_RPM_To_Joint_RPM(float motor_rpm, float gear_ratio);

/**
 * @brief Convert Joint rad/s to Motor shaft RPM
 * @param joint_rad_s Joint angular velocity in rad/s
 * @param gear_ratio Reducer gear ratio (> 0.0f)
 * @return Motor shaft angular velocity in RPM, or 0.0f if invalid
 */
float Joint_Rad_S_To_Motor_RPM(float joint_rad_s, float gear_ratio);

/**
 * @brief Convert Motor shaft RPM to Joint rad/s
 * @param motor_rpm Motor shaft angular velocity in RPM
 * @param gear_ratio Reducer gear ratio (> 0.0f)
 * @return Joint angular velocity in rad/s, or 0.0f if invalid
 */
float Motor_RPM_To_Joint_Rad_S(float motor_rpm, float gear_ratio);

/**
 * @brief Convert Motor shaft RPM to Motor command frequency (Hz)
 * @param motor_rpm Motor shaft angular velocity in RPM
 * @param pulses_per_rev Pulses per motor shaft revolution (> 0.0f)
 * @return Frequency in Hz, or 0.0f if invalid
 */
float Motor_RPM_To_Command_Freq_Hz(float motor_rpm, float pulses_per_rev);

/**
 * @brief Convert Motor command frequency (Hz) to Motor shaft RPM
 * @param freq_hz Command frequency in Hz
 * @param pulses_per_rev Pulses per motor shaft revolution (> 0.0f)
 * @return Motor shaft angular velocity in RPM, or 0.0f if invalid
 */
float Command_Freq_Hz_To_Motor_RPM(float freq_hz, float pulses_per_rev);

/**
 * @brief Convert Joint rad/s to Motor command frequency (Hz)
 * @param joint_rad_s Joint angular velocity in rad/s
 * @param gear_ratio Reducer gear ratio (> 0.0f)
 * @param pulses_per_rev Pulses per motor shaft revolution (> 0.0f)
 * @return Frequency in Hz, or 0.0f if invalid
 */
float Joint_Rad_S_To_Command_Freq_Hz(float joint_rad_s, float gear_ratio, float pulses_per_rev);

/**
 * @brief Convert joint-space RPM to motor-shaft PWM command frequency.
 * The input/output units are explicit: joint RPM -> motor shaft RPM -> Hz.
 */
float Joint_RPM_To_Command_Freq_Hz(float joint_rpm, float gear_ratio, float pulses_per_rev);

/**
 * @brief Convert Motor command frequency (Hz) to Joint rad/s
 * @param freq_hz Command frequency in Hz
 * @param gear_ratio Reducer gear ratio (> 0.0f)
 * @param pulses_per_rev Pulses per motor shaft revolution (> 0.0f)
 * @return Joint angular velocity in rad/s, or 0.0f if invalid
 */
float Command_Freq_Hz_To_Joint_Rad_S(float freq_hz, float gear_ratio, float pulses_per_rev);

/**
 * @brief Convert raw encoder count to Joint position in radians
 * @param encoder_count Raw quadrature encoder ticks
 * @param gear_ratio Reducer gear ratio (> 0.0f)
 * @param encoder_ppr Encoder pulses per revolution (> 0.0f)
 * @param encoder_quadrature Encoder quadrature multiplier (e.g. 4.0f)
 * @return Joint position in radians, or 0.0f if invalid
 */
float Encoder_Count_To_Joint_Rad(int64_t encoder_count, float gear_ratio, float encoder_ppr, float encoder_quadrature);

/**
 * @brief Convert Joint position in radians to expected encoder count
 * @param joint_rad Joint position in radians
 * @param gear_ratio Reducer gear ratio (> 0.0f)
 * @param encoder_ppr Encoder pulses per revolution (> 0.0f)
 * @param encoder_quadrature Encoder quadrature multiplier (e.g. 4.0f)
 * @return Rounded raw quadrature encoder count, or 0 if invalid
 */
int64_t Joint_Rad_To_Encoder_Count(float joint_rad, float gear_ratio, float encoder_ppr, float encoder_quadrature);

/**
 * @brief Convert encoder delta count over sample time dt_s to Joint velocity in rad/s
 * @param delta_counts Delta raw encoder ticks over interval dt_s
 * @param dt_s Sampling interval in seconds (> 0.0f)
 * @param gear_ratio Reducer gear ratio (> 0.0f)
 * @param encoder_ppr Encoder pulses per revolution (> 0.0f)
 * @param encoder_quadrature Encoder quadrature multiplier (e.g. 4.0f)
 * @return Joint velocity in rad/s, or 0.0f if invalid
 */
float Encoder_Delta_To_Joint_Rad_S(int64_t delta_counts, float dt_s, float gear_ratio, float encoder_ppr, float encoder_quadrature);

/**
 * @brief Convert degrees to radians
 * @param deg Angle in degrees
 * @return Angle in radians, or 0.0f if invalid
 */
float Joint_Deg_To_Rad(float deg);

/**
 * @brief Convert radians to degrees
 * @param rad Angle in radians
 * @return Angle in degrees, or 0.0f if invalid
 */
float Joint_Rad_To_Deg(float rad);


#ifdef __cplusplus
}
#endif

#endif // JOINT_UNITS_H
