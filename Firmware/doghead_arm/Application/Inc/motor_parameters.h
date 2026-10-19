#ifndef MOTOR_PARAMETERS_H
#define MOTOR_PARAMETERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 
 * -----------------------------------------------------------------------------
 * Hardware Parameters for Axes 1, 2, 3
 * -----------------------------------------------------------------------------
 * Gear Ratio: Reducer ratio (e.g., 114.0f means 114 motor turns = 1 joint turn)
 * Encoder PPR: Pulses Per Revolution of the motor shaft.
 * Timer Quadrature: The hardware timer's quadrature mode multiplier (usually 4).
 * Motor Pulses Per Rev: Frequency command pulses per motor shaft revolution.
 * 
 * These parameters are the user-confirmed mechanical baseline taken from the
 * original firmware configuration. Keep them synchronized with motor_config.c
 * and homing_calibration.cpp when the mechanism is changed.
 */

// --- Axis 1 Parameters ---
#define GEAR_RATIO_AXIS1            114.0f
#define ENCODER_PPR_AXIS1           100.0f
#define ENCODER_QUADRATURE_AXIS1    4.0f
#define MOTOR_PULSES_PER_REV_AXIS1  400.0f

// --- Axis 2 Parameters ---
#define GEAR_RATIO_AXIS2            114.0f
#define ENCODER_PPR_AXIS2           100.0f
#define ENCODER_QUADRATURE_AXIS2    4.0f
#define MOTOR_PULSES_PER_REV_AXIS2  400.0f

// --- Axis 3 Parameters ---
#define GEAR_RATIO_AXIS3            114.0f
#define ENCODER_PPR_AXIS3           100.0f
#define ENCODER_QUADRATURE_AXIS3    4.0f
#define MOTOR_PULSES_PER_REV_AXIS3  400.0f

#define ROBOT_AXIS_COUNT            3

/**
 * @struct AxisMechanicalParams_t
 * @brief Structure holding named per-axis mechanical parameters.
 * @note The values are confirmed by the project owner against the original
 *       firmware configuration. This flag records that confirmation; it does
 *       not replace later powered-load safety checks.
 */
typedef struct {
    const char* axis_name;
    float gear_ratio;
    float encoder_ppr;
    float encoder_quadrature;
    float motor_pulses_per_rev;
    bool is_verified;
} AxisMechanicalParams_t;

/**
 * @brief Get named mechanical parameters for the specified axis index (0, 1, or 2).
 * @param axis Axis index (0 = Axis 1, 1 = Axis 2, 2 = Axis 3)
 * @param out_params Pointer to structure to populate
 * @return true if axis is valid (0 <= axis < ROBOT_AXIS_COUNT), false otherwise.
 */
bool MotorParams_GetAxisParams(uint8_t axis, AxisMechanicalParams_t *out_params);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_PARAMETERS_H
