#include "motor_parameters.h"
#include <stddef.h>

static const AxisMechanicalParams_t g_axis_params_baseline[ROBOT_AXIS_COUNT] = {
    {
        .axis_name = "Axis_1_Joint_1_Shoulder",
        .gear_ratio = GEAR_RATIO_AXIS1,
        .encoder_ppr = ENCODER_PPR_AXIS1,
        .encoder_quadrature = ENCODER_QUADRATURE_AXIS1,
        .motor_pulses_per_rev = MOTOR_PULSES_PER_REV_AXIS1,
        .is_verified = true
    },
    {
        .axis_name = "Axis_2_Joint_2_Elbow",
        .gear_ratio = GEAR_RATIO_AXIS2,
        .encoder_ppr = ENCODER_PPR_AXIS2,
        .encoder_quadrature = ENCODER_QUADRATURE_AXIS2,
        .motor_pulses_per_rev = MOTOR_PULSES_PER_REV_AXIS2,
        .is_verified = true
    },
    {
        .axis_name = "Axis_3_Joint_3_Base",
        .gear_ratio = GEAR_RATIO_AXIS3,
        .encoder_ppr = ENCODER_PPR_AXIS3,
        .encoder_quadrature = ENCODER_QUADRATURE_AXIS3,
        .motor_pulses_per_rev = MOTOR_PULSES_PER_REV_AXIS3,
        .is_verified = true
    }
};

bool MotorParams_GetAxisParams(uint8_t axis, AxisMechanicalParams_t *out_params) {
    if (out_params == NULL || axis >= ROBOT_AXIS_COUNT) {
        return false;
    }
    *out_params = g_axis_params_baseline[axis];
    return true;
}
