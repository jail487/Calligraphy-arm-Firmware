#include "joint_units.h"
#include <math.h>

#define PI_M 3.14159265358979323846f
#define RPM_TO_RADS (2.0f * PI_M / 60.0f)
#define RADS_TO_RPM (60.0f / (2.0f * PI_M))

static inline bool Is_Param_Valid(float param) {
    return isfinite(param) && (param > 0.0f);
}

static inline bool Is_Value_Valid(float val) {
    return isfinite(val);
}

float Joint_RPM_To_Rad_S(float joint_rpm) {
    if (!Is_Value_Valid(joint_rpm)) return 0.0f;
    float res = joint_rpm * RPM_TO_RADS;
    return isfinite(res) ? res : 0.0f;
}

float Joint_Rad_S_To_RPM(float joint_rad_s) {
    if (!Is_Value_Valid(joint_rad_s)) return 0.0f;
    float res = joint_rad_s * RADS_TO_RPM;
    return isfinite(res) ? res : 0.0f;
}

float Joint_RPM_To_Motor_RPM(float joint_rpm, float gear_ratio) {
    if (!Is_Value_Valid(joint_rpm) || !Is_Param_Valid(gear_ratio)) return 0.0f;
    float res = joint_rpm * gear_ratio;
    return isfinite(res) ? res : 0.0f;
}

float Motor_RPM_To_Joint_RPM(float motor_rpm, float gear_ratio) {
    if (!Is_Value_Valid(motor_rpm) || !Is_Param_Valid(gear_ratio)) return 0.0f;
    float res = motor_rpm / gear_ratio;
    return isfinite(res) ? res : 0.0f;
}

float Joint_Rad_S_To_Motor_RPM(float joint_rad_s, float gear_ratio) {
    if (!Is_Value_Valid(joint_rad_s) || !Is_Param_Valid(gear_ratio)) return 0.0f;
    return Joint_RPM_To_Motor_RPM(Joint_Rad_S_To_RPM(joint_rad_s), gear_ratio);
}

float Motor_RPM_To_Joint_Rad_S(float motor_rpm, float gear_ratio) {
    if (!Is_Value_Valid(motor_rpm) || !Is_Param_Valid(gear_ratio)) return 0.0f;
    return Joint_RPM_To_Rad_S(Motor_RPM_To_Joint_RPM(motor_rpm, gear_ratio));
}

float Motor_RPM_To_Command_Freq_Hz(float motor_rpm, float pulses_per_rev) {
    if (!Is_Value_Valid(motor_rpm) || !Is_Param_Valid(pulses_per_rev)) return 0.0f;
    float res = (motor_rpm * pulses_per_rev) / 60.0f;
    return isfinite(res) ? res : 0.0f;
}

float Command_Freq_Hz_To_Motor_RPM(float freq_hz, float pulses_per_rev) {
    if (!Is_Value_Valid(freq_hz) || !Is_Param_Valid(pulses_per_rev)) return 0.0f;
    float res = (freq_hz * 60.0f) / pulses_per_rev;
    return isfinite(res) ? res : 0.0f;
}

float Joint_Rad_S_To_Command_Freq_Hz(float joint_rad_s, float gear_ratio, float pulses_per_rev) {
    if (!Is_Value_Valid(joint_rad_s) || !Is_Param_Valid(gear_ratio) || !Is_Param_Valid(pulses_per_rev)) return 0.0f;
    float motor_rpm = Joint_Rad_S_To_Motor_RPM(joint_rad_s, gear_ratio);
    return Motor_RPM_To_Command_Freq_Hz(motor_rpm, pulses_per_rev);
}

float Joint_RPM_To_Command_Freq_Hz(float joint_rpm, float gear_ratio, float pulses_per_rev) {
    if (!Is_Value_Valid(joint_rpm) || !Is_Param_Valid(gear_ratio) || !Is_Param_Valid(pulses_per_rev)) return 0.0f;
    return Motor_RPM_To_Command_Freq_Hz(
        Joint_RPM_To_Motor_RPM(joint_rpm, gear_ratio), pulses_per_rev);
}

float Command_Freq_Hz_To_Joint_Rad_S(float freq_hz, float gear_ratio, float pulses_per_rev) {
    if (!Is_Value_Valid(freq_hz) || !Is_Param_Valid(gear_ratio) || !Is_Param_Valid(pulses_per_rev)) return 0.0f;
    float motor_rpm = Command_Freq_Hz_To_Motor_RPM(freq_hz, pulses_per_rev);
    return Motor_RPM_To_Joint_Rad_S(motor_rpm, gear_ratio);
}

float Encoder_Count_To_Joint_Rad(int64_t encoder_count, float gear_ratio, float encoder_ppr, float encoder_quadrature) {
    if (!Is_Param_Valid(gear_ratio) || !Is_Param_Valid(encoder_ppr) || !Is_Param_Valid(encoder_quadrature)) return 0.0f;
    double counts_per_joint_rev = (double)gear_ratio * (double)encoder_ppr * (double)encoder_quadrature;
    if (counts_per_joint_rev <= 0.0) return 0.0f;
    double joint_revs = (double)encoder_count / counts_per_joint_rev;
    double joint_rad = joint_revs * (2.0 * 3.14159265358979323846);
    float res = (float)joint_rad;
    return isfinite(res) ? res : 0.0f;
}

int64_t Joint_Rad_To_Encoder_Count(float joint_rad, float gear_ratio, float encoder_ppr, float encoder_quadrature) {
    if (!Is_Value_Valid(joint_rad) || !Is_Param_Valid(gear_ratio) || !Is_Param_Valid(encoder_ppr) || !Is_Param_Valid(encoder_quadrature)) return 0;
    double counts_per_joint_rev = (double)gear_ratio * (double)encoder_ppr * (double)encoder_quadrature;
    if (counts_per_joint_rev <= 0.0) return 0;
    double joint_revs = (double)joint_rad / (2.0 * 3.14159265358979323846);
    double count_dbl = joint_revs * counts_per_joint_rev;
    if (!isfinite((float)count_dbl)) return 0;
    double round_val = (count_dbl >= 0.0) ? floor(count_dbl + 0.5) : ceil(count_dbl - 0.5);
    return (int64_t)round_val;
}

float Encoder_Delta_To_Joint_Rad_S(int64_t delta_counts, float dt_s, float gear_ratio, float encoder_ppr, float encoder_quadrature) {
    if (!Is_Param_Valid(dt_s) || !Is_Param_Valid(gear_ratio) || !Is_Param_Valid(encoder_ppr) || !Is_Param_Valid(encoder_quadrature)) return 0.0f;
    float delta_rad = Encoder_Count_To_Joint_Rad(delta_counts, gear_ratio, encoder_ppr, encoder_quadrature);
    float res = delta_rad / dt_s;
    return isfinite(res) ? res : 0.0f;
}

float Joint_Deg_To_Rad(float deg) {
    if (!Is_Value_Valid(deg)) return 0.0f;
    float res = deg * (PI_M / 180.0f);
    return isfinite(res) ? res : 0.0f;
}

float Joint_Rad_To_Deg(float rad) {
    if (!Is_Value_Valid(rad)) return 0.0f;
    float res = rad * (180.0f / PI_M);
    return isfinite(res) ? res : 0.0f;
}

