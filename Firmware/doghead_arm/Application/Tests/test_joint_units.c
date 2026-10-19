#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "joint_units.h"
#include "motor_parameters.h"
#include "nidec_motor_driver.h"

static int g_tests_run = 0;
static int g_tests_passed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        g_tests_run++; \
        if (cond) { \
            g_tests_passed++; \
        } else { \
            printf("[FAIL] Line %d: %s\n", __LINE__, msg); \
        } \
    } while(0)

static void test_nominal_conversions(void) {
    float rpm = 60.0f;
    float rad_s = Joint_RPM_To_Rad_S(rpm);
    TEST_ASSERT(fabsf(rad_s - (2.0f * 3.14159265f)) < 1e-4f, "Joint_RPM_To_Rad_S");
    TEST_ASSERT(fabsf(Joint_Rad_S_To_RPM(rad_s) - rpm) < 1e-4f, "Joint_Rad_S_To_RPM");

    float gear_ratio = 114.0f;
    float motor_rpm = Joint_RPM_To_Motor_RPM(1.0f, gear_ratio);
    TEST_ASSERT(fabsf(motor_rpm - 114.0f) < 1e-4f, "Joint_RPM_To_Motor_RPM");
    TEST_ASSERT(fabsf(Motor_RPM_To_Joint_RPM(motor_rpm, gear_ratio) - 1.0f) < 1e-4f, "Motor_RPM_To_Joint_RPM");
    TEST_ASSERT(fabsf(Motor_RPM_To_Command_Freq_Hz(motor_rpm, 400.0f) - 760.0f) < 1e-4f,
                "Output RPM to STMP frequency");

    float ppr = 400.0f;
    float freq = Motor_RPM_To_Command_Freq_Hz(60.0f, ppr);
    TEST_ASSERT(fabsf(freq - 400.0f) < 1e-4f, "Motor_RPM_To_Command_Freq_Hz");
    TEST_ASSERT(fabsf(Command_Freq_Hz_To_Motor_RPM(freq, ppr) - 60.0f) < 1e-4f, "Command_Freq_Hz_To_Motor_RPM");

    float joint_freq = Joint_Rad_S_To_Command_Freq_Hz((2.0f * 3.14159265f), gear_ratio, ppr);
    TEST_ASSERT(fabsf(joint_freq - (400.0f * 114.0f)) < 1e-1f, "Joint_Rad_S_To_Command_Freq_Hz");
    TEST_ASSERT(fabsf(Joint_RPM_To_Command_Freq_Hz(1.0f, gear_ratio, ppr) - 760.0f) < 1e-4f,
                "Joint RPM -> motor RPM -> PWM frequency");
    TEST_ASSERT(fabsf(Command_Freq_Hz_To_Joint_Rad_S(joint_freq, gear_ratio, ppr) - (2.0f * 3.14159265f)) < 1e-4f, "Command_Freq_Hz_To_Joint_Rad_S");
}

static void test_encoder_velocity_units(void) {
    // 100 PPR with x4 quadrature and 114:1 reduction: 45,600 counts/output rev.
    const float one_output_rev_rpm =
        Motor_ComputeVelocityRpmFromPulses(45600, 1.0f, 100.0f, 114.0f);
    TEST_ASSERT(fabsf(one_output_rev_rpm - 60.0f) < 1e-4f,
                "Encoder pulse conversion must return output-shaft RPM");

    const float reverse_rpm =
        Motor_ComputeVelocityRpmFromPulses(-45600, 1.0f, 100.0f, 114.0f);
    TEST_ASSERT(fabsf(reverse_rpm + 60.0f) < 1e-4f,
                "Encoder pulse sign must be preserved");

    const float one_count_rpm =
        Motor_ComputeVelocityRpmFromPulses(1, 0.01f, 100.0f, 114.0f);
    TEST_ASSERT(fabsf(one_count_rpm - (60.0f / 456.0f)) < 1e-5f,
                "Low-speed one-count quantization must be deterministic");

    TEST_ASSERT(Motor_ComputeVelocityRpmFromPulses(1, 0.0f, 100.0f, 114.0f) == 0.0f,
                "Zero sample time must fail safe");
    TEST_ASSERT(Motor_ComputeVelocityRpmFromPulses(1, 0.01f, 0.0f, 114.0f) == 0.0f,
                "Invalid encoder PPR must fail safe");
    TEST_ASSERT(Motor_ComputeVelocityRpmFromPulses(1, 0.01f, 100.0f, 0.0f) == 0.0f,
                "Invalid gear ratio must fail safe");

    Motor_t motor;
    memset(&motor, 0, sizeof(motor));
    motor.raw_velocity_rpm = 4.0f;
    motor.measured_velocity_rpm = 2.5f;
    TEST_ASSERT(fabsf(Motor_GetRawVelocity(&motor) - 4.0f) < 1e-6f,
                "Raw velocity getter must expose the unfiltered estimate");
    TEST_ASSERT(fabsf(Motor_GetVelocity(&motor) - 2.5f) < 1e-6f,
                "Filtered velocity getter must expose the filtered estimate");
    motor.config.encoder_inverted = true;
    TEST_ASSERT(fabsf(Motor_GetRawVelocity(&motor) + 4.0f) < 1e-6f,
                "Raw velocity getter must apply encoder inversion");
    TEST_ASSERT(fabsf(Motor_GetVelocity(&motor) + 2.5f) < 1e-6f,
                "Filtered velocity getter must apply encoder inversion");
}

static void test_encoder_conversions(void) {
    float gear_ratio = 114.0f;
    float ppr = 100.0f;
    float quad = 4.0f;
    int64_t full_rev_counts = 45600; // 114 * 100 * 4

    TEST_ASSERT(Encoder_Count_To_Joint_Rad(0, gear_ratio, ppr, quad) == 0.0f, "Encoder_Count_To_Joint_Rad 0");
    TEST_ASSERT(Joint_Rad_To_Encoder_Count(0.0f, gear_ratio, ppr, quad) == 0, "Joint_Rad_To_Encoder_Count 0");

    float rad = Encoder_Count_To_Joint_Rad(full_rev_counts, gear_ratio, ppr, quad);
    TEST_ASSERT(fabsf(rad - (2.0f * 3.14159265f)) < 1e-4f, "Encoder_Count_To_Joint_Rad 1rev");
    TEST_ASSERT(Joint_Rad_To_Encoder_Count(2.0f * 3.14159265f, gear_ratio, ppr, quad) == full_rev_counts, "Joint_Rad_To_Encoder_Count 1rev");

    float vel = Encoder_Delta_To_Joint_Rad_S(full_rev_counts, 1.0f, gear_ratio, ppr, quad);
    TEST_ASSERT(fabsf(vel - (2.0f * 3.14159265f)) < 1e-4f, "Encoder_Delta_To_Joint_Rad_S");
}

static void test_invalid_parameters(void) {
    float nan_val = 0.0f / 0.0f;
    float inf_val = 1.0f / 0.0f;

    TEST_ASSERT(Joint_RPM_To_Rad_S(nan_val) == 0.0f, "NaN RPM to Rad/S");
    TEST_ASSERT(Joint_RPM_To_Rad_S(inf_val) == 0.0f, "Inf RPM to Rad/S");
    TEST_ASSERT(Joint_RPM_To_Motor_RPM(10.0f, 0.0f) == 0.0f, "Zero gear ratio");
    TEST_ASSERT(Joint_RPM_To_Motor_RPM(10.0f, -114.0f) == 0.0f, "Negative gear ratio");
    TEST_ASSERT(Encoder_Count_To_Joint_Rad(1000, 114.0f, -100.0f, 4.0f) == 0.0f, "Negative PPR");
    TEST_ASSERT(Encoder_Delta_To_Joint_Rad_S(100, 0.0f, 114.0f, 100.0f, 4.0f) == 0.0f, "Zero dt");
}

static void test_mechanical_parameters(void) {
    AxisMechanicalParams_t params;
    TEST_ASSERT(MotorParams_GetAxisParams(0, &params), "Axis 0 params");
    TEST_ASSERT(params.gear_ratio == 114.0f, "Axis 0 gear ratio");
    TEST_ASSERT(params.is_verified == true, "Axis 0 confirmed flag");

    TEST_ASSERT(MotorParams_GetAxisParams(1, &params), "Axis 1 params");
    TEST_ASSERT(MotorParams_GetAxisParams(2, &params), "Axis 2 params");
    TEST_ASSERT(!MotorParams_GetAxisParams(3, &params), "Axis 3 out of bounds");
}

static void test_actuator_output_enable_default(void) {
    TEST_ASSERT(ACTUATOR_HARDWARE_OUTPUT_ENABLED == 0, "Default ACTUATOR_HARDWARE_OUTPUT_ENABLED must be 0");
}

static void test_rpm_gear_ratio_single_application(void) {
    float joint_rpm = 1.0f;
    float gear_ratio = 114.0f;
    float ppr = 400.0f;

    float motor_rpm = Joint_RPM_To_Motor_RPM(joint_rpm, gear_ratio);
    TEST_ASSERT(fabsf(motor_rpm - 114.0f) < 1e-4f, "Joint RPM to Motor shaft RPM single gear ratio check");

    float freq_hz = Joint_RPM_To_Command_Freq_Hz(joint_rpm, gear_ratio, ppr);
    TEST_ASSERT(fabsf(freq_hz - 760.0f) < 1e-4f, "Joint RPM to command frequency single gear ratio check");

    float joint_rad_s = Command_Freq_Hz_To_Joint_Rad_S(freq_hz, gear_ratio, ppr);
    float round_trip_joint_rpm = Joint_Rad_S_To_RPM(joint_rad_s);
    TEST_ASSERT(fabsf(round_trip_joint_rpm - 1.0f) < 1e-4f, "Round-trip frequency to Joint RPM check");
}

static void test_motor_test_vs_pva_single_gear_ratio_conversion(void) {
    // Representative reducer gear ratio (114:1 for Axis 1, 2, 3) and PPR (400)
    const float gear_ratio = 114.0f;
    const float ppr = 400.0f;

    // 1. Motor Test Path: target_motor_rpm = 10.0f (joint RPM)
    const float motor_test_joint_rpm = 10.0f;
    const float motor_test_freq = Joint_RPM_To_Command_Freq_Hz(motor_test_joint_rpm, gear_ratio, ppr);
    // (10 joint RPM * 114 gear ratio * 400 ppr) / 60 = 7,600 Hz
    TEST_ASSERT(fabsf(motor_test_freq - 7600.0f) < 1e-3f,
                "Motor Test 10 joint RPM with 114:1 gear ratio must produce 7600 Hz PWM (ratio applied once)");

    // 2. PVA PID Path: target velocity 60 deg/s -> (60/360)*60 = 10.0f joint RPM
    const float pva_target_vel_deg_s = 60.0f;
    const float pva_joint_rpm = (pva_target_vel_deg_s / 360.0f) * 60.0f; // 10.0f joint RPM
    const float pva_freq = Joint_RPM_To_Command_Freq_Hz(pva_joint_rpm, gear_ratio, ppr);
    TEST_ASSERT(fabsf(pva_freq - 7600.0f) < 1e-3f,
                "PVA PID 60 deg/s target velocity must produce identical 7600 Hz PWM (ratio applied once)");

    // 3. Verify no double conversion: if gear ratio was erroneously applied twice,
    // frequency would be 7600 * 114 = 866,400 Hz.
    TEST_ASSERT(motor_test_freq < 100000.0f && pva_freq < 100000.0f,
                "Neither Motor Test nor PVA path may pre-multiply by gear ratio to cause double conversion");
}

int main(void) {
    printf("Running Phase 0 unit conversion tests...\n");
    test_nominal_conversions();
    test_encoder_conversions();
    test_encoder_velocity_units();
    test_invalid_parameters();
    test_mechanical_parameters();
    test_actuator_output_enable_default();
    test_rpm_gear_ratio_single_application();
    test_motor_test_vs_pva_single_gear_ratio_conversion();

    printf("Tests run: %d, Passed: %d\n", g_tests_run, g_tests_passed);
    if (g_tests_run == g_tests_passed) {
        printf("ALL TESTS PASSED.\n");
        return 0;
    } else {
        printf("SOME TESTS FAILED!\n");
        return 1;
    }
}
