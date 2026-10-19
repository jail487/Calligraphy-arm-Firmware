#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "joint_units.h"
#include "motor_parameters.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TEST(JointUnitsTest, NominalConversions) {
    // RPM <-> Rad/S
    float rpm = 60.0f;
    float rad_s = Joint_RPM_To_Rad_S(rpm);
    EXPECT_NEAR(rad_s, static_cast<float>(2.0 * M_PI), 1e-4f);
    EXPECT_NEAR(Joint_Rad_S_To_RPM(rad_s), rpm, 1e-4f);

    // Joint RPM <-> Motor RPM (Gear ratio = 114.0)
    float gear_ratio = 114.0f;
    float joint_rpm = 1.0f;
    float motor_rpm = Joint_RPM_To_Motor_RPM(joint_rpm, gear_ratio);
    EXPECT_FLOAT_EQ(motor_rpm, 114.0f);
    EXPECT_FLOAT_EQ(Motor_RPM_To_Joint_RPM(motor_rpm, gear_ratio), joint_rpm);

    // Joint Rad/S <-> Motor RPM
    float motor_rpm_from_rad = Joint_Rad_S_To_Motor_RPM(static_cast<float>(2.0 * M_PI), gear_ratio);
    EXPECT_NEAR(motor_rpm_from_rad, 60.0f * 114.0f, 1e-2f);
    EXPECT_NEAR(Motor_RPM_To_Joint_Rad_S(motor_rpm_from_rad, gear_ratio), static_cast<float>(2.0 * M_PI), 1e-4f);

    // Motor RPM <-> Freq (Hz)
    float ppr = 400.0f;
    float freq = Motor_RPM_To_Command_Freq_Hz(60.0f, ppr);
    EXPECT_FLOAT_EQ(freq, 400.0f);
    EXPECT_FLOAT_EQ(Command_Freq_Hz_To_Motor_RPM(freq, ppr), 60.0f);

    // Joint Rad/S <-> Command Freq (Hz)
    float joint_freq = Joint_Rad_S_To_Command_Freq_Hz(static_cast<float>(2.0 * M_PI), gear_ratio, ppr);
    EXPECT_NEAR(joint_freq, 400.0f * 114.0f, 1e-1f);
    EXPECT_NEAR(Command_Freq_Hz_To_Joint_Rad_S(joint_freq, gear_ratio, ppr), static_cast<float>(2.0 * M_PI), 1e-4f);
}

TEST(JointUnitsTest, EncoderCountConversions) {
    float gear_ratio = 114.0f;
    float ppr = 100.0f;
    float quad = 4.0f;
    double counts_per_rev = 114.0 * 100.0 * 4.0; // 45600 count/rev

    // 0 count -> 0 rad
    EXPECT_FLOAT_EQ(Encoder_Count_To_Joint_Rad(0, gear_ratio, ppr, quad), 0.0f);
    EXPECT_EQ(Joint_Rad_To_Encoder_Count(0.0f, gear_ratio, ppr, quad), 0);

    // 1 full joint rev (2*PI rad) -> 45600 counts
    int64_t full_rev_counts = static_cast<int64_t>(counts_per_rev);
    float rad = Encoder_Count_To_Joint_Rad(full_rev_counts, gear_ratio, ppr, quad);
    EXPECT_NEAR(rad, static_cast<float>(2.0 * M_PI), 1e-4f);
    EXPECT_EQ(Joint_Rad_To_Encoder_Count(static_cast<float>(2.0 * M_PI), gear_ratio, ppr, quad), full_rev_counts);

    // Negative counts (-PI rad)
    int64_t neg_counts = -22800;
    float neg_rad = Encoder_Count_To_Joint_Rad(neg_counts, gear_ratio, ppr, quad);
    EXPECT_NEAR(neg_rad, static_cast<float>(-M_PI), 1e-4f);
    EXPECT_EQ(Joint_Rad_To_Encoder_Count(static_cast<float>(-M_PI), gear_ratio, ppr, quad), neg_counts);

    // Encoder Delta -> Rad/S
    float vel_rad_s = Encoder_Delta_To_Joint_Rad_S(45600, 1.0f, gear_ratio, ppr, quad);
    EXPECT_NEAR(vel_rad_s, static_cast<float>(2.0 * M_PI), 1e-4f);
}

TEST(JointUnitsTest, InvalidParameterRejection) {
    float nan_val = std::numeric_limits<float>::quiet_NaN();
    float inf_val = std::numeric_limits<float>::infinity();

    // NaN / Inf inputs
    EXPECT_FLOAT_EQ(Joint_RPM_To_Rad_S(nan_val), 0.0f);
    EXPECT_FLOAT_EQ(Joint_RPM_To_Rad_S(inf_val), 0.0f);
    EXPECT_FLOAT_EQ(Joint_RPM_To_Motor_RPM(nan_val, 114.0f), 0.0f);
    EXPECT_FLOAT_EQ(Encoder_Count_To_Joint_Rad(100, nan_val, 100.0f, 4.0f), 0.0f);

    // Invalid parameters (0 or negative)
    EXPECT_FLOAT_EQ(Joint_RPM_To_Motor_RPM(10.0f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(Joint_RPM_To_Motor_RPM(10.0f, -114.0f), 0.0f);
    EXPECT_FLOAT_EQ(Motor_RPM_To_Joint_RPM(100.0f, -1.0f), 0.0f);
    EXPECT_FLOAT_EQ(Motor_RPM_To_Command_Freq_Hz(100.0f, 0.0f), 0.0f);

    // Invalid encoder parameters
    EXPECT_FLOAT_EQ(Encoder_Count_To_Joint_Rad(1000, 0.0f, 100.0f, 4.0f), 0.0f);
    EXPECT_FLOAT_EQ(Encoder_Count_To_Joint_Rad(1000, 114.0f, -100.0f, 4.0f), 0.0f);
    EXPECT_FLOAT_EQ(Encoder_Count_To_Joint_Rad(1000, 114.0f, 100.0f, -4.0f), 0.0f);
    EXPECT_EQ(Joint_Rad_To_Encoder_Count(1.0f, 0.0f, 100.0f, 4.0f), 0);

    // Invalid dt
    EXPECT_FLOAT_EQ(Encoder_Delta_To_Joint_Rad_S(100, 0.0f, 114.0f, 100.0f, 4.0f), 0.0f);
    EXPECT_FLOAT_EQ(Encoder_Delta_To_Joint_Rad_S(100, -0.01f, 114.0f, 100.0f, 4.0f), 0.0f);
}

TEST(JointUnitsTest, NamedMechanicalParameters) {
    AxisMechanicalParams_t params;

    // Axis 0, 1, 2
    for (uint8_t i = 0; i < 3; i++) {
        EXPECT_TRUE(MotorParams_GetAxisParams(i, &params));
        EXPECT_GT(params.gear_ratio, 0.0f);
        EXPECT_GT(params.encoder_ppr, 0.0f);
        EXPECT_GT(params.encoder_quadrature, 0.0f);
        EXPECT_GT(params.motor_pulses_per_rev, 0.0f);
        EXPECT_TRUE(params.is_verified); // User-confirmed firmware baseline
    }

    // Invalid axis index
    EXPECT_FALSE(MotorParams_GetAxisParams(3, &params));
    EXPECT_FALSE(MotorParams_GetAxisParams(255, &params));
}
