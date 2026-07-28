#include "test_scenarios.h"
#include "robot_config.h"
#include "safety_manager.h"
#include "motor_config.h"
#include "nidec_motor_driver.h"
#include <cmath>

// ==========================================================
// 測試模式變數定義
// ==========================================================
volatile int32_t test_target_rpm_m1 = 0;  
volatile int32_t test_target_rpm_m2 = 0;
volatile int32_t test_target_rpm_m3 = 0;

volatile int32_t test_mode_selection = 0;
volatile int32_t mode1_target_rpm = 0;
volatile bool enable_square_test = false;

// ==========================================================
// API 實作
// ==========================================================
extern "C" void Robot_SetTestMode(bool enable) {
    // 統一使用 test_mode_selection 作為唯一模式來源
    // false -> PID 模式 (0)
    // true  -> 測試模式 (1: 定速模式)
    test_mode_selection = enable ? 1 : 0;
    if (enable) {
        RobotSafety_RequestRun();
    }
    if (enable) {
        ik_mode_enabled = false;  // 測試模式下關閉運動學控制
    }
}

extern "C" void Robot_SetTestSpeed(int32_t rpm_motor1, int32_t rpm_motor2, int32_t rpm_motor3) {
    if (rpm_motor1 > 1710 || rpm_motor1 < -1710 ||
        rpm_motor2 > 1710 || rpm_motor2 < -1710 ||
        rpm_motor3 > 1710 || rpm_motor3 < -1710) {
        RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
        return;
    }
    // 統一寫入 FreeRTOS 控制任務使用的目標變數
    test_target_rpm_m1 = rpm_motor1;
    test_target_rpm_m2 = rpm_motor2;
    test_target_rpm_m3 = rpm_motor3;
}

extern "C" void Robot_SquareTest_Tick(void) {
    if (!enable_square_test) return;

    static uint8_t sq_state = 255;
    static float cur_x = 80.0f;
    static float cur_y = 80.0f;
    float step = 0.02f; // 1ms 迴圈下 0.02mm = 20 mm/s

    if (sq_state == 255) {
        // 剛結束歸零，從當前的實體坐標出發
        cur_x = robot_state.real_x;
        cur_y = robot_state.real_y;
        sq_state = 100;
    } else if (sq_state == 100) {
        // 平滑移動到正方形起點 (80, 80)
        float dx = 80.0f - cur_x;
        float dy = 80.0f - cur_y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < step) {
            cur_x = 80.0f;
            cur_y = 80.0f;
            sq_state = 0; // 抵達起點，開始正方形
        } else {
            cur_x += (dx / dist) * step;
            cur_y += (dy / dist) * step;
        }
    } else if (sq_state == 0) { // 80,80 -> 80,160
        cur_y += step;
        if (cur_y >= 160.0f) {
            cur_y = 160.0f;
            sq_state = 1;
        }
    } else if (sq_state == 1) { // 80,160 -> 160,160
        cur_x += step;
        if (cur_x >= 160.0f) {
            cur_x = 160.0f;
            sq_state = 2;
        }
    } else if (sq_state == 2) { // 160,160 -> 160,80
        cur_y -= step;
        if (cur_y <= 80.0f) {
            cur_y = 80.0f;
            sq_state = 3;
        }
    } else if (sq_state == 3) { // 160,80 -> 80,80
        cur_x -= step;
        if (cur_x <= 80.0f) {
            cur_x = 80.0f;
            sq_state = 0;
        }
    }

    Robot_SetTargetPosition(cur_x, cur_y, robot_target.pose_z);
}

extern "C" void Robot_TestMode_Tick(void) {
    if (test_mode_selection == 0) return;

    // 狀態變數
    static int32_t last_mode = 0;

    // Mode 2: 線性加速變數
    static float ramp_current_rpm = 0.0f;
    const float ramp_target_rpm = 5000.0f; // 最大速度降至 5000
    const float ramp_duration_sec = 10.0f; // 加速時間縮短至 10秒 (原本60秒)
    const float ramp_step = ramp_target_rpm / (ramp_duration_sec * 1000.0f); // 每ms增加量

    // Mode 3: 階梯加速變數
    static int32_t step_current_rpm = 0;
    const int32_t step_target_rpm = 5000;   // 最大速度降至 5000
    const int32_t step_increment = 250;     // 階梯增量加大到 250 (原本50)
    const uint32_t step_interval_ms = 1000; // 時間間隔縮短到 1秒 (原本4秒)
    static uint32_t step_timer = 0;

    // Mode 4: 自定義時序變數
    static uint32_t m4_timer = 0;
    const uint32_t m4_step_duration = 1000; // 1000ms
    const int32_t m4_spd_low = 100;
    const int32_t m4_spd_mid = 200;
    const int32_t m4_spd_high = 300;
    const int32_t m4_spd_mid_neg = -200;

    // 偵測模式切換，重置狀態
    if (test_mode_selection != last_mode) {
        // 重置 Mode 2 狀態
        ramp_current_rpm = 0.0f;

        // 重置 Mode 3 狀態
        step_current_rpm = 0;
        step_timer = 0;

        // 重置 Mode 4 狀態
        m4_timer = 0;

        // 如果切換回 Mode 1，使用設定的目標速度
        if (test_mode_selection == 1) {
            test_target_rpm_m1 = mode1_target_rpm;
            test_target_rpm_m2 = mode1_target_rpm;
            test_target_rpm_m3 = mode1_target_rpm;
        }

        last_mode = test_mode_selection;
    }

    // 根據模式執行邏輯
    switch (test_mode_selection) {
        case 1: // 定速模式 (可調)
            break;

        case 2: // 線性加速 (10秒到5000)
            if (ramp_current_rpm < ramp_target_rpm) {
                ramp_current_rpm += ramp_step;
                if (ramp_current_rpm > ramp_target_rpm)
                    ramp_current_rpm = ramp_target_rpm;
                test_target_rpm_m1 = (int32_t)ramp_current_rpm;
                test_target_rpm_m2 = (int32_t)ramp_current_rpm; // M2 同步
            }
            break;

        case 3: // 階梯加速 (每1秒+250)
            step_timer++;
            if (step_timer >= step_interval_ms) {
                step_timer = 0;
                if (step_current_rpm < step_target_rpm) {
                    step_current_rpm += step_increment;
                    if (step_current_rpm > step_target_rpm)
                        step_current_rpm = step_target_rpm;
                    test_target_rpm_m1 = step_current_rpm;
                    test_target_rpm_m2 = step_current_rpm; // M2 同步
                }
            }
            break;

        case 4: // 自定義時序模式
            m4_timer++;
            {
                uint32_t period = m4_step_duration * 3;
                uint32_t current_time = m4_timer % period;

                if (current_time < m4_step_duration) {
                    test_target_rpm_m1 = m4_spd_mid;
                    test_target_rpm_m2 = 0;
                    test_target_rpm_m3 = m4_spd_low;
                } else if (current_time < m4_step_duration * 2) {
                    test_target_rpm_m1 = m4_spd_low;
                    test_target_rpm_m2 = m4_spd_mid;
                    test_target_rpm_m3 = m4_spd_mid;
                } else {
                    test_target_rpm_m1 = m4_spd_high;
                    test_target_rpm_m2 = 0;
                    test_target_rpm_m3 = m4_spd_mid_neg;
                }
            }
            break;
        default:
            break;
    }

    if (!RobotSafety_CanDrive()) {
        Motor_Stop(&motor_joint_13pin);
        Motor_Stop(&motor_joint_13pin_2);
        Motor_Stop(&motor_joint_13pin_3);
        return;
    }
    
    Motor_Update(&motor_joint_13pin);
    Motor_Update(&motor_joint_13pin_2);
    Motor_Update(&motor_joint_13pin_3);

    robot_state.real_theta1 = Motor_GetAngle(&motor_joint_13pin);
    robot_state.real_theta2 = Motor_GetAngle(&motor_joint_13pin_2);
    robot_state.real_theta3 = Motor_GetAngle(&motor_joint_13pin_3);

    robot_state.real_rpm1 = Motor_GetVelocity(&motor_joint_13pin);
    robot_state.real_rpm2 = Motor_GetVelocity(&motor_joint_13pin_2);
    robot_state.real_rpm3 = Motor_GetVelocity(&motor_joint_13pin_3);

    Motor_Start(&motor_joint_13pin);
    Motor_Start(&motor_joint_13pin_2);
    Motor_Start(&motor_joint_13pin_3);

    Motor_SetSpeed(&motor_joint_13pin, test_target_rpm_m1);
    Motor_SetSpeed(&motor_joint_13pin_2, test_target_rpm_m2);
    Motor_SetSpeed(&motor_joint_13pin_3, test_target_rpm_m3);
}
