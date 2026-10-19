#include "mainpp.h"
#include "motor_config.h"
#include "nidec_motor_driver.h"
#include "safety_manager.h"
#include "homing_calibration.h"
#include "test_scenarios.h"
#include "robot_interface.h"
#include "microros_app.h"
#include "hardware_io.h"
#include "runtime_monitor.h"
#include "robot_config.h"
#include "motor_parameters.h"
#include "joint_units.h"
#include "trajectory_planner.h"
#include <cmath>

static uint32_t g_current_time_ms = 0;

// Control-state ownership is local to the 1 kHz robot loop.  A session or
// mode transition must never inherit interpolation velocity or PID history
// from the previous command stream.
static uint32_t g_motion_session_id = 0U;
static bool g_motion_session_initialized = false;
static RobotControlMode_t g_last_control_mode = CONTROL_MODE_DISABLED;
static bool g_last_control_mode_valid = false;
static uint8_t g_requested_enable_mask = 0U;
static uint8_t g_applied_enable_mask = 0U;
static uint8_t g_physical_enable_mask = 0U;

static void ResetMotionControlState(void) {
    g_pva_interpolator.reset();
    joint1_pid.reset(robot_state.real_theta1);
    joint2_pid.reset(robot_state.real_theta2);
    joint3_pid.reset(robot_state.real_theta3);
    robot_state.traj_vel1 = 0.0f;
    robot_state.traj_vel2 = 0.0f;
    robot_state.traj_vel3 = 0.0f;
}

static void ResetMotionControlTracking(void) {
    ResetMotionControlState();
    g_last_control_mode_valid = false;
}

extern "C" void Robot_ControlStateAbort(void) {
    test_mode_selection = 0;
    ResetMotionControlTracking();
    robot_state.pid_output_rpm1 = 0.0f;
    robot_state.pid_output_rpm2 = 0.0f;
    robot_state.pid_output_rpm3 = 0.0f;
    g_requested_enable_mask = 0U;
    g_applied_enable_mask = 0U;
    g_physical_enable_mask = 0U;
    RobotInterface_InvalidateCommand();
    RobotSafety_Stop();
}

// 供未來通訊層 (如 UART/micro-ROS) 呼叫的測試指令產生器入口
extern "C" void Mock_TestCommand_Producer(uint8_t target_axis, int32_t target_rpm) {
    RobotCommand_t cmd = {}; // Zero initialization
    cmd.test_mode = 1;       // Single-axis test mode
    cmd.test_axis = target_axis;
    cmd.test_rpm = target_rpm;
    
    RobotInterface_SetCommand(&cmd);
}

extern "C" void Robot_Init(void) {
    HardwareIO_Init();
    Motor_System_Config();
    RobotSafety_Init();
    RobotInterface_Init();

    test_mode_selection = 0;

    joint1_pid.reset();
    joint2_pid.reset();
    joint3_pid.reset();

    g_motion_session_id = RobotInterface_GetSessionId();
    g_motion_session_initialized = true;
    g_last_control_mode = CONTROL_MODE_DISABLED;
    g_last_control_mode_valid = false;
    g_requested_enable_mask = 0U;
    g_applied_enable_mask = 0U;
    g_physical_enable_mask = 0U;
    
    // 統一呼叫 Robot_HomingStart，內部已處理 bypass_homing 分支
    Robot_HomingStart();
}

static void Robot_ReadFeedback(void) {
    Motor_Update(&motor_joint_13pin);
    Motor_Update(&motor_joint_13pin_2);
    Motor_Update(&motor_joint_13pin_3);

    robot_state.real_theta1 = Motor_GetAngle(&motor_joint_13pin);
    robot_state.real_theta2 = Motor_GetAngle(&motor_joint_13pin_2);
    robot_state.real_theta3 = Motor_GetAngle(&motor_joint_13pin_3);
    
    robot_state.real_rpm1 = Motor_GetVelocity(&motor_joint_13pin);
    robot_state.real_rpm2 = Motor_GetVelocity(&motor_joint_13pin_2);
    robot_state.real_rpm3 = Motor_GetVelocity(&motor_joint_13pin_3);
}

// Performs directional soft-clamping. If a limit switch is pressed and the motor is trying to push into it,
// we force the requested RPM to 0.0f to stop movement in that direction, but allow movement away from it.
// We do NOT trigger a hard fault, to allow the ROS planner to recover.
static void Robot_ApplyDirectionalLimitClamps(float& req_rpm1, float& req_rpm2, float& req_rpm3) {
    float sign1 = (home_search_rpm_m1 >= 0) ? 1.0f : -1.0f;
    float sign2 = (home_search_rpm_m2 >= 0) ? 1.0f : -1.0f;
    float sign3 = (home_search_rpm_m3 >= 0) ? 1.0f : -1.0f;

    // We no longer require a 20.0f threshold since we are not throwing a hard fault.
    // Any push into the limit switch (> 0.0f) is clamped.
    if ((g_requested_enable_mask & (1U << 0)) && homing_joint1_done && HW_IsHomeSwitchTriggered(HW_JOINT_1)) {
        if ((req_rpm1 * sign1) > 0.0f) {
            req_rpm1 = 0.0f;
        }
    }
    if ((g_requested_enable_mask & (1U << 1)) && homing_joint2_done && HW_IsHomeSwitchTriggered(HW_JOINT_2)) {
        if ((req_rpm2 * sign2) > 0.0f) {
            req_rpm2 = 0.0f;
        }
    }
    if ((g_requested_enable_mask & (1U << 2)) && homing_joint3_done && HW_IsHomeSwitchTriggered(HW_JOINT_3)) {
        if ((req_rpm3 * sign3) > 0.0f) {
            req_rpm3 = 0.0f;
        }
    }
}

// Performs directional soft-clamping for software angle limits.
static void Robot_ApplyDirectionalSoftwareClamps(float& req_rpm1, float& req_rpm2, float& req_rpm3) {
    const float tolerance = 5.0f;

    // Joint 1: 0.0 to 111.0
    if (homing_joint1_done) {
        if (robot_state.real_theta1 < (JOINT1_MIN_DEG - tolerance) && req_rpm1 < 0.0f) {
            req_rpm1 = 0.0f; // Prevent moving further negative
        } else if (robot_state.real_theta1 > (JOINT1_MAX_DEG + tolerance) && req_rpm1 > 0.0f) {
            req_rpm1 = 0.0f; // Prevent moving further positive
        }
    }

    // Joint 2: 110.0 to 180.0
    if (homing_joint2_done) {
        if (robot_state.real_theta2 < (JOINT2_MIN_DEG - tolerance) && req_rpm2 < 0.0f) {
            req_rpm2 = 0.0f;
        } else if (robot_state.real_theta2 > (JOINT2_MAX_DEG + tolerance) && req_rpm2 > 0.0f) {
            req_rpm2 = 0.0f;
        }
    }

    // Joint 3: -180.0 to 180.0
    if (homing_joint3_done) {
        if (robot_state.real_theta3 < (JOINT3_MIN_DEG - tolerance) && req_rpm3 < 0.0f) {
            req_rpm3 = 0.0f;
        } else if (robot_state.real_theta3 > (JOINT3_MAX_DEG + tolerance) && req_rpm3 > 0.0f) {
            req_rpm3 = 0.0f;
        }
    }
}

static void Robot_PublishTelemetry(void) {
    RobotTelemetry_t telem = {};
    
    switch (RobotSafety_GetState()) {
        case ROBOT_STATE_SAFE:
        case ROBOT_STATE_READY:
            telem.system_state = ROBOT_SYS_STATE_IDLE;
            break;
        case ROBOT_STATE_HOMING:
            telem.system_state = ROBOT_SYS_STATE_HOMING;
            break;
        case ROBOT_STATE_RUNNING:
            telem.system_state = ROBOT_SYS_STATE_RUNNING;
            break;
        case ROBOT_STATE_FAULT:
        case ROBOT_STATE_ESTOP:
        default:
            telem.system_state = ROBOT_SYS_STATE_FAULT;
            break;
    }
    
    telem.real_theta1 = robot_state.real_theta1;
    telem.real_theta2 = robot_state.real_theta2;
    telem.real_theta3 = robot_state.real_theta3;
    telem.real_rpm1 = robot_state.real_rpm1;
    telem.real_rpm2 = robot_state.real_rpm2;
    telem.real_rpm3 = robot_state.real_rpm3;

    RobotCommand_t active_cmd = {};
    telem.has_command = RobotInterface_UpdateCommand(&active_cmd);
    telem.control_mode = telem.has_command ? active_cmd.control_mode : CONTROL_MODE_DISABLED;
    telem.active_command_sequence = telem.has_command ? active_cmd.sequence : 0U;
    telem.command_received_tick = telem.has_command ? active_cmd.timestamp_ms : 0U;

    const JointPvaState& applied = g_pva_interpolator.getState();
    for (int i = 0; i < 3; ++i) {
        telem.applied_position_rad[i] = applied.pos_rad[i];
        telem.applied_velocity_rad_s[i] = applied.vel_rad_s[i];
    }

    telem.requested_enable_mask = g_requested_enable_mask;
    telem.applied_enable_mask = g_applied_enable_mask;
    telem.physical_enable_mask = g_physical_enable_mask;

    if (telem.has_command && active_cmd.control_mode == CONTROL_MODE_JOINT_PVA) {
        for (int i = 0; i < 3; ++i) {
            telem.target_position_rad[i] = active_cmd.joint_pva.position_rad[i];
            telem.target_velocity_rad_s[i] = active_cmd.joint_pva.velocity_rad_s[i];
            telem.target_acceleration_rad_s2[i] = active_cmd.joint_pva.acceleration_rad_s2[i];
        }
    }

    // This is the command produced by the validated controller.  It is kept
    // independent from measured RPM and remains observable in shadow mode.
    telem.commanded_motor_rpm[0] = robot_state.pid_output_rpm1;
    telem.commanded_motor_rpm[1] = robot_state.pid_output_rpm2;
    telem.commanded_motor_rpm[2] = robot_state.pid_output_rpm3;

    // Motor_Update() maintains these cumulative counts.  The timer counter is
    // only a modulo snapshot and must not be exposed as encoder_count.
    telem.motor1_count = Motor_GetTotalPulseCount(&motor_joint_13pin);
    telem.motor2_count = Motor_GetTotalPulseCount(&motor_joint_13pin_2);
    telem.motor3_count = Motor_GetTotalPulseCount(&motor_joint_13pin_3);
    telem.current_time_ms = g_current_time_ms;
    
    RobotInterface_SetTelemetry(&telem);
}


static bool ApplyCommand(const RobotCommand_t& cmd) {
    if (cmd.control_mode == CONTROL_MODE_DISABLED && cmd.test_mode == 0) {
        test_mode_selection = 0;
        return true;
    }
    if (cmd.control_mode == CONTROL_MODE_HOLD || cmd.control_mode == CONTROL_MODE_STOP) {
        test_mode_selection = 0;
        return true;
    }
    if (cmd.control_mode == CONTROL_MODE_JOINT_PVA) {
        test_mode_selection = 0;
        return RobotSafety_RequestRun();
    }
    const bool legacy_motor_test =
        cmd.control_mode == CONTROL_MODE_DISABLED && cmd.test_mode == 1;
    if (cmd.control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST || legacy_motor_test) {
        if (cmd.control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST && !cmd.motor_test.enable) {
            // An explicit disable is a safety stop.  It must not fall back to
            // the legacy test_mode fields.
            Robot_ControlStateAbort();
            return true;
        }
        if (legacy_motor_test && (cmd.test_axis >= 3U || cmd.test_rpm < -100 || cmd.test_rpm > 100)) {
            return false;
        }
        test_mode_selection = 1;
        return RobotSafety_RequestRun();
    }

    return false;
}

static bool MotorOutputsAreValid(void) {
    constexpr float BRINGUP_CAP = BRINGUP_MAX_MOTOR_RPM;
    return std::isfinite(robot_state.pid_output_rpm1) &&
           std::isfinite(robot_state.pid_output_rpm2) &&
           std::isfinite(robot_state.pid_output_rpm3) &&
           std::fabs(robot_state.pid_output_rpm1) <= BRINGUP_CAP &&
           std::fabs(robot_state.pid_output_rpm2) <= BRINGUP_CAP &&
           std::fabs(robot_state.pid_output_rpm3) <= BRINGUP_CAP;
}

/**
 * @brief Audited Safety Gate Wrapper for Motor_Start.
 * Line k contains RobotSafety_CanDrive() which directly evaluates and controls execution.
 * Line k+1 executes Motor_Start.
 */
static inline void Gate_Motor_Start(Motor_t* motor) {
    if (!ACTUATOR_HARDWARE_OUTPUT_ENABLED) return;
    if (RobotSafety_CanDrive())
        Motor_Start(motor);
}

/**
 * @brief Audited Safety Gate Wrapper for Motor_SetSpeed.
 * Line k contains RobotSafety_CanDrive() which directly evaluates and controls execution.
 * Line k+1 executes Motor_SetSpeed.
 */
static inline void Gate_Motor_SetSpeed(Motor_t* motor, int32_t speed_rpm) {
    if (!ACTUATOR_HARDWARE_OUTPUT_ENABLED) return;
    if (RobotSafety_CanDrive())
        Motor_SetSpeed(motor, speed_rpm);
}

/**
 * @brief Documented Safe-Stop Exception Wrapper for Motor_Stop.
 * Unconditionally stops physical motor when an axis is disabled, faulted, or excluded.
 */
static inline void SafeStop_Motor(Motor_t* motor) {
    Motor_Stop(motor);
}

/**
 * @brief Master Actuator Output Function.
 * Single audited entrance for physical actuator operations.
 * Evaluates master hardware enable, safety drive state, and enable mask.
 */
static void UltimateGate_ApplyActuatorCommand(uint8_t enable_mask, float rpm1, float rpm2, float rpm3) {
    g_physical_enable_mask = 0U;
    if (!ACTUATOR_HARDWARE_OUTPUT_ENABLED || !RobotSafety_CanDrive() || RobotSafety_IsFaulted()) {
        SafeStop_Motor(&motor_joint_13pin);
        SafeStop_Motor(&motor_joint_13pin_2);
        SafeStop_Motor(&motor_joint_13pin_3);
        return;
    }

    // Violent safety validation of outputs before casting
    if (!std::isfinite(rpm1) || !std::isfinite(rpm2) || !std::isfinite(rpm3)) {
        SafeStop_Motor(&motor_joint_13pin);
        SafeStop_Motor(&motor_joint_13pin_2);
        SafeStop_Motor(&motor_joint_13pin_3);
        return;
    }

    constexpr float max_rpm = BRINGUP_MAX_MOTOR_RPM;
    float safe_rpm1 = rpm1;
    float safe_rpm2 = rpm2;
    float safe_rpm3 = rpm3;

    // Hard clamp to Bringup Max RPM limits
    if (safe_rpm1 > max_rpm) safe_rpm1 = max_rpm;
    else if (safe_rpm1 < -max_rpm) safe_rpm1 = -max_rpm;
    
    if (safe_rpm2 > max_rpm) safe_rpm2 = max_rpm;
    else if (safe_rpm2 < -max_rpm) safe_rpm2 = -max_rpm;
    
    if (safe_rpm3 > max_rpm) safe_rpm3 = max_rpm;
    else if (safe_rpm3 < -max_rpm) safe_rpm3 = -max_rpm;

    g_physical_enable_mask = enable_mask & 0x07U;

    // Axis 1 Actuation
    if (enable_mask & (1U << 0)) {
        Gate_Motor_Start(&motor_joint_13pin);
        Gate_Motor_SetSpeed(&motor_joint_13pin, (int32_t)safe_rpm1);
    } else {
        SafeStop_Motor(&motor_joint_13pin);
    }

    // Axis 2 Actuation
    if (enable_mask & (1U << 1)) {
        Gate_Motor_Start(&motor_joint_13pin_2);
        Gate_Motor_SetSpeed(&motor_joint_13pin_2, (int32_t)safe_rpm2);
    } else {
        SafeStop_Motor(&motor_joint_13pin_2);
    }

    // Axis 3 Actuation
    if (enable_mask & (1U << 2)) {
        Gate_Motor_Start(&motor_joint_13pin_3);
        Gate_Motor_SetSpeed(&motor_joint_13pin_3, (int32_t)safe_rpm3);
    } else {
        SafeStop_Motor(&motor_joint_13pin_3);
    }
}

// ==========================================================
// STM32CubeIDE 現場表達式 (Live Expressions) 直接硬體除錯變數
// ==========================================================
volatile int32_t live_test_enable = 0; // 設為 1 即可在 Live Expressions 任意手動輸入轉速
volatile int32_t live_test_rpm1 = 0;   // 軸 1 馬達 RPM（-150 ~ +150）
volatile int32_t live_test_rpm2 = 0;   // 軸 2 馬達 RPM（-150 ~ +150）
volatile int32_t live_test_rpm3 = 0;   // 軸 3 馬達 RPM（-150 ~ +150）

extern "C" void Robot_Loop(float dt_seconds) {
    uint8_t motor_enable_mask = 0U;
    float req_rpm1 = 0.0f;
    float req_rpm2 = 0.0f;
    float req_rpm3 = 0.0f;
    g_requested_enable_mask = 0U;
    g_applied_enable_mask = 0U;
    g_physical_enable_mask = 0U;

    // --- STM32CubeIDE 現場表達式直接手動控制分支 ---
    if (live_test_enable == 1) {
        RobotSafety_ClearFault();
        uint8_t mask = 0U;
        if (live_test_rpm1 != 0) mask |= (1U << 0);
        if (live_test_rpm2 != 0) mask |= (1U << 1);
        if (live_test_rpm3 != 0) mask |= (1U << 2);

        int32_t r1 = live_test_rpm1;
        int32_t r2 = live_test_rpm2;
        int32_t r3 = live_test_rpm3;
        const int32_t max_rpm = (int32_t)BRINGUP_MAX_MOTOR_RPM;
        if (r1 > max_rpm) r1 = max_rpm;
        if (r1 < -max_rpm) r1 = -max_rpm;
        if (r2 > max_rpm) r2 = max_rpm;
        if (r2 < -max_rpm) r2 = -max_rpm;
        if (r3 > max_rpm) r3 = max_rpm;
        if (r3 < -max_rpm) r3 = -max_rpm;

        g_requested_enable_mask = mask;
        g_applied_enable_mask = mask;
        g_physical_enable_mask = mask;

        if (mask & (1U << 0)) {
            Gate_Motor_Start(&motor_joint_13pin);
            Gate_Motor_SetSpeed(&motor_joint_13pin, r1);
        } else {
            SafeStop_Motor(&motor_joint_13pin);
        }
        if (mask & (1U << 1)) {
            Gate_Motor_Start(&motor_joint_13pin_2);
            Gate_Motor_SetSpeed(&motor_joint_13pin_2, r2);
        } else {
            SafeStop_Motor(&motor_joint_13pin_2);
        }
        if (mask & (1U << 2)) {
            Gate_Motor_Start(&motor_joint_13pin_3);
            Gate_Motor_SetSpeed(&motor_joint_13pin_3, r3);
        } else {
            SafeStop_Motor(&motor_joint_13pin_3);
        }
        return;
    }

    const uint32_t active_session = RobotInterface_GetSessionId();
    if (!g_motion_session_initialized || active_session != g_motion_session_id) {
        ResetMotionControlTracking();
        g_motion_session_id = active_session;
        g_motion_session_initialized = true;
    }

    if (dt_seconds <= 0.0f || !std::isfinite(dt_seconds)) {
        ResetMotionControlTracking();
        RobotSafety_TriggerFault(ROBOT_FAULT_TRAJECTORY);
    } else {
        if (!RobotSafety_CanExecute()) {
            ResetMotionControlTracking();
            RobotSafety_Stop();
        } else {
            RobotCommand_t cmd;
            bool has_cmd = RobotInterface_UpdateCommand(&cmd);

            if (!has_cmd) {
                ResetMotionControlTracking();
                RobotSafety_Stop();
            } else {
                if (cmd.control_mode == CONTROL_MODE_JOINT_PVA) {
                    g_requested_enable_mask = cmd.joint_pva.enable_mask & 0x07U;
                } else if (cmd.control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST) {
                    g_requested_enable_mask = cmd.motor_test.enable
                        ? (uint8_t)(1U << cmd.motor_test.axis) : 0U;
                } else if (cmd.control_mode == CONTROL_MODE_DISABLED && cmd.test_mode == 1 &&
                           cmd.test_axis < 3U) {
                    g_requested_enable_mask = (uint8_t)(1U << cmd.test_axis);
                }
                // Legacy test_mode uses CONTROL_MODE_DISABLED as its wire value;
                // compare the effective mode so leaving that path also clears
                // the previous servo history.
                const RobotControlMode_t effective_mode =
                    (((cmd.control_mode == CONTROL_MODE_DISABLED && cmd.test_mode == 1) ||
                      cmd.control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST))
                        ? CONTROL_MODE_MOTOR_VELOCITY_TEST
                        : cmd.control_mode;
                const bool mode_changed = !g_last_control_mode_valid ||
                                          effective_mode != g_last_control_mode;
                if (mode_changed) {
                    ResetMotionControlState();
                }

                if (!ApplyCommand(cmd)) {
                    ResetMotionControlTracking();
                    RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
                } else {
                    g_last_control_mode = effective_mode;
                    g_last_control_mode_valid = true;

                    if (cmd.control_mode == CONTROL_MODE_HOLD ||
                        cmd.control_mode == CONTROL_MODE_STOP ||
                        (cmd.control_mode == CONTROL_MODE_DISABLED && cmd.test_mode == 0)) {
                        ResetMotionControlState();
                        if (cmd.control_mode == CONTROL_MODE_STOP) {
                            RobotSafety_Stop();
                        }
                    } else if (cmd.control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST && !cmd.motor_test.enable) {
                        // Explicit MotorTest disable is an all-axis stop.
                        Robot_ControlStateAbort();
                    } else if (cmd.control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST || test_mode_selection == 1) {
                        MotorCommand_t test_cmd = {0};
                        const bool enabled = (cmd.control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST)
                            ? cmd.motor_test.enable : true;
                        const uint8_t target_axis = (cmd.control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST)
                            ? cmd.motor_test.axis : cmd.test_axis;
                        const int32_t target_rpm = (cmd.control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST)
                            ? (int32_t)cmd.motor_test.target_motor_rpm : cmd.test_rpm;

                        if (!Robot_TestMode_BuildCommand(enabled, target_axis, target_rpm, &test_cmd)) {
                            RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
                        } else {
                            req_rpm1 = (float)test_cmd.rpm[0];
                            req_rpm2 = (float)test_cmd.rpm[1];
                            req_rpm3 = (float)test_cmd.rpm[2];
                            motor_enable_mask = test_cmd.enable_mask;
                        }
                    } else if (cmd.control_mode == CONTROL_MODE_JOINT_PVA) {
                        if (!g_pva_interpolator.isActive()) {
                            g_pva_interpolator.init(
                                Joint_Deg_To_Rad(robot_state.real_theta1),
                                Joint_Deg_To_Rad(robot_state.real_theta2),
                                Joint_Deg_To_Rad(robot_state.real_theta3)
                            );
                        }

                        JointPvaState pva_state;
                        if (!g_pva_interpolator.update(cmd.joint_pva, dt_seconds, pva_state)) {
                            RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
                        } else {
                            float p1_deg = Joint_Rad_To_Deg(pva_state.pos_rad[0]);
                            float v1_deg_s = Joint_Rad_To_Deg(pva_state.vel_rad_s[0]);
                            float a1_deg_s2 = Joint_Rad_To_Deg(pva_state.acc_rad_s2[0]);

                            float p2_deg = Joint_Rad_To_Deg(pva_state.pos_rad[1]);
                            float v2_deg_s = Joint_Rad_To_Deg(pva_state.vel_rad_s[1]);
                            float a2_deg_s2 = Joint_Rad_To_Deg(pva_state.acc_rad_s2[1]);

                            float p3_deg = Joint_Rad_To_Deg(pva_state.pos_rad[2]);
                            float v3_deg_s = Joint_Rad_To_Deg(pva_state.vel_rad_s[2]);
                            float a3_deg_s2 = Joint_Rad_To_Deg(pva_state.acc_rad_s2[2]);

                            robot_state.traj_vel1 = v1_deg_s;
                            robot_state.traj_vel2 = v2_deg_s;
                            robot_state.traj_vel3 = v3_deg_s;

                            req_rpm1 = joint1_pid.update(p1_deg, v1_deg_s, a1_deg_s2, robot_state.real_theta1, dt_seconds);
                            req_rpm2 = joint2_pid.update(p2_deg, v2_deg_s, a2_deg_s2, robot_state.real_theta2, dt_seconds);
                            req_rpm3 = joint3_pid.update(p3_deg, v3_deg_s, a3_deg_s2, robot_state.real_theta3, dt_seconds);

                            motor_enable_mask = cmd.joint_pva.enable_mask;
                        }
                    } else {
                        // Unrecognized or unsupported mode fails closed
                        RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
                    }
                }
            }

            const bool is_test_mode = (cmd.control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST || test_mode_selection == 1);
            if (!is_test_mode) {
                // Apply soft directional limit clamps without throwing a global fault
                // First apply software boundary clamps
                Robot_ApplyDirectionalSoftwareClamps(req_rpm1, req_rpm2, req_rpm3);
                // Then apply hardware limit switch clamps
                Robot_ApplyDirectionalLimitClamps(req_rpm1, req_rpm2, req_rpm3);
            }
            
            robot_state.pid_output_rpm1 = req_rpm1;
            robot_state.pid_output_rpm2 = req_rpm2;
            robot_state.pid_output_rpm3 = req_rpm3;

            if (!MotorOutputsAreValid()) {
                RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
                motor_enable_mask = 0U;
                req_rpm1 = req_rpm2 = req_rpm3 = 0.0f;
            }
        }
    }

    // Keep shadow execution results observable.  Physical permission is
    // checked only by UltimateGate_ApplyActuatorCommand(), immediately before
    // the Motor_* APIs.
    if (RobotSafety_IsFaulted()) {
        motor_enable_mask = 0U;
        req_rpm1 = req_rpm2 = req_rpm3 = 0.0f;
    }

    robot_state.pid_output_rpm1 = req_rpm1;
    robot_state.pid_output_rpm2 = req_rpm2;
    robot_state.pid_output_rpm3 = req_rpm3;

    // This is the logical post-validation/applied mask.  It remains visible
    // in shadow mode, while UltimateGate_ApplyActuatorCommand records the
    // separately gated physical mask.
    g_applied_enable_mask = motor_enable_mask & 0x07U;

    UltimateGate_ApplyActuatorCommand(motor_enable_mask, req_rpm1, req_rpm2, req_rpm3);
}



extern "C" void Robot_ControlTask_Tick(float dt_seconds) {
    g_current_time_ms += (uint32_t)(dt_seconds * 1000.0f);
    
    Robot_ReadFeedback();
    
    // 注意：Deadline miss 觸發邏輯被移至 freertos.c 的取樣函式呼叫後直接處理
    // if (RuntimeMonitor_GetConsecutiveMisses() >= 3) { ... }
    
    uint32_t exti_events = HW_TakeHomeEvents();
    if (exti_events & (1u << HW_JOINT_1)) Robot_HandleHomeSwitch(JOINT1_HOME_PIN);
    if (exti_events & (1u << HW_JOINT_2)) Robot_HandleHomeSwitch(JOINT2_HOME_PIN);
    if (exti_events & (1u << HW_JOINT_3)) Robot_HandleHomeSwitch(JOINT3_HOME_PIN);

    RobotSafetyState_t safety = RobotSafety_GetState();
    switch (safety) {
        case ROBOT_STATE_HOMING:
            Robot_Homing_Tick();
            break;

        case ROBOT_STATE_READY:
        case ROBOT_STATE_RUNNING:
            Robot_Loop(dt_seconds);
            break;

        case ROBOT_STATE_SAFE:
            // SAFE disables physical drive in a shadow build but must not
            // disable validation, interpolation, PID, or telemetry snapshots.
            Robot_Loop(dt_seconds);
            break;
            
        case ROBOT_STATE_FAULT:
        case ROBOT_STATE_ESTOP:
            break;
    }
    
    Robot_PublishTelemetry();
}
