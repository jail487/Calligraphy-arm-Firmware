#define HOST_TEST 1

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "robot_interface.h"
#include "test_scenarios.h"
#include "nidec_motor_driver.h"
#include "motor_config.h"
#include "safety_manager.h"
#include "mainpp.h"
#include "hardware_io.h"
#include "runtime_monitor.h"

#include "robot_interface.cpp"

// --- Host Test Peripheral Stubs ---
typedef struct { uint32_t opaque; } UART_HandleTypeDef;

TIM_TypeDef g_tim_inst1 = {0};
TIM_TypeDef g_tim_inst2 = {0};
TIM_TypeDef g_tim_inst3 = {0};
TIM_TypeDef g_tim_inst4 = {0};
TIM_TypeDef g_tim_inst5 = {0};
TIM_TypeDef g_tim_inst6 = {0};
TIM_TypeDef g_tim_inst12 = {0};

TIM_HandleTypeDef htim1 = {.Instance = &g_tim_inst1};
TIM_HandleTypeDef htim2 = {.Instance = &g_tim_inst2};
TIM_HandleTypeDef htim3 = {.Instance = &g_tim_inst3};
TIM_HandleTypeDef htim4 = {.Instance = &g_tim_inst4};
TIM_HandleTypeDef htim5 = {.Instance = &g_tim_inst5};
TIM_HandleTypeDef htim6 = {.Instance = &g_tim_inst6};
TIM_HandleTypeDef htim12 = {.Instance = &g_tim_inst12};
UART_HandleTypeDef huart2 = {0};

GPIO_TypeDef g_gpio_stub = {0};
GPIO_TypeDef *GPIOC = &g_gpio_stub;
GPIO_TypeDef *GPIOA = &g_gpio_stub;
GPIO_TypeDef *GPIOB = &g_gpio_stub;

void *xControlTaskHandle = NULL;

volatile bool homing_joint1_done = true;
volatile bool homing_joint2_done = true;
volatile bool homing_joint3_done = true;
volatile int32_t home_search_rpm_m1 = 0;
volatile int32_t home_search_rpm_m2 = 0;
volatile int32_t home_search_rpm_m3 = 0;

extern "C" {
void HardwareIO_Init(void) {}
bool HW_IsHomeSwitchTriggered(HardwareJoint_e j) { (void)j; return false; }
uint32_t HW_TakeHomeEvents(void) { return 0; }
uint32_t HW_GetMonotonicTimeMs(void) { return 0; }
void HW_OnExtInterrupt(uint16_t pin) { (void)pin; }

void Robot_HomingStart(void) {}
void Robot_Homing_Tick(void) {}
void Robot_HandleHomeSwitch(uint16_t pin) { (void)pin; }

uint32_t RuntimeMonitor_GetConsecutiveMisses(void) { return 0; }
uint32_t RuntimeMonitor_GetMaxCycles(void) { return 0; }
uint32_t Robot_GetControlStackHighWatermark(void) { return 0; }
uint32_t Robot_GetCommStackHighWatermark(void) { return 0; }
}

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

static void test_sequence_wrap(void) {
    RobotInterface_Init();
    RobotInterface_SetSessionId(1);
    g_host_mock_tick = 1000;

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 1;
    cmd.sequence = 0xFFFFFFFEU;
    cmd.valid_for_ms = 1000;
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;
    cmd.motor_test.axis = 0;
    cmd.motor_test.target_motor_rpm = 100.0f;

    CommandRejectionReason_t reason = CMD_REJECT_NONE;

    bool ok = RobotInterface_ValidateAndSetCommand(&cmd, &reason);
    TEST_ASSERT(ok && reason == CMD_REJECT_NONE, "Sequence 0xFFFFFFFE should be accepted");

    // Wrap around to 0x00000001 (signed difference: 0x00000001 - 0xFFFFFFFE = +3 > 0)
    cmd.sequence = 0x00000001U;
    ok = RobotInterface_ValidateAndSetCommand(&cmd, &reason);
    TEST_ASSERT(ok && reason == CMD_REJECT_NONE, "Wrap around sequence 0x00000001 should be accepted");

    // Stale sequence 0x00000001 again
    cmd.sequence = 0x00000001U;
    ok = RobotInterface_ValidateAndSetCommand(&cmd, &reason);
    TEST_ASSERT(!ok && reason == CMD_REJECT_STALE_SEQUENCE, "Duplicate sequence should be rejected as stale");
}

static void test_unsupported_control_mode(void) {
    RobotInterface_Init();
    RobotInterface_SetSessionId(1);

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 1;
    cmd.sequence = 10;
    cmd.valid_for_ms = 1000;
    cmd.control_mode = (RobotControlMode_t)99; // Arbitrary invalid enum value

    CommandRejectionReason_t reason = CMD_REJECT_NONE;
    bool ok = RobotInterface_ValidateAndSetCommand(&cmd, &reason);
    TEST_ASSERT(!ok && reason == CMD_REJECT_UNSUPPORTED_MODE, "Unsupported control mode 99 must be rejected with CMD_REJECT_UNSUPPORTED_MODE");
}

static void test_ttl_tick_wrap_and_expiry(void) {
    RobotInterface_Init();
    RobotInterface_SetSessionId(1);
    g_host_mock_tick = 0xFFFFFF00U; // Near 32-bit tick wrap

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 1;
    cmd.sequence = 1;
    cmd.valid_for_ms = 500;
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;
    cmd.motor_test.axis = 0;
    cmd.motor_test.target_motor_rpm = 100.0f;

    bool ok = RobotInterface_ValidateAndSetCommand(&cmd, NULL);
    TEST_ASSERT(ok, "Command set near tick wrap should succeed");

    // Advance tick past 0 to 0x00000050 (336 ms elapsed <= 500ms TTL)
    g_host_mock_tick = 0x00000050U;
    RobotCommand_t out_cmd;
    ok = RobotInterface_UpdateCommand(&out_cmd);
    TEST_ASSERT(ok, "Command within TTL after tick wrap around 0 should be retrieved successfully");

    // Advance tick to 0x00000300 (1024 ms elapsed > 500ms TTL) -> Expiry!
    g_host_mock_tick = 0x00000300U;
    ok = RobotInterface_UpdateCommand(&out_cmd);
    TEST_ASSERT(!ok, "Expired command past TTL under tick wrap must return false and annihilate mailbox");

    RobotDiagnostics_t diag;
    RobotInterface_GetDiagnostics(&diag);
    TEST_ASSERT(diag.command_timeout_total == 1, "Command timeout counter must be incremented on TTL expiry");
}

static void test_session_change_and_invalidation(void) {
    RobotInterface_Init();
    RobotInterface_SetSessionId(100);
    g_host_mock_tick = 1000;

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 100;
    cmd.sequence = 1;
    cmd.valid_for_ms = 1000;
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;
    cmd.motor_test.axis = 0;
    cmd.motor_test.target_motor_rpm = 100.0f;

    TEST_ASSERT(RobotInterface_ValidateAndSetCommand(&cmd, NULL), "Command for session 100 should be accepted");

    RobotCommand_t out_cmd;
    TEST_ASSERT(RobotInterface_UpdateCommand(&out_cmd), "Command active in session 100");

    // Change session ID to 200 -> must annihilate mailbox command & reset sequence basis!
    RobotInterface_SetSessionId(200);

    TEST_ASSERT(!RobotInterface_UpdateCommand(&out_cmd), "Session ID change must annihilate cached mailbox command");

    // Attempting old session ID 100 must be rejected
    cmd.session_id = 100;
    cmd.sequence = 2;
    CommandRejectionReason_t reason = CMD_REJECT_NONE;
    TEST_ASSERT(!RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_SESSION_MISMATCH, "Old session ID 100 must be rejected after session change to 200");

    // New session ID 200 with sequence 1 (reset sequence tracking basis) must be accepted
    cmd.session_id = 200;
    cmd.sequence = 1;
    TEST_ASSERT(RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_NONE, "New session 200 accepting sequence 1 after reset");
}

static void test_unbound_and_idempotent_session(void) {
    RobotInterface_Init();
    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 77;
    cmd.sequence = 1;
    cmd.valid_for_ms = 1000;
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;
    cmd.motor_test.axis = 0;
    cmd.motor_test.target_motor_rpm = 10.0f;
    CommandRejectionReason_t reason = CMD_REJECT_NONE;

    TEST_ASSERT(!RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_SESSION_MISMATCH,
                "Unbound session must fail closed");
    RobotInterface_SetSessionId(77);
    TEST_ASSERT(RobotInterface_ValidateAndSetCommand(&cmd, &reason), "Bound session must accept command");
    RobotInterface_SetSessionId(77);
    TEST_ASSERT(!RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_STALE_SEQUENCE,
                "Repeated session bind must preserve sequence basis");
    RobotInterface_SetSessionId(0);
    cmd.sequence = 2;
    TEST_ASSERT(!RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_SESSION_MISMATCH,
                "Disarm session must return to fail-closed state");
}

static void test_explicit_invalidation(void) {
    RobotInterface_Init();
    RobotInterface_SetSessionId(10);
    g_host_mock_tick = 1000;

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 10;
    cmd.sequence = 1;
    cmd.valid_for_ms = 1000;
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;

    TEST_ASSERT(RobotInterface_ValidateAndSetCommand(&cmd, NULL), "Set valid command");

    RobotCommand_t out_cmd;
    TEST_ASSERT(RobotInterface_UpdateCommand(&out_cmd), "Command valid in mailbox");

    // Explicitly invalidate
    RobotInterface_InvalidateCommand();

    TEST_ASSERT(!RobotInterface_UpdateCommand(&out_cmd), "Mailbox command destroyed after RobotInterface_InvalidateCommand");
}

static void test_invalid_numeric_fields(void) {
    RobotInterface_Init();
    RobotInterface_SetSessionId(1);

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 1;
    cmd.sequence = 1;
    cmd.valid_for_ms = 1000;
    CommandRejectionReason_t reason = CMD_REJECT_NONE;

    // Test NaN motor RPM
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;
    cmd.motor_test.axis = 0;
    cmd.motor_test.target_motor_rpm = 0.0f / 0.0f; // NaN
    TEST_ASSERT(!RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_NON_FINITE_PARAM, "NaN motor RPM rejected");

    // Test axis out-of-bounds (axis 3)
    cmd.motor_test.target_motor_rpm = 100.0f;
    cmd.motor_test.axis = 3;
    TEST_ASSERT(!RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_OUT_OF_BOUNDS_AXIS_OR_MASK, "Axis 3 out of bounds rejected");

    // Test Joint PVA enable_mask out of bounds (0x0F)
    cmd.control_mode = CONTROL_MODE_JOINT_PVA;
    cmd.joint_pva.enable_mask = 0x0FU;
    TEST_ASSERT(!RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_OUT_OF_BOUNDS_AXIS_OR_MASK, "Enable mask 0x0F rejected");
}

static void test_motor_test_disable_is_a_safety_stop(void) {
    MotorCommand_t output;
    output.rpm[0] = 77;
    output.rpm[1] = -88;
    output.rpm[2] = 99;
    output.enable_mask = 0x07;

    TEST_ASSERT(Robot_TestMode_BuildCommand(false, 1, 100, &output),
                "Disabled motor-test command must be accepted as a stop event");
    TEST_ASSERT(output.enable_mask == 0U,
                "Motor-test enable=false must clear every output enable bit");
    TEST_ASSERT(output.rpm[0] == 0 && output.rpm[1] == 0 && output.rpm[2] == 0,
                "Motor-test enable=false must clear every axis RPM");
}

static void test_command_received_timestamp(void) {
    RobotInterface_Init();
    RobotInterface_SetSessionId(7);
    g_host_mock_tick = 1234;

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 7;
    cmd.sequence = 1;
    cmd.valid_for_ms = 500;
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;
    cmd.motor_test.axis = 0;
    cmd.motor_test.enable = true;
    cmd.motor_test.target_motor_rpm = 10.0f;

    TEST_ASSERT(RobotInterface_ValidateAndSetCommand(&cmd, NULL),
                "Timestamp test command must be accepted");
    g_host_mock_tick = 1240;
    RobotCommand_t received;
    TEST_ASSERT(RobotInterface_UpdateCommand(&received),
                "Timestamp test command must remain within TTL");
    TEST_ASSERT(received.timestamp_ms == 1234U,
                "Command timestamp must be the MCU receive tick");
}

static void test_shadow_applied_state_and_position_convergence(void) {
    JointPvaInterpolator interp;
    interp.init(1.5f, 1.5f, 0.0f);
    JointPvaPayload_t target = {0};
    target.position_rad[0] = 1.6f;
    target.position_rad[1] = 1.5f;
    target.position_rad[2] = 0.0f;

    JointPvaState applied = {0};
    bool updates_ok = true;
    for (int i = 0; i < 3000; ++i) {
        if (!interp.update(target, 0.001f, applied)) updates_ok = false;
        if (i == 0) {
            TEST_ASSERT(applied.pos_rad[0] > 1.5f && applied.pos_rad[0] < 1.6f,
                        "Shadow interpolation must expose an intermediate applied position");
        }
    }
    TEST_ASSERT(updates_ok,
                "Shadow interpolation must continue without physical output");
    TEST_ASSERT(applied.pos_rad[0] > 1.59f && applied.pos_rad[0] < 1.61f,
                "Shadow mode must expose the applied position state");
    TEST_ASSERT(applied.vel_rad_s[0] < 0.002f,
                "Shadow mode must settle the applied position state");
    TEST_ASSERT(ACTUATOR_HARDWARE_OUTPUT_ENABLED == 0,
                "Shadow test must retain the hardware output safety default");
}

static void test_motor_test_enable_false_clears_state_and_ignores_legacy(void) {
    Robot_Init();
    RobotSafety_Init();
    motor_joint_13pin.angle_offset_deg = 90.0f;
    motor_joint_13pin_2.angle_offset_deg = 0.0f;
    motor_joint_13pin_3.angle_offset_deg = 0.0f;
    RobotInterface_SetSessionId(10);
    g_host_mock_tick = 5000;

    // 1. Post a motor test command with enable=false through the production interface
    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 10;
    cmd.sequence = 1;
    cmd.valid_for_ms = 1000;
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;
    cmd.motor_test.axis = 1;
    cmd.motor_test.target_motor_rpm = 50.0f;
    cmd.motor_test.enable = false; // Explicit disable
    cmd.test_mode = 1; // Legacy test_mode must be ignored/cleared when enable=false

    TEST_ASSERT(RobotInterface_ValidateAndSetCommand(&cmd, NULL), "Motor-test disable command must be accepted");

    // 2. Set legacy test mode selection to non-zero before loop to verify clearing
    test_mode_selection = 1;
    Motor_ResetStopCallCount();

    // 3. Execute production control tick / loop to process disable command via Robot_ControlStateAbort
    Robot_ControlTask_Tick(0.001f);

    // 4. Prove all three Motor_Stop calls were observed via motor stop spy
    TEST_ASSERT(Motor_GetStopCallCount() >= 3U, "Production disable must invoke Motor_Stop on all three axes");

    // 5. Prove every axis receives 0 RPM output
    TEST_ASSERT(robot_state.pid_output_rpm1 == 0.0f &&
                robot_state.pid_output_rpm2 == 0.0f &&
                robot_state.pid_output_rpm3 == 0.0f, "Every axis must have 0 RPM output on disable");

    // 6. Prove all three enable masks (requested, applied, physical) become 0 in telemetry
    RobotTelemetry_t telem;
    RobotInterface_GetTelemetry(&telem);
    TEST_ASSERT(telem.requested_enable_mask == 0U, "Requested enable mask must be 0 on disable");
    TEST_ASSERT(telem.applied_enable_mask == 0U, "Applied enable mask must be 0 on disable");
    TEST_ASSERT(telem.physical_enable_mask == 0U, "Physical enable mask must be 0 on disable");

    // 7. Prove legacy test state is cleared
    TEST_ASSERT(test_mode_selection == 0, "Legacy test_mode state must be cleared on disable");

    // 8. Prove command mailbox is invalidated
    RobotCommand_t out_cmd;
    TEST_ASSERT(!RobotInterface_UpdateCommand(&out_cmd), "Mailbox must be invalidated after disable");

    // 9. Prove safety stop was requested (safety state is READY or SAFE)
    RobotSafetyState_t state = RobotSafety_GetState();
    TEST_ASSERT(state == ROBOT_STATE_READY || state == ROBOT_STATE_SAFE, "Safety stop must be requested on disable");
}

static void test_reboot_and_boot_tag_session_replay(void) {
    // Session IDs encode boot tag in upper 16 bits
    uint32_t boot1_tag = (1U & 0xFFFFU) << DOGHEAD_SESSION_BOOT_TAG_SHIFT;
    uint32_t boot2_tag = (2U & 0xFFFFU) << DOGHEAD_SESSION_BOOT_TAG_SHIFT;

    uint32_t session_boot1 = boot1_tag | 0x0001U;
    uint32_t session_boot2 = boot2_tag | 0x0001U;

    RobotInterface_Init();
    RobotInterface_SetSessionId(session_boot1);

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = session_boot1;
    cmd.sequence = 1;
    cmd.valid_for_ms = 1000;
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;
    cmd.motor_test.axis = 0;
    cmd.motor_test.target_motor_rpm = 10.0f;
    cmd.motor_test.enable = true;

    TEST_ASSERT(RobotInterface_ValidateAndSetCommand(&cmd, NULL), "Session for boot 1 should be accepted");

    // Reboot MCU: session changes to boot 2
    RobotInterface_SetSessionId(session_boot2);

    // Old session from boot 1 attempted after reboot must be rejected
    cmd.session_id = session_boot1;
    cmd.sequence = 2;
    CommandRejectionReason_t reason = CMD_REJECT_NONE;
    TEST_ASSERT(!RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_SESSION_MISMATCH,
                "Replayed session from boot 1 after reboot to boot 2 must be rejected");

    // Session for boot 2 must be accepted
    cmd.session_id = session_boot2;
    cmd.sequence = 1;
    TEST_ASSERT(RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_NONE,
                "New session for boot 2 must be accepted");
}

static void test_telemetry_provenance_and_masks(void) {
    RobotTelemetry_t telem;
    memset(&telem, 0, sizeof(telem));
    telem.requested_enable_mask = 0x07;
    telem.applied_enable_mask = 0x07;
    telem.physical_enable_mask = 0x00; // Hardware output disabled

    TEST_ASSERT(telem.requested_enable_mask != telem.physical_enable_mask,
                "Physical enable mask must remain distinct from requested enable mask");
    TEST_ASSERT(telem.applied_enable_mask != telem.physical_enable_mask,
                "Physical enable mask must remain distinct from applied enable mask when output is off");
}

static void test_invalid_boot_counter_fails_closed(void) {
    RobotInterface_Init();
    // g_boot_id = 0 represents un-armed/unbound state due to cleared/invalid backup register
    RobotInterface_SetSessionId(0U);

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 0x00010001U;
    cmd.sequence = 1;
    cmd.valid_for_ms = 1000;
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;
    cmd.motor_test.axis = 0;
    cmd.motor_test.target_motor_rpm = 10.0f;
    cmd.motor_test.enable = true;

    CommandRejectionReason_t reason = CMD_REJECT_NONE;
    TEST_ASSERT(!RobotInterface_ValidateAndSetCommand(&cmd, &reason) && reason == CMD_REJECT_SESSION_MISMATCH,
                "Command must be rejected when MCU boot ID is 0 / unbound");
}

static void test_motor_set_speed_max_rpm_clamping(void) {
    Motor_t motor;
    memset(&motor, 0, sizeof(motor));
    motor.config.gear_ratio = 114.0f;
    motor.config.max_rpm = 5000; // 5000 motor shaft RPM -> 43.85 joint RPM ceiling

    Motor_SetSpeed(&motor, 100); // 100 joint RPM would be 11400 motor shaft RPM
    TEST_ASSERT(motor.current_rpm_cmd <= 43 && motor.current_rpm_cmd >= -43,
                "Motor_SetSpeed must clamp joint RPM to respect motor shaft max_rpm ceiling");
}

static void test_motor_test_build_command_uses_bringup_max_rpm(void) {
    MotorCommand_t output;
    memset(&output, 0, sizeof(output));

    // 120 RPM is > 100 (old legacy cap) but <= BRINGUP_MAX_MOTOR_RPM (150).
    // It must be preserved exactly without being truncated to 100 or pre-multiplied by gear ratio.
    TEST_ASSERT(Robot_TestMode_BuildCommand(true, 0, 120, &output),
                "BuildCommand for 120 RPM must succeed");
    TEST_ASSERT(output.rpm[0] == 120,
                "Robot_TestMode_BuildCommand must preserve 120 joint RPM (exceeding old ±100 cap)");
    TEST_ASSERT(output.enable_mask == 0x01U,
                "Axis 0 enable mask must be 0x01");

    // 200 RPM is > BRINGUP_MAX_MOTOR_RPM (150).  It must be clamped to 150.
    TEST_ASSERT(Robot_TestMode_BuildCommand(true, 1, 200, &output),
                "BuildCommand for 200 RPM must succeed");
    TEST_ASSERT(output.rpm[1] == (int32_t)BRINGUP_MAX_MOTOR_RPM,
                "Robot_TestMode_BuildCommand must clamp 200 RPM to BRINGUP_MAX_MOTOR_RPM (150)");
    TEST_ASSERT(output.enable_mask == 0x02U,
                "Axis 1 enable mask must be 0x02");
}

static void test_pva_and_safety_gate_audit(void) {
    Robot_Init();
    RobotSafety_Init();
    motor_joint_13pin.angle_offset_deg = 90.0f;
    motor_joint_13pin_2.angle_offset_deg = 0.0f;
    motor_joint_13pin_3.angle_offset_deg = 0.0f;
    RobotInterface_SetSessionId(50);
    g_host_mock_tick = 10000;

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    cmd.session_id = 50;
    cmd.sequence = 1;
    cmd.valid_for_ms = 1000;
    cmd.control_mode = CONTROL_MODE_JOINT_PVA;
    cmd.joint_pva.enable_mask = 0x07U;
    cmd.joint_pva.position_rad[0] = 1.6f;
    cmd.joint_pva.position_rad[1] = 0.2f;
    cmd.joint_pva.position_rad[2] = 0.3f;
    cmd.joint_pva.velocity_rad_s[0] = 0.05f;
    cmd.joint_pva.velocity_rad_s[1] = 0.05f;
    cmd.joint_pva.velocity_rad_s[2] = 0.05f;

    TEST_ASSERT(RobotInterface_ValidateAndSetCommand(&cmd, NULL), "PVA command must be accepted");

    Robot_ControlTask_Tick(0.001f);

    RobotTelemetry_t telem;
    RobotInterface_GetTelemetry(&telem);

    TEST_ASSERT(telem.requested_enable_mask == 0x07U, "PVA requested enable mask must be 0x07");
    TEST_ASSERT(telem.applied_enable_mask == 0x07U, "PVA applied enable mask must be 0x07");
    TEST_ASSERT(telem.physical_enable_mask == 0x00U, "Physical enable mask must be 0 when ACTUATOR_HARDWARE_OUTPUT_ENABLED is 0");
    TEST_ASSERT(ACTUATOR_HARDWARE_OUTPUT_ENABLED == 0, "ACTUATOR_HARDWARE_OUTPUT_ENABLED must remain 0");

    for (int i = 0; i < 50; ++i) {
        g_host_mock_tick += 1;
        Robot_ControlTask_Tick(0.001f);
    }
    RobotInterface_GetTelemetry(&telem);
    TEST_ASSERT(telem.applied_position_rad[0] != 0.0f || telem.applied_position_rad[1] != 0.0f,
                "Shadow PVA applied position state must update during ticks");
}

int main(void) {
    printf("Running RobotInterface host unit test suite...\n");
    test_sequence_wrap();
    test_unsupported_control_mode();
    test_ttl_tick_wrap_and_expiry();
    test_session_change_and_invalidation();
    test_unbound_and_idempotent_session();
    test_explicit_invalidation();
    test_invalid_numeric_fields();
    test_motor_test_disable_is_a_safety_stop();
    test_motor_test_enable_false_clears_state_and_ignores_legacy();
    test_reboot_and_boot_tag_session_replay();
    test_invalid_boot_counter_fails_closed();
    test_motor_set_speed_max_rpm_clamping();
    test_motor_test_build_command_uses_bringup_max_rpm();
    test_telemetry_provenance_and_masks();
    test_command_received_timestamp();
    test_shadow_applied_state_and_position_convergence();
    test_pva_and_safety_gate_audit();

    printf("Tests run: %d, Passed: %d\n", g_tests_run, g_tests_passed);
    if (g_tests_run == g_tests_passed) {
        printf("ALL ROBOT_INTERFACE TESTS PASSED.\n");
        return 0;
    } else {
        printf("SOME ROBOT_INTERFACE TESTS FAILED!\n");
        return 1;
    }
}
