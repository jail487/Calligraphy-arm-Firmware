/**
 * @file microros_app.c
 * @brief micro-ROS State Machine & Transport Implementation for STM32F446RE (ROS 2 Humble)
 * @note Custom doghead_interfaces (McuState, McuDiagnostics, etc.) are gated behind
 *       MICROROS_ENABLE_DOGHEAD_INTERFACES=1. Uses bounded RXNE polling in CommTask context
 *       for transport receive fallback while preserving DMA TX.
 */

#include "microros_app.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#if MICROROS_ENABLE_TOPIC_TELEMETRY
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#endif
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#if MICROROS_ENABLE_TOPIC_TELEMETRY
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/u_int32.h>
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "joint_units.h"
#include "motor_parameters.h"
#include "safety_manager.h"
#include "test_scenarios.h"
#include "mainpp.h"
#include "robot_config.h"
#include "runtime_monitor.h"
#include "freertos_runtime.h"

// 引用 dma_transport.c 宣告的函式、變數與 UART 句柄
typedef struct {
    volatile uint32_t tx_bytes;
    volatile uint32_t tx_frames;
    volatile uint32_t tx_errors;
    volatile uint32_t rx_bytes;
    volatile uint32_t rx_line_errors;
    volatile uint32_t rx_overflow_errors;
    volatile uint32_t open_failures;
} DmaTransportStats_t;

volatile uint8_t g_microros_entity_stage = MICROROS_ENTITY_STAGE_NONE;
volatile uint8_t g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_NONE;
volatile uint8_t g_microros_last_disconnect_reason = MICROROS_DISCONNECT_NONE;
volatile int32_t g_microros_last_error_rc = 0;

extern volatile DmaTransportStats_t g_dma_transport_stats;
extern bool cubemx_transport_open(struct uxrCustomTransport * transport);
extern bool cubemx_transport_close(struct uxrCustomTransport * transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

extern UART_HandleTypeDef huart2;

typedef enum {
    MICROROS_STATE_WAIT_AGENT,
    MICROROS_STATE_CREATE_ENTITIES,
    MICROROS_STATE_RUNNING,
    MICROROS_STATE_DESTROY_ENTITIES
} MicroROSState_t;

static MicroROSState_t g_state = MICROROS_STATE_WAIT_AGENT;

#if MICROROS_ENABLE_TOPIC_TELEMETRY
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static bool support_initialized = false;
static bool node_initialized = false;
#endif
static bool agent_disconnected = false;
static bool cleanup_failed = false;
static TickType_t last_agent_check = 0U;
static TickType_t last_heartbeat_pub = 0U;
static uint32_t g_mcu_state_sequence = 0U;
static uint32_t g_agent_reconnect_total = 0U;
static uint32_t g_boot_id = 0U;
static uint32_t g_last_bound_session_id = 0U;

static void MicroROS_InitBootId(void)
{
    if (g_boot_id != 0U) {
        return;
    }

    // RTC backup register contents survive MCU reset while VBAT is present.
    // Incrementing the persisted counter gives the host a per-boot epoch and
    // avoids treating a reboot with a reset state_sequence as a continuation.
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    uint32_t previous = RTC->BKP0R;
    if (previous == 0U || previous >= 0xFFFFFFFEU) {
        g_boot_id = 1U;
    } else {
        g_boot_id = previous + 1U;
    }
    RTC->BKP0R = g_boot_id;
}

#if MICROROS_ENABLE_TOPIC_TELEMETRY
static rcl_publisher_t pub_heartbeat;
static rcl_publisher_t pub_joint_states;
static bool publisher_initialized = false;
static bool joint_pub_initialized = false;

static std_msgs__msg__UInt32 msg_heartbeat;
static uint32_t g_heartbeat_counter = 0;

static sensor_msgs__msg__JointState msg_joint_states;
static double joint_positions[3] = {0.0, 0.0, 0.0};
static double joint_velocities[3] = {0.0, 0.0, 0.0};
static rosidl_runtime_c__String joint_names[3];
#endif

#if MICROROS_HAS_DOGHEAD_INTERFACES
static rcl_publisher_t pub_mcu_state;
static rcl_publisher_t pub_mcu_diagnostics;
static rcl_subscription_t sub_joint_setpoint;
static rcl_subscription_t sub_motor_test;
static rcl_subscription_t sub_session_control;
static rclc_executor_t executor;

static doghead_interfaces__msg__McuState msg_mcu_state;
static doghead_interfaces__msg__McuDiagnostics msg_mcu_diagnostics;
static doghead_interfaces__msg__McuJointSetpoint msg_joint_setpoint_sub;
static doghead_interfaces__msg__McuMotorTestCommand msg_motor_test_sub;
static doghead_interfaces__msg__McuSessionControl msg_session_control_sub;

static bool mcu_state_pub_initialized = false;
static bool mcu_diag_pub_initialized = false;
static bool joint_setpoint_sub_initialized = false;
static bool motor_test_sub_initialized = false;
static bool session_control_sub_initialized = false;
static bool executor_initialized = false;
#endif

#define MICROROS_AGENT_PING_PERIOD_MS 1000U
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if MICROROS_HAS_DOGHEAD_INTERFACES
static void sub_session_control_callback(const void * msgin) {
    const doghead_interfaces__msg__McuSessionControl * msg =
        (const doghead_interfaces__msg__McuSessionControl *)msgin;
    if (msg == NULL) return;

    if (!msg->armed) {
        RobotInterface_SetSessionId(0U);
        RobotInterface_InvalidateCommand();
        return;
    }

    if (g_boot_id == 0U) {
        RobotInterface_RecordRejection(CMD_REJECT_INVALID_PROTOCOL);
        return;
    }

    const uint32_t expected_boot_tag =
        (g_boot_id & 0xFFFFU) << DOGHEAD_SESSION_BOOT_TAG_SHIFT;
    const uint32_t session_epoch = msg->session_id & DOGHEAD_SESSION_EPOCH_MASK;
    if (msg->protocol_version != DOGHEAD_PROTOCOL_VERSION ||
        msg->session_id == 0U || session_epoch == 0U ||
        (msg->session_id & DOGHEAD_SESSION_BOOT_TAG_MASK) != expected_boot_tag) {
        RobotInterface_RecordRejection(CMD_REJECT_INVALID_PROTOCOL);
        return;
    }

    if (g_last_bound_session_id != 0U && msg->session_id != g_last_bound_session_id) {
        const uint32_t previous_epoch =
            g_last_bound_session_id & DOGHEAD_SESSION_EPOCH_MASK;
        const uint32_t epoch_delta = (session_epoch - previous_epoch) & DOGHEAD_SESSION_EPOCH_MASK;
        if (epoch_delta == 0U || epoch_delta >= 0x8000U) {
            RobotInterface_RecordRejection(CMD_REJECT_SESSION_MISMATCH);
            return;
        }
    }

    // A valid, newer boot-bound Arm epoch is the only way to bind commands
    // after transport reconnect or MCU reboot.
    g_last_bound_session_id = msg->session_id;
    RobotInterface_SetSessionId(msg->session_id);
    RobotSafety_ClearFault();
}

static void sub_joint_setpoint_callback(const void * msgin) {
    const doghead_interfaces__msg__McuJointSetpoint * msg =
        (const doghead_interfaces__msg__McuJointSetpoint *)msgin;
    if (msg == NULL) return;

    if (RobotInterface_GetSessionId() == 0U && msg->session_id != 0U) {
        const uint32_t expected_boot_tag = (g_boot_id & 0xFFFFU) << DOGHEAD_SESSION_BOOT_TAG_SHIFT;
        if ((msg->session_id & DOGHEAD_SESSION_BOOT_TAG_MASK) == expected_boot_tag) {
            RobotInterface_SetSessionId(msg->session_id);
        }
    }

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = msg->protocol_version;
    cmd.session_id = msg->session_id;
    cmd.sequence = msg->sequence;
    cmd.valid_for_ms = msg->valid_for_ms;
    cmd.control_mode = CONTROL_MODE_JOINT_PVA;

    cmd.joint_pva.enable_mask = msg->enable_mask;
    memcpy(cmd.joint_pva.position_rad, msg->position_rad, sizeof(msg->position_rad));
    memcpy(cmd.joint_pva.velocity_rad_s, msg->velocity_rad_s, sizeof(msg->velocity_rad_s));
    memcpy(cmd.joint_pva.acceleration_rad_s2, msg->acceleration_rad_s2, sizeof(msg->acceleration_rad_s2));

    CommandRejectionReason_t reason;
    (void)RobotInterface_ValidateAndSetCommand(&cmd, &reason);
}

static void sub_motor_test_callback(const void * msgin) {
    const doghead_interfaces__msg__McuMotorTestCommand * msg =
        (const doghead_interfaces__msg__McuMotorTestCommand *)msgin;
    if (msg == NULL) return;

    // A disable is an out-of-band safety stop.  It must remain effective even
    // if the sender is replaying an old sequence or the MCU has lost the
    // active session.  No legacy test_mode state may survive this event.
    if (!msg->enable) {
        Robot_ControlStateAbort();
        return;
    }

    // 遠端全自動硬體歸零觸發通道 (CMD_AXIS_SYSTEM_HOMING = 99)
    if (msg->axis == CMD_AXIS_SYSTEM_HOMING) {
        (void)RobotSafety_RequestHoming();
        return;
    }

    // 線上動態 PID 與前饋調參通道:
    // • axis 10~13: 設定 (Kp, Ki, Kd) (CMD_AXIS_PID_*)
    // • axis 20~23: 設定 (Kv, Ka)     (CMD_AXIS_FEEDFORWARD_*)
    if (msg->axis >= CMD_AXIS_PID_BASE && msg->axis < CMD_AXIS_FEEDFORWARD_BASE) {
        Robot_SetPidGains(msg->axis, msg->target_motor_rpm, msg->requested_accel_rpm_s, msg->requested_jerk_rpm_s2, -1.0f, -1.0f);
        return;
    } else if (msg->axis >= CMD_AXIS_FEEDFORWARD_BASE && msg->axis < CMD_AXIS_SYSTEM_HOMING) {
        Robot_SetPidGains(msg->axis - CMD_AXIS_PID_BASE, -1.0f, -1.0f, -1.0f, msg->target_motor_rpm, msg->requested_accel_rpm_s);
        return;
    }

    if (RobotInterface_GetSessionId() == 0U && msg->session_id != 0U) {
        const uint32_t expected_boot_tag = (g_boot_id & 0xFFFFU) << DOGHEAD_SESSION_BOOT_TAG_SHIFT;
        if ((msg->session_id & DOGHEAD_SESSION_BOOT_TAG_MASK) == expected_boot_tag) {
            RobotInterface_SetSessionId(msg->session_id);
        }
    }

    RobotCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.protocol_version = msg->protocol_version;
    cmd.session_id = msg->session_id;
    cmd.sequence = msg->sequence;
    cmd.valid_for_ms = msg->valid_for_ms;
    cmd.control_mode = CONTROL_MODE_MOTOR_VELOCITY_TEST;

    cmd.motor_test.axis = msg->axis;
    cmd.motor_test.target_motor_rpm = msg->target_motor_rpm;
    cmd.motor_test.requested_accel_rpm_s = msg->requested_accel_rpm_s;
    cmd.motor_test.requested_jerk_rpm_s2 = msg->requested_jerk_rpm_s2;
    cmd.motor_test.enable = true;

    cmd.test_mode = 0;
    cmd.test_axis = 0U;
    cmd.test_rpm = 0;

    CommandRejectionReason_t reason;
    (void)RobotInterface_ValidateAndSetCommand(&cmd, &reason);
}
#endif

static void MicroROS_ResetEntityHandles(void)
{
#if MICROROS_ENABLE_TOPIC_TELEMETRY
    memset(&support, 0, sizeof(support));
    node = rcl_get_zero_initialized_node();
    pub_heartbeat = rcl_get_zero_initialized_publisher();
    pub_joint_states = rcl_get_zero_initialized_publisher();
    publisher_initialized = false;
    joint_pub_initialized = false;
#if MICROROS_HAS_DOGHEAD_INTERFACES
    pub_mcu_state = rcl_get_zero_initialized_publisher();
    pub_mcu_diagnostics = rcl_get_zero_initialized_publisher();
    sub_joint_setpoint = rcl_get_zero_initialized_subscription();
    sub_motor_test = rcl_get_zero_initialized_subscription();
    sub_session_control = rcl_get_zero_initialized_subscription();
    mcu_state_pub_initialized = false;
    mcu_diag_pub_initialized = false;
    joint_setpoint_sub_initialized = false;
    motor_test_sub_initialized = false;
    session_control_sub_initialized = false;
    executor_initialized = false;
#endif
#endif
}

static void MicroROS_DestroyEntities(void)
{
    cleanup_failed = false;
    RobotInterface_SetSessionId(0U);
    RobotInterface_InvalidateCommand(); // 無條件清空舊指令與佇列

#if MICROROS_ENABLE_TOPIC_TELEMETRY
    if (support_initialized && agent_disconnected) {
        g_agent_reconnect_total++;
        rmw_context_t * rmw_context =
            rcl_context_get_rmw_context(&support.context);
        if (rmw_context != NULL) {
            (void)rmw_uros_set_context_entity_destroy_session_timeout(
                rmw_context, 0);
        }
    }

#if MICROROS_HAS_DOGHEAD_INTERFACES
    if (executor_initialized) {
        (void)rclc_executor_fini(&executor);
        executor_initialized = false;
    }
    if (motor_test_sub_initialized) {
        if (rcl_subscription_fini(&sub_motor_test, &node) != RCL_RET_OK) {
            cleanup_failed = true;
        }
        motor_test_sub_initialized = false;
    }
    if (joint_setpoint_sub_initialized) {
        if (rcl_subscription_fini(&sub_joint_setpoint, &node) != RCL_RET_OK) {
            cleanup_failed = true;
        }
        joint_setpoint_sub_initialized = false;
    }
    if (session_control_sub_initialized) {
        if (rcl_subscription_fini(&sub_session_control, &node) != RCL_RET_OK) {
            cleanup_failed = true;
        }
        session_control_sub_initialized = false;
    }
    if (mcu_diag_pub_initialized) {
        if (rcl_publisher_fini(&pub_mcu_diagnostics, &node) != RCL_RET_OK) {
            cleanup_failed = true;
        }
        mcu_diag_pub_initialized = false;
    }
    if (mcu_state_pub_initialized) {
        if (rcl_publisher_fini(&pub_mcu_state, &node) != RCL_RET_OK) {
            cleanup_failed = true;
        }
        mcu_state_pub_initialized = false;
    }
#endif

    if (joint_pub_initialized) {
        if (rcl_publisher_fini(&pub_joint_states, &node) != RCL_RET_OK) {
            cleanup_failed = true;
        }
        joint_pub_initialized = false;
    }

    if (publisher_initialized) {
        if (rcl_publisher_fini(&pub_heartbeat, &node) != RCL_RET_OK) {
            cleanup_failed = true;
        }
        publisher_initialized = false;
    }

    if (node_initialized) {
        if (rcl_node_fini(&node) != RCL_RET_OK) {
            cleanup_failed = true;
        }
        node_initialized = false;
    }
    if (support_initialized) {
        if (rclc_support_fini(&support) != RCL_RET_OK) {
            cleanup_failed = true;
        }
        support_initialized = false;
    }
#else
    if (agent_disconnected) {
        g_agent_reconnect_total++;
    }
#endif

    agent_disconnected = false;
    MicroROS_ResetEntityHandles();
}

bool MicroROS_Init(void)
{
    if (g_state != MICROROS_STATE_WAIT_AGENT) {
        return false;
    }

    // 綁定自訂 DMA Transport 介面
    rmw_uros_set_custom_transport(
        true,
        (void *) &huart2,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read
    );

    g_state = MICROROS_STATE_WAIT_AGENT;
    agent_disconnected = false;
    g_microros_entity_stage = MICROROS_ENTITY_STAGE_NONE;
    g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_NONE;
    g_microros_last_disconnect_reason = MICROROS_DISCONNECT_NONE;
    g_microros_last_error_rc = 0;
    g_mcu_state_sequence = 0U;
    MicroROS_InitBootId();
    // A freshly initialized transport is deliberately unbound.  Only a valid,
    // explicit SessionControl arm message may establish a command epoch.
    RobotInterface_SetSessionId(0U);
    RobotInterface_InvalidateCommand();
    MicroROS_ResetEntityHandles();
    return true;
}

void MicroROS_Spin(void)
{
    switch (g_state) {
        case MICROROS_STATE_WAIT_AGENT:
            // 嘗試 Ping Agent (等 100ms)
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                g_state = MICROROS_STATE_CREATE_ENTITIES;
            }
            break;

        case MICROROS_STATE_CREATE_ENTITIES:
            cleanup_failed = false;
            MicroROS_ResetEntityHandles();
            g_microros_entity_stage = MICROROS_ENTITY_STAGE_NONE;

#if MICROROS_ENABLE_TOPIC_TELEMETRY
            rcl_ret_t rc = RCL_RET_OK;
            allocator = rcl_get_default_allocator();

            // 1. 初始化 Support
            g_microros_entity_stage = MICROROS_ENTITY_STAGE_SUPPORT_INIT;
            rc = rclc_support_init(&support, 0, NULL, &allocator);
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_SUPPORT_INIT;
                g_microros_last_error_rc = (int32_t)rc;
                if (support.context.impl != NULL) {
                    (void)rclc_support_fini(&support);
                }
                RobotInterface_SetSessionId(0U);
                RobotInterface_InvalidateCommand();
                MicroROS_ResetEntityHandles();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
            support_initialized = true;

            // 2. 建立 Node
            g_microros_entity_stage = MICROROS_ENTITY_STAGE_NODE_INIT;
            rc = rclc_node_init_default(&node, "doghead_arm_node", "", &support);
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_NODE_INIT;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
            node_initialized = true;

            // 3. 建立 Heartbeat Publisher (/doghead/heartbeat)
            g_microros_entity_stage = MICROROS_ENTITY_STAGE_HEARTBEAT_PUB_INIT;
            rc = rclc_publisher_init_default(
                    &pub_heartbeat,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
                    "/doghead/heartbeat");
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_HEARTBEAT_PUB_INIT;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
            publisher_initialized = true;

            // 4. 建立 JointState Publisher (/joint_states) - Best Effort QoS
            g_microros_entity_stage = MICROROS_ENTITY_STAGE_JOINT_STATES_PUB_INIT;
            rc = rclc_publisher_init_best_effort(
                    &pub_joint_states,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                    "/joint_states");
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_JOINT_STATES_PUB_INIT;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
            joint_pub_initialized = true;

            // 配置 JointState 動態陣列記憶體結構
            joint_names[0].data = (char *)"joint1";
            joint_names[0].size = 6;
            joint_names[0].capacity = 7;
            joint_names[1].data = (char *)"joint2";
            joint_names[1].size = 6;
            joint_names[1].capacity = 7;
            joint_names[2].data = (char *)"joint3";
            joint_names[2].size = 6;
            joint_names[2].capacity = 7;

            msg_joint_states.name.data = joint_names;
            msg_joint_states.name.size = 3;
            msg_joint_states.name.capacity = 3;

            msg_joint_states.position.data = joint_positions;
            msg_joint_states.position.size = 3;
            msg_joint_states.position.capacity = 3;

            msg_joint_states.velocity.data = joint_velocities;
            msg_joint_states.velocity.size = 3;
            msg_joint_states.velocity.capacity = 3;

#if MICROROS_HAS_DOGHEAD_INTERFACES
            // 5. 建立 McuState Publisher (/doghead/mcu/state) - Best Effort QoS
            g_microros_entity_stage = MICROROS_ENTITY_STAGE_MCU_STATE_PUB_INIT;
            rc = rclc_publisher_init_best_effort(
                    &pub_mcu_state,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(doghead_interfaces, msg, McuState),
                    "/doghead/mcu/state");
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_MCU_STATE_PUB_INIT;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
            mcu_state_pub_initialized = true;

            // 6. 建立 McuDiagnostics Publisher (/doghead/mcu/diagnostics)
            g_microros_entity_stage = MICROROS_ENTITY_STAGE_MCU_DIAG_PUB_INIT;
            rc = rclc_publisher_init_default(
                    &pub_mcu_diagnostics,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(doghead_interfaces, msg, McuDiagnostics),
                    "/doghead/mcu/diagnostics");
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_MCU_DIAG_PUB_INIT;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
            mcu_diag_pub_initialized = true;

            // 7. 建立 Command Subscriptions & Executor
            g_microros_entity_stage = MICROROS_ENTITY_STAGE_SESSION_CONTROL_SUB_INIT;
            rc = rclc_subscription_init_default(
                    &sub_session_control,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(doghead_interfaces, msg, McuSessionControl),
                    "/doghead/mcu/session_control");
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_SESSION_CONTROL_SUB_INIT;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
            session_control_sub_initialized = true;

            g_microros_entity_stage = MICROROS_ENTITY_STAGE_JOINT_SETPOINT_SUB_INIT;
            rc = rclc_subscription_init_default(
                    &sub_joint_setpoint,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(doghead_interfaces, msg, McuJointSetpoint),
                    "/doghead/mcu/joint_setpoint");
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_JOINT_SETPOINT_SUB_INIT;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
            joint_setpoint_sub_initialized = true;

            g_microros_entity_stage = MICROROS_ENTITY_STAGE_MOTOR_TEST_SUB_INIT;
            rc = rclc_subscription_init_default(
                    &sub_motor_test,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(doghead_interfaces, msg, McuMotorTestCommand),
                    "/doghead/mcu/motor_test_command");
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_MOTOR_TEST_SUB_INIT;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
            motor_test_sub_initialized = true;

            g_microros_entity_stage = MICROROS_ENTITY_STAGE_EXECUTOR_INIT;
            rc = rclc_executor_init(&executor, &support.context, 3, &allocator);
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_EXECUTOR_INIT;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
            executor_initialized = true;

            g_microros_entity_stage = MICROROS_ENTITY_STAGE_EXECUTOR_ADD_SESSION_CONTROL;
            rc = rclc_executor_add_subscription(
                    &executor,
                    &sub_session_control,
                    &msg_session_control_sub,
                    &sub_session_control_callback,
                    ON_NEW_DATA
            );
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_EXECUTOR_ADD_SESSION_CONTROL;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }

            g_microros_entity_stage = MICROROS_ENTITY_STAGE_EXECUTOR_ADD_JOINT_SETPOINT;
            rc = rclc_executor_add_subscription(
                    &executor,
                    &sub_joint_setpoint,
                    &msg_joint_setpoint_sub,
                    &sub_joint_setpoint_callback,
                    ON_NEW_DATA
            );
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_EXECUTOR_ADD_JOINT_SETPOINT;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }

            g_microros_entity_stage = MICROROS_ENTITY_STAGE_EXECUTOR_ADD_MOTOR_TEST;
            rc = rclc_executor_add_subscription(
                    &executor,
                    &sub_motor_test,
                    &msg_motor_test_sub,
                    &sub_motor_test_callback,
                    ON_NEW_DATA
            );
            if (rc != RCL_RET_OK) {
                g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_EXECUTOR_ADD_MOTOR_TEST;
                g_microros_last_error_rc = (int32_t)rc;
                MicroROS_DestroyEntities();
                g_state = MICROROS_STATE_WAIT_AGENT;
                break;
            }
#endif
            g_microros_entity_stage = MICROROS_ENTITY_STAGE_COMPLETE;
            g_microros_last_failed_stage = MICROROS_ENTITY_STAGE_NONE;
            g_microros_last_disconnect_reason = MICROROS_DISCONNECT_NONE;
            g_microros_last_error_rc = 0;
#endif

            last_agent_check = xTaskGetTickCount();
            last_heartbeat_pub = xTaskGetTickCount();
            g_state = MICROROS_STATE_RUNNING;
            break;

        case MICROROS_STATE_RUNNING:
            if ((TickType_t)(xTaskGetTickCount() - last_agent_check) >=
                pdMS_TO_TICKS(MICROROS_AGENT_PING_PERIOD_MS)) {
                last_agent_check = xTaskGetTickCount();
                rmw_ret_t ping_rc = rmw_uros_ping_agent(100, 1);
                if (ping_rc != RMW_RET_OK) {
                    g_microros_last_disconnect_reason = MICROROS_DISCONNECT_PING_FAIL;
                    g_microros_last_error_rc = (int32_t)ping_rc;
                    agent_disconnected = true;
                    g_state = MICROROS_STATE_DESTROY_ENTITIES;
                    break;
                }
            }

#if MICROROS_ENABLE_TOPIC_TELEMETRY
#if MICROROS_HAS_DOGHEAD_INTERFACES
            if (executor_initialized) {
                (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            }
#endif

            // 發布心跳訊號 (1 Hz 頻率, wrap-safe tick 算術)
            TickType_t now = xTaskGetTickCount();
            if ((TickType_t)(now - last_heartbeat_pub) >= pdMS_TO_TICKS(1000)) {
                last_heartbeat_pub = now;
                msg_heartbeat.data = g_heartbeat_counter++;
                rcl_ret_t rc = rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL);
                if (rc != RCL_RET_OK) {
                    g_microros_last_disconnect_reason = MICROROS_DISCONNECT_HEARTBEAT_PUB_FAIL;
                    g_microros_last_error_rc = (int32_t)rc;
                    agent_disconnected = true;
                    g_state = MICROROS_STATE_DESTROY_ENTITIES;
                }
            }
#endif
            break;

        case MICROROS_STATE_DESTROY_ENTITIES:
            MicroROS_DestroyEntities();
            g_state = MICROROS_STATE_WAIT_AGENT;
            break;

        default:
            MicroROS_DestroyEntities();
            g_state = MICROROS_STATE_WAIT_AGENT;
            break;
    }
}

void MicroROS_PublishJointStates(const RobotTelemetry_t * telem)
{
#if MICROROS_ENABLE_TOPIC_TELEMETRY
    if (g_state != MICROROS_STATE_RUNNING || !joint_pub_initialized || telem == NULL) {
        return;
    }

    float pos0 = Joint_Deg_To_Rad(telem->real_theta1);
    float pos1 = Joint_Deg_To_Rad(telem->real_theta2);
    float pos2 = Joint_Deg_To_Rad(telem->real_theta3);

    float vel0 = Joint_RPM_To_Rad_S(telem->real_rpm1);
    float vel1 = Joint_RPM_To_Rad_S(telem->real_rpm2);
    float vel2 = Joint_RPM_To_Rad_S(telem->real_rpm3);

    joint_positions[0] = isfinite(pos0) ? (double)pos0 : 0.0;
    joint_positions[1] = isfinite(pos1) ? (double)pos1 : 0.0;
    joint_positions[2] = isfinite(pos2) ? (double)pos2 : 0.0;

    joint_velocities[0] = isfinite(vel0) ? (double)vel0 : 0.0;
    joint_velocities[1] = isfinite(vel1) ? (double)vel1 : 0.0;
    joint_velocities[2] = isfinite(vel2) ? (double)vel2 : 0.0;

    rcl_ret_t rc = rcl_publish(&pub_joint_states, &msg_joint_states, NULL);
    if (rc != RCL_RET_OK) {
        g_microros_last_disconnect_reason = MICROROS_DISCONNECT_JOINT_STATES_PUB_FAIL;
        g_microros_last_error_rc = (int32_t)rc;
        agent_disconnected = true;
        g_state = MICROROS_STATE_DESTROY_ENTITIES;
    }
#else
    (void)telem;
#endif
}

void MicroROS_PublishMcuState(const RobotTelemetry_t * telem)
{
#if MICROROS_ENABLE_TOPIC_TELEMETRY && MICROROS_HAS_DOGHEAD_INTERFACES
    if (g_state != MICROROS_STATE_RUNNING || !mcu_state_pub_initialized || telem == NULL) {
        return;
    }

    RobotDiagnostics_t diag;
    RobotInterface_GetDiagnostics(&diag);
    const bool has_cmd = telem->has_command;

    msg_mcu_state.protocol_version = DOGHEAD_PROTOCOL_VERSION;
    msg_mcu_state.boot_id = g_boot_id;
    msg_mcu_state.state_sequence = g_mcu_state_sequence++;
    msg_mcu_state.uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    msg_mcu_state.control_mode = (uint8_t)(has_cmd ? telem->control_mode : CONTROL_MODE_DISABLED);
    msg_mcu_state.safety_state = (uint8_t)RobotSafety_GetState();
    msg_mcu_state.fault_code = (uint16_t)RobotSafety_GetFault();
    msg_mcu_state.requested_enable_mask = has_cmd ? telem->requested_enable_mask : 0x00;
    msg_mcu_state.enable_mask = has_cmd ? telem->applied_enable_mask : 0x00;
    msg_mcu_state.physical_enable_mask = telem->physical_enable_mask;
    msg_mcu_state.agent_connected = (g_state == MICROROS_STATE_RUNNING);

    if (has_cmd && telem->control_mode == CONTROL_MODE_JOINT_PVA) {
        for (int i = 0; i < 3; i++) {
            msg_mcu_state.target_position_rad[i] = telem->target_position_rad[i];
            msg_mcu_state.target_velocity_rad_s[i] = telem->target_velocity_rad_s[i];
            msg_mcu_state.target_acceleration_rad_s2[i] = telem->target_acceleration_rad_s2[i];
            msg_mcu_state.applied_position_rad[i] = telem->applied_position_rad[i];
            msg_mcu_state.applied_velocity_rad_s[i] = telem->applied_velocity_rad_s[i];
        }
    } else {
        memset(msg_mcu_state.target_position_rad, 0, sizeof(msg_mcu_state.target_position_rad));
        memset(msg_mcu_state.target_velocity_rad_s, 0, sizeof(msg_mcu_state.target_velocity_rad_s));
        memset(msg_mcu_state.target_acceleration_rad_s2, 0, sizeof(msg_mcu_state.target_acceleration_rad_s2));
        memset(msg_mcu_state.applied_position_rad, 0, sizeof(msg_mcu_state.applied_position_rad));
        memset(msg_mcu_state.applied_velocity_rad_s, 0, sizeof(msg_mcu_state.applied_velocity_rad_s));
    }

    float p0 = Joint_Deg_To_Rad(telem->real_theta1);
    float p1 = Joint_Deg_To_Rad(telem->real_theta2);
    float p2 = Joint_Deg_To_Rad(telem->real_theta3);
    msg_mcu_state.measured_position_rad[0] = isfinite(p0) ? p0 : 0.0f;
    msg_mcu_state.measured_position_rad[1] = isfinite(p1) ? p1 : 0.0f;
    msg_mcu_state.measured_position_rad[2] = isfinite(p2) ? p2 : 0.0f;

    float v0 = Joint_RPM_To_Rad_S(telem->real_rpm1);
    float v1 = Joint_RPM_To_Rad_S(telem->real_rpm2);
    float v2 = Joint_RPM_To_Rad_S(telem->real_rpm3);
    msg_mcu_state.measured_velocity_rad_s[0] = isfinite(v0) ? v0 : 0.0f;
    msg_mcu_state.measured_velocity_rad_s[1] = isfinite(v1) ? v1 : 0.0f;
    msg_mcu_state.measured_velocity_rad_s[2] = isfinite(v2) ? v2 : 0.0f;

    for (int i = 0; i < 3; i++) {
        msg_mcu_state.position_error_rad[i] = msg_mcu_state.applied_position_rad[i] - msg_mcu_state.measured_position_rad[i];
    }

    msg_mcu_state.commanded_motor_rpm[0] = isfinite(telem->commanded_motor_rpm[0]) ? telem->commanded_motor_rpm[0] : 0.0f;
    msg_mcu_state.commanded_motor_rpm[1] = isfinite(telem->commanded_motor_rpm[1]) ? telem->commanded_motor_rpm[1] : 0.0f;
    msg_mcu_state.commanded_motor_rpm[2] = isfinite(telem->commanded_motor_rpm[2]) ? telem->commanded_motor_rpm[2] : 0.0f;

    msg_mcu_state.encoder_count[0] = (int64_t)telem->motor1_count;
    msg_mcu_state.encoder_count[1] = (int64_t)telem->motor2_count;
    msg_mcu_state.encoder_count[2] = (int64_t)telem->motor3_count;

    // `commanded_motor_rpm[]` is joint/output-shaft RPM.  Convert with the
    // explicit joint-RPM -> motor-shaft-RPM -> command-Hz helper; measured RPM
    // remains independent telemetry.
    msg_mcu_state.pwm_frequency_hz[0] = (uint32_t)fabsf(
        Joint_RPM_To_Command_Freq_Hz(telem->commanded_motor_rpm[0],
                                     GEAR_RATIO_AXIS1, MOTOR_PULSES_PER_REV_AXIS1));
    msg_mcu_state.pwm_frequency_hz[1] = (uint32_t)fabsf(
        Joint_RPM_To_Command_Freq_Hz(telem->commanded_motor_rpm[1],
                                     GEAR_RATIO_AXIS2, MOTOR_PULSES_PER_REV_AXIS2));
    msg_mcu_state.pwm_frequency_hz[2] = (uint32_t)fabsf(
        Joint_RPM_To_Command_Freq_Hz(telem->commanded_motor_rpm[2],
                                     GEAR_RATIO_AXIS3, MOTOR_PULSES_PER_REV_AXIS3));

    const TickType_t now_tick = xTaskGetTickCount();
    const TickType_t command_age_ticks = has_cmd
        ? (TickType_t)(now_tick - telem->command_received_tick) : 0U;
    const uint32_t command_age_ms = (uint32_t)(command_age_ticks * portTICK_PERIOD_MS);
    msg_mcu_state.command_age_ms = command_age_ms > UINT16_MAX
        ? UINT16_MAX : (uint16_t)command_age_ms;
    msg_mcu_state.last_accepted_command_sequence = diag.last_accepted_sequence;
    msg_mcu_state.rejected_command_total = diag.rejected_command_total;

    rcl_ret_t rc = rcl_publish(&pub_mcu_state, &msg_mcu_state, NULL);
    if (rc != RCL_RET_OK) {
        g_microros_last_disconnect_reason = MICROROS_DISCONNECT_MCU_STATE_PUB_FAIL;
        g_microros_last_error_rc = (int32_t)rc;
        agent_disconnected = true;
        g_state = MICROROS_STATE_DESTROY_ENTITIES;
    }
#else
    (void)telem;
#endif
}

void MicroROS_PublishMcuDiagnostics(const RobotDiagnostics_t * diag)
{
#if MICROROS_ENABLE_TOPIC_TELEMETRY && MICROROS_HAS_DOGHEAD_INTERFACES
    if (g_state != MICROROS_STATE_RUNNING || !mcu_diag_pub_initialized || diag == NULL) {
        return;
    }

    msg_mcu_diagnostics.control_deadline_miss_total = diag->control_deadline_miss_total;
    msg_mcu_diagnostics.control_wcet_cycles = RuntimeMonitor_GetMaxCycles();
    msg_mcu_diagnostics.command_timeout_total = diag->command_timeout_total;
    msg_mcu_diagnostics.invalid_command_total = diag->invalid_command_total;
    msg_mcu_diagnostics.stale_sequence_total = diag->stale_sequence_total;
    msg_mcu_diagnostics.session_mismatch_total = diag->session_mismatch_total;
    msg_mcu_diagnostics.uart_ore_total = g_dma_transport_stats.rx_line_errors;
    msg_mcu_diagnostics.uart_ne_total = 0U;
    msg_mcu_diagnostics.uart_fe_total = 0U;
    msg_mcu_diagnostics.dma_rx_overflow_total = g_dma_transport_stats.rx_overflow_errors;
    msg_mcu_diagnostics.transport_tx_timeout_total = g_dma_transport_stats.tx_errors;
    msg_mcu_diagnostics.agent_reconnect_total = g_agent_reconnect_total;
    msg_mcu_diagnostics.free_heap_bytes = (uint32_t)xPortGetFreeHeapSize();
    msg_mcu_diagnostics.minimum_ever_free_heap_bytes = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    msg_mcu_diagnostics.control_stack_high_watermark = Robot_GetControlStackHighWatermark();
    msg_mcu_diagnostics.comm_stack_high_watermark = Robot_GetCommStackHighWatermark();

    rcl_ret_t rc = rcl_publish(&pub_mcu_diagnostics, &msg_mcu_diagnostics, NULL);
    if (rc != RCL_RET_OK) {
        g_microros_last_disconnect_reason = MICROROS_DISCONNECT_MCU_DIAG_PUB_FAIL;
        g_microros_last_error_rc = (int32_t)rc;
        agent_disconnected = true;
        g_state = MICROROS_STATE_DESTROY_ENTITIES;
    }
#else
    (void)diag;
#endif
}

void MicroROS_Fini(void)
{
#if MICROROS_ENABLE_TOPIC_TELEMETRY
    if (!support_initialized && !node_initialized) {
        g_state = MICROROS_STATE_WAIT_AGENT;
        return;
    }
#endif

    g_state = MICROROS_STATE_DESTROY_ENTITIES;
    MicroROS_Spin();
}

bool MicroROS_IsInitialized(void)
{
    return (g_state == MICROROS_STATE_RUNNING) && !cleanup_failed;
}
