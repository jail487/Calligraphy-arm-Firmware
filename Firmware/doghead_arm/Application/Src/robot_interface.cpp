#include "robot_interface.h"
#include "robot_config.h"
#include "trajectory_planner.h"
#include <cmath>
#include <cstring>


typedef struct {
    RobotCommand_t command;
    uint32_t received_at;
} RobotCommandEnvelope_t;

#ifdef HOST_TEST
typedef uint32_t TickType_t;
typedef void* QueueHandle_t;
typedef uint32_t StaticQueue_t;
typedef uint32_t BaseType_t;

#define pdFALSE 0
#define pdTRUE 1
#define pdPASS 1
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()

uint32_t g_host_mock_tick = 0;
inline uint32_t xTaskGetTickCount(void) { return g_host_mock_tick; }
inline uint32_t xTaskGetTickCountFromISR(void) { return g_host_mock_tick; }
inline uint32_t __get_IPSR(void) { return 0; }
inline void portYIELD_FROM_ISR(BaseType_t x) { (void)x; }

static RobotCommandEnvelope_t g_host_queue_buf;
static bool g_host_queue_has_data = false;

inline QueueHandle_t xQueueCreateStatic(uint32_t len, uint32_t sz, uint8_t* storage, StaticQueue_t* control) {
    (void)len; (void)sz; (void)storage; (void)control;
    return (QueueHandle_t)1;
}

inline BaseType_t xQueueOverwrite(QueueHandle_t q, const void* item) {
    (void)q;
    memcpy(&g_host_queue_buf, item, sizeof(RobotCommandEnvelope_t));
    g_host_queue_has_data = true;
    return pdPASS;
}

inline BaseType_t xQueueOverwriteFromISR(QueueHandle_t q, const void* item, BaseType_t* px) {
    (void)px;
    return xQueueOverwrite(q, item);
}

inline BaseType_t xQueueReceive(QueueHandle_t q, void* item, uint32_t ticks) {
    (void)q; (void)ticks;
    if (!g_host_queue_has_data) return 0;
    memcpy(item, &g_host_queue_buf, sizeof(RobotCommandEnvelope_t));
    g_host_queue_has_data = false;
    return pdPASS;
}

inline void xQueueReset(QueueHandle_t q) {
    (void)q;
    g_host_queue_has_data = false;
    memset(&g_host_queue_buf, 0, sizeof(g_host_queue_buf));
}
#else
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h" // For HAL and CMSIS intrinsics like __get_IPSR()
#endif

// ==========================================================
// Static Queue 與 Snapshot
// ==========================================================
static StaticQueue_t command_queue_control;
static uint8_t command_queue_storage[sizeof(RobotCommandEnvelope_t)];
static QueueHandle_t command_queue = NULL;

static RobotCommandEnvelope_t g_current_envelope; // 緩存最新有效指令
static bool g_have_current_command = false;

static RobotTelemetry_t g_current_telemetry;
static RobotDiagnostics_t g_diagnostics;

static uint32_t g_session_id = 0;
static uint32_t g_last_accepted_sequence = 0;
static bool g_has_accepted_sequence = false;

extern "C" void RobotInterface_Init(void) {
    command_queue = xQueueCreateStatic(1, sizeof(RobotCommandEnvelope_t), command_queue_storage, &command_queue_control);
    memset(&g_current_envelope, 0, sizeof(RobotCommandEnvelope_t));
    memset(&g_current_telemetry, 0, sizeof(RobotTelemetry_t));
    memset(&g_diagnostics, 0, sizeof(RobotDiagnostics_t));
    
    g_current_telemetry.system_state = ROBOT_SYS_STATE_INIT;
    g_have_current_command = false;
    g_session_id = 0;
    g_last_accepted_sequence = 0;
    g_has_accepted_sequence = false;
}

extern "C" bool RobotInterface_ValidateCommand(
    const RobotCommand_t* cmd,
    uint32_t active_session,
    uint32_t last_seq,
    bool has_last_seq,
    CommandRejectionReason_t* out_reason
) {
    CommandRejectionReason_t dummy_reason = CMD_REJECT_NONE;
    CommandRejectionReason_t* reason = out_reason ? out_reason : &dummy_reason;
    *reason = CMD_REJECT_NONE;

    if (!cmd) {
        *reason = CMD_REJECT_NON_FINITE_PARAM;
        return false;
    }

    // 1. Protocol version check
    if (cmd->protocol_version != DOGHEAD_PROTOCOL_VERSION) {
        *reason = CMD_REJECT_INVALID_PROTOCOL;
        return false;
    }

    // 2. Control mode validity check (reject unsupported/unknown enum values)
    if (cmd->control_mode < CONTROL_MODE_DISABLED || cmd->control_mode > CONTROL_MODE_STOP) {
        *reason = CMD_REJECT_UNSUPPORTED_MODE;
        return false;
    }

    // A STOP is safe to accept even when reconnect/disarm already cleared the
    // active session.  All motion modes remain fail-closed until an explicit
    // non-zero Arm session is bound.
    const bool session_independent_stop = (cmd->control_mode == CONTROL_MODE_STOP);
    if (!session_independent_stop && (active_session == 0 || cmd->session_id != active_session)) {
        *reason = CMD_REJECT_SESSION_MISMATCH;
        return false;
    }

    // 4. Sequence check (wrap-safe uint32 comparison using signed difference)
    if (has_last_seq) {
        int32_t diff = (int32_t)(cmd->sequence - last_seq);
        if (diff <= 0) {
            *reason = CMD_REJECT_STALE_SEQUENCE;
            return false;
        }
    }

    // 5. TTL check (must be > 0 and <= 10000ms)
    if (cmd->valid_for_ms == 0 || cmd->valid_for_ms > 10000U) {
        *reason = CMD_REJECT_EXPIRED_TTL;
        return false;
    }

    // 6. Mode-specific & field validity checks
    if (cmd->control_mode == CONTROL_MODE_MOTOR_VELOCITY_TEST) {
        if (cmd->motor_test.axis >= 3U) {
            *reason = CMD_REJECT_OUT_OF_BOUNDS_AXIS_OR_MASK;
            return false;
        }
        if (!std::isfinite(cmd->motor_test.target_motor_rpm) ||
            !std::isfinite(cmd->motor_test.requested_accel_rpm_s) ||
            !std::isfinite(cmd->motor_test.requested_jerk_rpm_s2)) {
            *reason = CMD_REJECT_NON_FINITE_PARAM;
            return false;
        }
    } else if (cmd->control_mode == CONTROL_MODE_JOINT_PVA) {
        if (cmd->joint_pva.enable_mask > 0x07U) {
            *reason = CMD_REJECT_OUT_OF_BOUNDS_AXIS_OR_MASK;
            return false;
        }
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(cmd->joint_pva.position_rad[i]) ||
                !std::isfinite(cmd->joint_pva.velocity_rad_s[i]) ||
                !std::isfinite(cmd->joint_pva.acceleration_rad_s2[i])) {
                *reason = CMD_REJECT_NON_FINITE_PARAM;
                return false;
            }
            if (std::fabs(cmd->joint_pva.velocity_rad_s[i]) > JOINT_PVA_MAX_VEL_RAD_S ||
                std::fabs(cmd->joint_pva.acceleration_rad_s2[i]) > JOINT_PVA_MAX_ACCEL_RAD_S2) {
                *reason = CMD_REJECT_NON_FINITE_PARAM;
                return false;
            }
        }
        const float JOINT_EPS_RAD = 0.005f; // ~0.28 deg tolerance for float precision / micro-switch reset pose
        if (cmd->joint_pva.position_rad[0] < (JOINT1_MIN_RAD - JOINT_EPS_RAD) || cmd->joint_pva.position_rad[0] > (JOINT1_MAX_RAD + JOINT_EPS_RAD) ||
            cmd->joint_pva.position_rad[1] < (JOINT2_MIN_RAD - JOINT_EPS_RAD) || cmd->joint_pva.position_rad[1] > (JOINT2_MAX_RAD + JOINT_EPS_RAD) ||
            cmd->joint_pva.position_rad[2] < (JOINT3_MIN_RAD - JOINT_EPS_RAD) || cmd->joint_pva.position_rad[2] > (JOINT3_MAX_RAD + JOINT_EPS_RAD)) {
            *reason = CMD_REJECT_OUT_OF_BOUNDS_AXIS_OR_MASK;
            return false;
        }
    } else if (cmd->test_mode == 1) {
        if (cmd->test_axis >= 3U) {
            *reason = CMD_REJECT_OUT_OF_BOUNDS_AXIS_OR_MASK;
            return false;
        }
        if (cmd->test_rpm < -500 || cmd->test_rpm > 500) {
            *reason = CMD_REJECT_NON_FINITE_PARAM;
            return false;
        }
    }

    return true;
}

extern "C" bool RobotInterface_ValidateAndSetCommand(const RobotCommand_t* cmd, CommandRejectionReason_t* out_reason) {
    if (!cmd || !command_queue) return false;

    uint32_t active_session = RobotInterface_GetSessionId();
    CommandRejectionReason_t reason = CMD_REJECT_NONE;

    bool valid = RobotInterface_ValidateCommand(
        cmd,
        active_session,
        g_last_accepted_sequence,
        g_has_accepted_sequence,
        &reason
    );

    if (out_reason) {
        *out_reason = reason;
    }

    if (!valid) {
        RobotInterface_RecordRejection(reason);
        return false;
    }

    taskENTER_CRITICAL();
    g_last_accepted_sequence = cmd->sequence;
    g_has_accepted_sequence = true;
    g_diagnostics.last_accepted_sequence = cmd->sequence;
    g_diagnostics.last_rejection_reason = CMD_REJECT_NONE;
    taskEXIT_CRITICAL();

    RobotCommandEnvelope_t env;
    memcpy(&env.command, cmd, sizeof(RobotCommand_t));

    if (__get_IPSR() != 0) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        env.received_at = xTaskGetTickCountFromISR();
        xQueueOverwriteFromISR(command_queue, &env, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        env.received_at = xTaskGetTickCount();
        xQueueOverwrite(command_queue, &env);
    }

    return true;
}

extern "C" void RobotInterface_SetCommand(const RobotCommand_t* cmd) {
    (void)RobotInterface_ValidateAndSetCommand(cmd, NULL);
}

extern "C" bool RobotInterface_UpdateCommand(RobotCommand_t* out_cmd) {
    if (!out_cmd || !command_queue) return false;
    
    RobotCommandEnvelope_t newest;
    if (xQueueReceive(command_queue, &newest, 0) == pdPASS) {
        g_current_envelope = newest;
        g_have_current_command = true;
    }
    
    if (!g_have_current_command) {
        return false;
    }
    
    const TickType_t now = xTaskGetTickCount();
    const TickType_t age = now - g_current_envelope.received_at;
    const TickType_t ttl_ticks = pdMS_TO_TICKS(
        (g_current_envelope.command.valid_for_ms > 0) ? g_current_envelope.command.valid_for_ms : 250U
    );
    
    if (age > ttl_ticks) {
        taskENTER_CRITICAL();
        g_have_current_command = false;
        memset(&g_current_envelope, 0, sizeof(g_current_envelope));
        g_diagnostics.command_timeout_total++;
        taskEXIT_CRITICAL();
        return false;
    }
    
    memcpy(out_cmd, &g_current_envelope.command, sizeof(RobotCommand_t));
    // The envelope timestamp is captured at acceptance time using the MCU
    // RTOS tick.  Never trust the transport-provided command timestamp.
    out_cmd->timestamp_ms = g_current_envelope.received_at;
    return true;
}

extern "C" void RobotInterface_GetTelemetry(RobotTelemetry_t* out_telem) {
    if (!out_telem) return;
    
    taskENTER_CRITICAL();
    memcpy(out_telem, &g_current_telemetry, sizeof(RobotTelemetry_t));
    taskEXIT_CRITICAL();
}

extern "C" void RobotInterface_SetTelemetry(const RobotTelemetry_t* telem) {
    if (!telem) return;
    
    taskENTER_CRITICAL();
    memcpy(&g_current_telemetry, telem, sizeof(RobotTelemetry_t));
    taskEXIT_CRITICAL();
}

extern "C" void RobotInterface_GetDiagnostics(RobotDiagnostics_t* out_diag) {
    if (!out_diag) return;
    taskENTER_CRITICAL();
    memcpy(out_diag, &g_diagnostics, sizeof(RobotDiagnostics_t));
    taskEXIT_CRITICAL();
}

extern "C" void RobotInterface_RecordRejection(CommandRejectionReason_t reason) {
    taskENTER_CRITICAL();
    g_diagnostics.rejected_command_total++;
    g_diagnostics.last_rejection_reason = reason;
    switch (reason) {
        case CMD_REJECT_INVALID_PROTOCOL:
        case CMD_REJECT_NON_FINITE_PARAM:
        case CMD_REJECT_OUT_OF_BOUNDS_AXIS_OR_MASK:
        case CMD_REJECT_UNSUPPORTED_MODE:
            g_diagnostics.invalid_command_total++;
            break;
        case CMD_REJECT_SESSION_MISMATCH:
            g_diagnostics.session_mismatch_total++;
            break;
        case CMD_REJECT_STALE_SEQUENCE:
            g_diagnostics.stale_sequence_total++;
            break;
        case CMD_REJECT_EXPIRED_TTL:
            g_diagnostics.command_timeout_total++;
            break;
        default:
            break;
    }
    taskEXIT_CRITICAL();
}

extern "C" void RobotInterface_InvalidateCommand(void) {
    taskENTER_CRITICAL();
    if (command_queue) {
        xQueueReset(command_queue);
    }
    g_have_current_command = false;
    memset(&g_current_envelope, 0, sizeof(g_current_envelope));
    taskEXIT_CRITICAL();
}

extern "C" uint32_t RobotInterface_GetSessionId(void) {
    uint32_t session;
    taskENTER_CRITICAL();
    session = g_session_id;
    taskEXIT_CRITICAL();
    return session;
}

extern "C" void RobotInterface_SetSessionId(uint32_t session_id) {
    taskENTER_CRITICAL();
    if (g_session_id == session_id) {
        taskEXIT_CRITICAL();
        return;
    }
    g_session_id = session_id;
    g_has_accepted_sequence = false;
    g_last_accepted_sequence = 0;
    if (command_queue) {
        xQueueReset(command_queue);
    }
    g_have_current_command = false;
    memset(&g_current_envelope, 0, sizeof(g_current_envelope));
    taskEXIT_CRITICAL();
}
