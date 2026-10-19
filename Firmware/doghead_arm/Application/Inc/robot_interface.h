#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ==========================================================
// Protocol 常數與狀態機
// ==========================================================
#define DOGHEAD_PROTOCOL_VERSION 1

// Session IDs carry the MCU boot epoch in the upper 16 bits and a host Arm
// epoch in the lower 16 bits.  This keeps the existing message contract while
// preventing an old session from being replayed after an MCU reboot.
#define DOGHEAD_SESSION_BOOT_TAG_SHIFT 16U
#define DOGHEAD_SESSION_BOOT_TAG_MASK 0xFFFF0000U
#define DOGHEAD_SESSION_EPOCH_MASK 0x0000FFFFU

typedef enum {
    CONTROL_MODE_DISABLED = 0,
    CONTROL_MODE_MOTOR_VELOCITY_TEST = 1,
    CONTROL_MODE_JOINT_PVA = 2,
    CONTROL_MODE_HOLD = 3,
    CONTROL_MODE_STOP = 4
} RobotControlMode_t;

// ==========================================================
// 系統狀態定義 (State Machine)
// ==========================================================
typedef enum {
    ROBOT_SYS_STATE_INIT = 0,
    ROBOT_SYS_STATE_HOMING,
    ROBOT_SYS_STATE_IDLE,       // 歸零完成，等待指令
    ROBOT_SYS_STATE_RUNNING,    // 正在執行控制指令
    ROBOT_SYS_STATE_COMM_LOST,  // 通訊超時
    ROBOT_SYS_STATE_FAULT       // 發生錯誤 (如極限開關觸發、硬體異常)
} RobotSystemState_e;

// ==========================================================
// Motor Test Dedicated Payload
// ==========================================================
//
// -------  END-TO-END RPM UNIT CONTRACT  -----------------------
// All velocity quantities flow as joint/output-shaft RPM through the
// bringup control loop unless stated otherwise:
//
//   Source                      | Unit               | Notes
//   ----------------------------|--------------------|--------------------------
//   PID update() output         | joint RPM          | deg/s ff converted via
//                               |                    | (deg/s / 360) * 60 * kv
//   PVA vel_rad_s command       | joint rad/s        | converted to deg/s in
//                               |                    | mainpp then to joint RPM
//   Motor Test target_motor_rpm | joint RPM          | NOT motor shaft RPM;
//                               |                    | name retained for wire
//                               |                    | compatibility
//   Motor_SetSpeed(rpm) input   | joint RPM          | applies gear ratio once
//                               |                    | internally via
//                               |                    | Joint_RPM_To_Cmd_Freq_Hz
//   current_rpm_cmd (Motor_t)   | joint RPM          | = Motor_SetSpeed input
//   commanded_motor_rpm[] telem | joint RPM          | = pid_output_rpm;
//                               |                    | NOT motor shaft RPM
//   real_rpm1/2/3 (measured)    | joint RPM          | from encoder feedback
//
// Gear ratio is applied EXACTLY ONCE: inside Impl_Freq13Pin_SetSpeed().
// Neither the PVA PID path nor the Motor Test path applies it before that.
// ---------------------------------------------------------------
typedef struct {
    uint8_t axis;
    float target_motor_rpm;        /* Joint/output-shaft RPM (NOT motor shaft RPM).
                                    * Despite the name, this carries the same unit as
                                    * Motor_SetSpeed() input.  Gear ratio is applied
                                    * exactly once inside the driver. */
    float requested_accel_rpm_s;   /* Joint/output-shaft RPM/s */
    float requested_jerk_rpm_s2;   /* Joint/output-shaft RPM/s^2 */
    bool enable;
} MotorTestPayload_t;

// ==========================================================
// Joint PVA Dedicated Payload
// ==========================================================
typedef struct {
    uint8_t enable_mask;
    float position_rad[3];
    float velocity_rad_s[3];
    float acceleration_rad_s2[3];
} JointPvaPayload_t;

// ==========================================================
// 機器人命令結構 (Robot Command)
// ==========================================================
typedef struct {
    uint16_t protocol_version;
    uint32_t session_id;
    uint32_t sequence;
    uint16_t valid_for_ms;
    // Set by RobotInterface_UpdateCommand() from the RTOS receive tick.  The
    // transport must not supply this value because command age is measured
    // from the MCU receive time, not from a host timestamp.
    uint32_t timestamp_ms;

    RobotControlMode_t control_mode;
    
    // Dedicated mode payloads
    MotorTestPayload_t motor_test;
    JointPvaPayload_t joint_pva;

    // Legacy fields for backward compatibility
    int32_t test_mode;     // 0=off, 1=single-axis test
    uint8_t test_axis;     // 0, 1, 2
    int32_t test_rpm;      // -100 ... +100
} RobotCommand_t;

// ==========================================================
// 機器人遙測結構 (Robot Telemetry Snapshot)
// ==========================================================
typedef struct {
    RobotSystemState_e system_state;
    
    // 實際角度
    float real_theta1;
    float real_theta2;
    float real_theta3;
    
    // 真實 RPM
    float real_rpm1;
    float real_rpm2;
    float real_rpm3;
    
    // Command/output provenance.  requested is the incoming command mask;
    // applied is the validated logical mask used by shadow execution; and
    // physical is the mask that passed the final actuator gate.  These are
    // deliberately separate so output-off telemetry cannot be mistaken for
    // a skipped controller or for physical drive.
    RobotControlMode_t control_mode;
    bool has_command;
    uint8_t requested_enable_mask;
    uint8_t applied_enable_mask;
    uint8_t physical_enable_mask;
    uint32_t active_command_sequence;
    uint32_t command_received_tick;
    float target_position_rad[3];
    float target_velocity_rad_s[3];
    float target_acceleration_rad_s2[3];
    float applied_position_rad[3];
    float applied_velocity_rad_s[3];
    /* Joint/output-shaft RPM commanded by the controller (= pid_output_rpm after
     * validation).  This is NOT motor shaft RPM.  Gear ratio conversion is applied
     * exactly once, inside the motor driver, when the command is executed.
     * Distinct from real_rpm1/2/3 which are measured encoder-based values. */
    float commanded_motor_rpm[3];

    // Cumulative encoder count, not the current timer counter snapshot.
    int64_t motor1_count;
    int64_t motor2_count;
    int64_t motor3_count;
    
    uint32_t current_time_ms;
} RobotTelemetry_t;

// ==========================================================
// 機器人遙測與診斷結構
// ==========================================================
typedef enum {
    CMD_REJECT_NONE = 0,
    CMD_REJECT_INVALID_PROTOCOL,
    CMD_REJECT_SESSION_MISMATCH,
    CMD_REJECT_STALE_SEQUENCE,
    CMD_REJECT_EXPIRED_TTL,
    CMD_REJECT_NON_FINITE_PARAM,
    CMD_REJECT_OUT_OF_BOUNDS_AXIS_OR_MASK,
    CMD_REJECT_UNSUPPORTED_MODE
} CommandRejectionReason_t;

typedef struct {
    uint32_t control_deadline_miss_total;
    uint32_t command_timeout_total;
    uint32_t invalid_command_total;
    uint32_t stale_sequence_total;
    uint32_t session_mismatch_total;
    uint32_t rejected_command_total;
    uint32_t last_accepted_sequence;
    CommandRejectionReason_t last_rejection_reason;
} RobotDiagnostics_t;

// ==========================================================
// 公開 API
// ==========================================================

// 初始化通訊介面
void RobotInterface_Init(void);

// 取得當前的 Session ID
uint32_t RobotInterface_GetSessionId(void);

// 設定當前的 Session ID (例如 Arm 時)
void RobotInterface_SetSessionId(uint32_t session_id);

// Account for malformed transport/session-control input without accepting it.
void RobotInterface_RecordRejection(CommandRejectionReason_t reason);

// 驗證指令 (純邏輯驗證，無側寫/改動 Mailbox)
bool RobotInterface_ValidateCommand(const RobotCommand_t* cmd, uint32_t active_session, uint32_t last_seq, bool has_last_seq, CommandRejectionReason_t* out_reason);

// 驗證並原子性寫入新指令
bool RobotInterface_ValidateAndSetCommand(const RobotCommand_t* cmd, CommandRejectionReason_t* out_reason);

// 供通訊層 (如 micro-ROS 或 UART) 呼叫，寫入新指令 (Double Buffered)
void RobotInterface_SetCommand(const RobotCommand_t* cmd);

// 供控制迴圈呼叫，取得最新指令並進行 Timeout 檢查
bool RobotInterface_UpdateCommand(RobotCommand_t* out_cmd);

// 供通訊層呼叫，取得當下狀態快照 (Snapshot)
void RobotInterface_GetTelemetry(RobotTelemetry_t* out_telem);

// 供控制迴圈呼叫，更新當前狀態快照
void RobotInterface_SetTelemetry(const RobotTelemetry_t* telem);

// 供通訊層或控制迴圈呼叫，取得診斷與拒絕計數
void RobotInterface_GetDiagnostics(RobotDiagnostics_t* out_diag);

// 供通訊斷線或 Fault 時呼叫，立刻無條件銷毀佇列與快取指令
void RobotInterface_InvalidateCommand(void);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_INTERFACE_H
