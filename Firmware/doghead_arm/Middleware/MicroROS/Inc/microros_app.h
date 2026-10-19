/**
 * @file microros_app.h
 * @brief micro-ROS 應用層標頭檔
 */

#ifndef MICROROS_APP_H
#define MICROROS_APP_H

#include <stdbool.h>
#include "robot_interface.h"

/**
 * @brief micro-ROS topic telemetry & type support capability gate.
 *
 * HISTORICAL NOTE & SAFETY STATUS:
 * Historical pre-compiled static library type-support contained C++ global constructors
 * (_GLOBAL__sub_I_...) that executed during __libc_init_array prior to main(), causing an
 * immediate STM32 HardFault on bare-metal startup. This historical failure belonged to the
 * incompatible pre-T22 library.
 *
 * In T22, a compatible natural-GCC-attribute Humble static library was rebuilt and verified
 * through ping/session HIL (zero transport error counters and zero CFSR).
 *
 * Full ROS topic telemetry (/joint_states, /doghead/heartbeat, and doghead_interfaces) and RCL entity creation
 * now default ON (1) for HIL telemetry bringup. Explicit MICROROS_ENABLE_TOPICS=0 or
 * MICROROS_ENABLE_TOPIC_TELEMETRY=0 remains supported as the diagnostic ping-only fallback mode.
 */
#if defined(MICROROS_ENABLE_TOPICS)
  #ifndef MICROROS_ENABLE_TOPIC_TELEMETRY
    #define MICROROS_ENABLE_TOPIC_TELEMETRY MICROROS_ENABLE_TOPICS
  #endif
#endif

#ifndef MICROROS_ENABLE_TOPIC_TELEMETRY
  #define MICROROS_ENABLE_TOPIC_TELEMETRY 1
#endif

#ifndef MICROROS_ENABLE_DOGHEAD_INTERFACES
  #define MICROROS_ENABLE_DOGHEAD_INTERFACES 1
#endif

#if MICROROS_ENABLE_TOPIC_TELEMETRY
  #if defined(MICROROS_ENABLE_DOGHEAD_INTERFACES) && (MICROROS_ENABLE_DOGHEAD_INTERFACES == 1)
    #if defined(__has_include)
      #if __has_include(<doghead_interfaces/msg/mcu_state.h>)
        #define MICROROS_HAS_DOGHEAD_INTERFACES 1
      #else
        #define MICROROS_HAS_DOGHEAD_INTERFACES 0
      #endif
    #else
      #define MICROROS_HAS_DOGHEAD_INTERFACES 1
    #endif
  #else
    #ifndef MICROROS_HAS_DOGHEAD_INTERFACES
      #define MICROROS_HAS_DOGHEAD_INTERFACES 0
    #endif
  #endif
#else
  #undef MICROROS_HAS_DOGHEAD_INTERFACES
  #define MICROROS_HAS_DOGHEAD_INTERFACES 0
#endif

typedef enum {
    MICROROS_ENTITY_STAGE_NONE = 0,
    MICROROS_ENTITY_STAGE_SUPPORT_INIT,
    MICROROS_ENTITY_STAGE_NODE_INIT,
    MICROROS_ENTITY_STAGE_HEARTBEAT_PUB_INIT,
    MICROROS_ENTITY_STAGE_JOINT_STATES_PUB_INIT,
#if MICROROS_HAS_DOGHEAD_INTERFACES
    MICROROS_ENTITY_STAGE_MCU_STATE_PUB_INIT,
    MICROROS_ENTITY_STAGE_MCU_DIAG_PUB_INIT,
    MICROROS_ENTITY_STAGE_SESSION_CONTROL_SUB_INIT,
    MICROROS_ENTITY_STAGE_JOINT_SETPOINT_SUB_INIT,
    MICROROS_ENTITY_STAGE_MOTOR_TEST_SUB_INIT,
    MICROROS_ENTITY_STAGE_EXECUTOR_INIT,
    MICROROS_ENTITY_STAGE_EXECUTOR_ADD_SESSION_CONTROL,
    MICROROS_ENTITY_STAGE_EXECUTOR_ADD_JOINT_SETPOINT,
    MICROROS_ENTITY_STAGE_EXECUTOR_ADD_MOTOR_TEST,
#endif
    MICROROS_ENTITY_STAGE_COMPLETE
} MicroROSEntityStage_t;

typedef enum {
    MICROROS_DISCONNECT_NONE = 0,
    MICROROS_DISCONNECT_PING_FAIL,
    MICROROS_DISCONNECT_HEARTBEAT_PUB_FAIL,
    MICROROS_DISCONNECT_JOINT_STATES_PUB_FAIL,
    MICROROS_DISCONNECT_MCU_STATE_PUB_FAIL,
    MICROROS_DISCONNECT_MCU_DIAG_PUB_FAIL
} MicroROSDisconnectReason_t;

extern volatile uint8_t g_microros_entity_stage;
extern volatile uint8_t g_microros_last_failed_stage;
extern volatile uint8_t g_microros_last_disconnect_reason;
extern volatile int32_t g_microros_last_error_rc;

#if MICROROS_ENABLE_TOPIC_TELEMETRY
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/u_int32.h>
#if MICROROS_HAS_DOGHEAD_INTERFACES
#include <doghead_interfaces/msg/mcu_state.h>
#include <doghead_interfaces/msg/mcu_diagnostics.h>
#include <doghead_interfaces/msg/mcu_joint_setpoint.h>
#include <doghead_interfaces/msg/mcu_motor_test_command.h>
#include <doghead_interfaces/msg/mcu_session_control.h>
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 micro-ROS（Node, Publisher, Subscriber, Executor）
 * @return true 成功, false 失敗
 * @note 應在 FreeRTOS 任務中呼叫，不要在中斷或 main 初始化階段呼叫
 */
bool MicroROS_Init(void);

/**
 * @brief 發布關節狀態到 /joint_states
 * @param telem 機器人姿態與轉速快照
 * @note 建議以 10-50Hz 頻率呼叫
 */
void MicroROS_PublishJointStates(const RobotTelemetry_t * telem);

/**
 * @brief 發布 MCU 狀態到 /doghead/state (當 doghead_interfaces 存在時)
 * @param telem 機器人姿態與轉速快照
 */
void MicroROS_PublishMcuState(const RobotTelemetry_t * telem);

/**
 * @brief 發布 MCU 診斷到 /doghead/diagnostics (當 doghead_interfaces 存在時)
 * @param diag 診斷與拒絕計數
 */
void MicroROS_PublishMcuDiagnostics(const RobotDiagnostics_t * diag);

/**
 * @brief 執行 Executor（處理 Subscription 回呼）
 * @note 建議在 CommTask 中以高頻率呼叫（例如 100Hz）
 */
void MicroROS_Spin(void);

/**
 * @brief 清理 micro-ROS 資源
 * @note 通常在系統關閉時呼叫
 */
void MicroROS_Fini(void);

/**
 * @brief 檢查 micro-ROS 是否已初始化
 * @return true 已初始化, false 未初始化
 */
bool MicroROS_IsInitialized(void);


#ifdef __cplusplus
}
#endif

#endif // MICROROS_APP_H
