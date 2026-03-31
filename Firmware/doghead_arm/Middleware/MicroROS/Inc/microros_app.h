/**
 * @file microros_app.h
 * @brief micro-ROS 應用層標頭檔
 */

#ifndef MICROROS_APP_H
#define MICROROS_APP_H

#include <stdbool.h>

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
 * @note 建議以 10-50Hz 頻率呼叫
 */
void MicroROS_PublishJointStates(void);

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
