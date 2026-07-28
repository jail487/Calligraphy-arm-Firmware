/**
 * @file mainpp.h
 * @brief C 與 C++ 的橋接介面 (Main C++)
 */
#ifndef MAINPP_H
#define MAINPP_H

#include "robot_config.h"
#include "homing_calibration.h"
#include "test_scenarios.h"
#include "safety_manager.h"

#ifdef __cplusplus
#include "trajectory_planner.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// 初始化機器人 (馬達、PID、工作狀態)
void Robot_Init(void);

// 機器人主迴圈 (需由 FreeRTOS 週期呼叫)
// dt_seconds: 距離上次呼叫的時間差 (秒)，例如 1ms = 0.001f
void Robot_Loop(float dt_seconds);
void Robot_ControlTask_Tick(float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif // MAINPP_H
