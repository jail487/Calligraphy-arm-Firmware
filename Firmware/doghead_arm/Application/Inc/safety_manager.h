#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ROBOT_STATE_SAFE = 0,
  ROBOT_STATE_HOMING,
  ROBOT_STATE_READY,
  ROBOT_STATE_RUNNING,
  ROBOT_STATE_FAULT,
  ROBOT_STATE_ESTOP
} RobotSafetyState_t;

typedef enum {
  ROBOT_FAULT_NONE = 0,
  ROBOT_FAULT_HOMING_TIMEOUT,
  ROBOT_FAULT_HOMING_TRAVEL_LIMIT,
  ROBOT_FAULT_POSITIVE_LIMIT,
  ROBOT_FAULT_NEGATIVE_LIMIT,
  ROBOT_FAULT_INVALID_COMMAND,
  ROBOT_FAULT_TRAJECTORY
} RobotFaultCode_t;

void RobotSafety_Init(void);
RobotSafetyState_t RobotSafety_GetState(void);
RobotFaultCode_t RobotSafety_GetFault(void);
bool RobotSafety_CanDrive(void);
bool RobotSafety_IsFaulted(void);
bool RobotSafety_RequestRun(void);
void RobotSafety_Stop(void);
bool RobotSafety_RequestHoming(void);
void RobotSafety_HomingComplete(void);
void RobotSafety_TriggerFault(RobotFaultCode_t fault);
void RobotSafety_TriggerEStop(void);
void RobotSafety_ClearFault(void);

#ifdef __cplusplus
}
#endif

#endif
