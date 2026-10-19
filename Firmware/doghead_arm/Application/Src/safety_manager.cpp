#include "safety_manager.h"
#include "robot_config.h"
#include "motor_config.h"
#include "nidec_motor_driver.h"
#include "robot_interface.h"
#include "homing_calibration.h"

static volatile RobotSafetyState_t safety_state = ROBOT_STATE_SAFE;
static volatile RobotFaultCode_t safety_fault = ROBOT_FAULT_NONE;

static void DisableAllMotors(void) {
    Motor_Stop(&motor_joint_13pin);
    Motor_Stop(&motor_joint_13pin_2);
    Motor_Stop(&motor_joint_13pin_3);
}

extern "C" void RobotSafety_Init(void) { 
    safety_fault = ROBOT_FAULT_NONE; 
#if (ACTUATOR_HARDWARE_OUTPUT_ENABLED != 0)
    safety_state = ROBOT_STATE_HOMING; 
#else
    safety_state = ROBOT_STATE_SAFE;
#endif
    DisableAllMotors(); 
}

extern "C" RobotSafetyState_t RobotSafety_GetState(void) { 
    return safety_state; 
}

extern "C" RobotFaultCode_t RobotSafety_GetFault(void) { 
    return safety_fault; 
}

extern volatile int32_t live_test_enable;

extern "C" bool RobotSafety_CanDrive(void) { 
    if (ACTUATOR_HARDWARE_OUTPUT_ENABLED == 0) return false;
    if (live_test_enable == 1) return true;
    return safety_state == ROBOT_STATE_RUNNING; 
}

extern "C" bool RobotSafety_CanHomingDrive(void) {
    return ACTUATOR_HARDWARE_OUTPUT_ENABLED != 0 &&
           safety_state == ROBOT_STATE_HOMING;
}

extern "C" bool RobotSafety_CanExecute(void) {
    return safety_state == ROBOT_STATE_SAFE ||
           safety_state == ROBOT_STATE_READY ||
           safety_state == ROBOT_STATE_RUNNING;
}

extern "C" bool RobotSafety_IsFaulted(void) { 
    return safety_state == ROBOT_STATE_FAULT || safety_state == ROBOT_STATE_ESTOP; 
}

extern "C" bool RobotSafety_RequestRun(void) {
    if (safety_state == ROBOT_STATE_FAULT || safety_state == ROBOT_STATE_ESTOP || safety_state == ROBOT_STATE_HOMING) {
        return false;
    }
    safety_state = ROBOT_STATE_RUNNING;
    return true;
}

extern "C" void RobotSafety_Stop(void) {
    if (safety_state == ROBOT_STATE_RUNNING) safety_state = ROBOT_STATE_READY;
    DisableAllMotors();
    RobotInterface_InvalidateCommand();
}

extern "C" bool RobotSafety_RequestHoming(void) {
    if (safety_state == ROBOT_STATE_ESTOP) return false;
#if (ACTUATOR_HARDWARE_OUTPUT_ENABLED == 0)
    return false;
#else
    safety_fault = ROBOT_FAULT_NONE;
    safety_state = ROBOT_STATE_HOMING;
    DisableAllMotors();
    Robot_HomingStart();
    return true;
#endif
}

extern "C" void RobotSafety_HomingComplete(void) { 
    if (safety_state == ROBOT_STATE_HOMING) safety_state = ROBOT_STATE_READY; 
    DisableAllMotors(); 
}

extern "C" void RobotSafety_TriggerFault(RobotFaultCode_t fault) { 
    if (safety_state != ROBOT_STATE_ESTOP) { 
        safety_fault = fault; 
        safety_state = ROBOT_STATE_FAULT; 
        DisableAllMotors(); 
        RobotInterface_InvalidateCommand();
    } 
}

extern "C" void RobotSafety_TriggerEStop(void) { 
    safety_fault = ROBOT_FAULT_NONE; 
    safety_state = ROBOT_STATE_ESTOP; 
    DisableAllMotors(); 
    RobotInterface_InvalidateCommand();
}


extern "C" void RobotSafety_ClearFault(void) { 
    if (safety_state == ROBOT_STATE_FAULT) { 
        safety_fault = ROBOT_FAULT_NONE; 
        safety_state = ROBOT_STATE_SAFE; 
        DisableAllMotors(); 
    } 
}
