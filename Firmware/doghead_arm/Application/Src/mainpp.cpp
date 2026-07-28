#include "mainpp.h"
#include "nidec_motor_driver.h"
#include "safety_manager.h"
#include "homing_calibration.h"
#include "test_scenarios.h"
#include <cmath>

extern "C" void Robot_Init(void) {
    // 呼叫 C 語言底層驅動初始化
    Motor_System_Config();
    RobotSafety_Init();

    // 開機先進入歸零流程，歸零完成後才回到一般控制
    // homing_joint3_done = true;  // 預設設為 true，不校正第三軸直接開始測試

    // 預設先回到正常控制模式，歸零完成後再由控制任務啟動運動學
    test_mode_selection = 0;
    ik_mode_enabled = false;

    // 重置 PID 狀態
    joint1_pid.reset();
    joint2_pid.reset();
    joint3_pid.reset();
    
    // 重置軌跡規劃器
    traj_joint1.reset();
    traj_joint2.reset();
    traj_joint3.reset();

    // 預設目標設為當前位置 (防止開機暴衝)
    robot_target.pose_x = 65.764f;
    robot_target.pose_y = 0.0f;
    robot_target.pose_z = 103.228f;

    // 初始角度設定 (對齊最新歸零定標角度：115.0°, 180.0°, 0.0°)
    robot_target.target_theta1_deg = 115.0f;
    robot_target.target_theta2_deg = 180.0f;
    robot_target.target_theta3_deg = 0.0f;
    robot_target.target_angle1_deg = robot_target.target_theta1_deg;
    robot_target.target_angle2_deg = robot_target.target_theta2_deg;
    robot_target.target_angle3_deg = robot_target.target_theta3_deg;
}

extern "C" void Robot_Loop(float dt_seconds) {
    if (dt_seconds <= 0.0f || !std::isfinite(dt_seconds)) {
        RobotSafety_TriggerFault(ROBOT_FAULT_TRAJECTORY);
        return;
    }
    if (!RobotSafety_CanDrive()) {
        Motor_Stop(&motor_joint_13pin);
        Motor_Stop(&motor_joint_13pin_2);
        Motor_Stop(&motor_joint_13pin_3);
        return;
    }
    // --- 測試模式：直接控制馬達速度 (Legacy Mode 1/2/3) ---
    if (test_mode_selection > 0) {
        return; 
    }
    
    // --- 步驟 A: 讀取感測器 (Feedback) ---
    Motor_Update(&motor_joint_13pin);
    Motor_Update(&motor_joint_13pin_2);
    Motor_Update(&motor_joint_13pin_3);

    // 取得真實角度 (Degree)
    robot_state.real_theta1 = Motor_GetAngle(&motor_joint_13pin);
    robot_state.real_theta2 = Motor_GetAngle(&motor_joint_13pin_2);
    robot_state.real_theta3 = Motor_GetAngle(&motor_joint_13pin_3);

    // --- 步驟 B: 計算目標角度 (Setpoint) ---
    robot_target.target_angle1_deg = robot_target.target_theta1_deg;
    robot_target.target_angle2_deg = robot_target.target_theta2_deg;
    robot_target.target_angle3_deg = robot_target.target_theta3_deg;

    bool ik_direct_mode = false;

    // 完整的 3-DOF 逆運動學 (XYZ 轉換到 M1, M2, M3 角度)
    if (ik_mode_enabled) {
        MotorAngles3D solution = kinematics.solveIK3D({robot_target.pose_x, robot_target.pose_y, robot_target.pose_z}, BASE_R_OFFSET, robot_state.real_theta1, robot_state.real_theta2);
        if (solution.is_reachable) {
            robot_target.target_angle1_deg = FiveBarKinematics::rad2deg(solution.theta1);
            robot_target.target_angle2_deg = FiveBarKinematics::rad2deg(solution.theta2);
            robot_target.target_angle3_deg = FiveBarKinematics::rad2deg(solution.theta3);
            ik_direct_mode = true;
            if (!Robot_AreJointTargetsValid(robot_target.target_angle1_deg, robot_target.target_angle2_deg, robot_target.target_angle3_deg)) {
                RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
                return;
            }
        } else {
            RobotSafety_TriggerFault(ROBOT_FAULT_INVALID_COMMAND);
            return;
        }
    }

    // --- 步驟 C: 安全檢查 (FK) --- 
    Point3D current_pos = kinematics.solveFK3D(
        FiveBarKinematics::deg2rad(robot_state.real_theta1),
        FiveBarKinematics::deg2rad(robot_state.real_theta2),
        FiveBarKinematics::deg2rad(robot_state.real_theta3),
        BASE_R_OFFSET
    );
    robot_state.real_x = current_pos.x;
    robot_state.real_y = current_pos.y;
    robot_state.real_z = current_pos.z;
    
    // 邊界檢查 (可在此處加入高度保護邏輯)

    // --- 步驟 D/E: IK 直出模式時直接把目標角送進 PID，不經過軌跡規劃 ---
    Motor_Start(&motor_joint_13pin);
    Motor_Start(&motor_joint_13pin_2);
    Motor_Start(&motor_joint_13pin_3);

    if (ik_direct_mode) {
        robot_state.pid_output_rpm1 = joint1_pid.update(robot_target.target_angle1_deg, 0.0f, 0.0f, robot_state.real_theta1, dt_seconds);
        robot_state.pid_output_rpm2 = joint2_pid.update(robot_target.target_angle2_deg, 0.0f, 0.0f, robot_state.real_theta2, dt_seconds);
        robot_state.pid_output_rpm3 = joint3_pid.update(robot_target.target_angle3_deg, 0.0f, 0.0f, robot_state.real_theta3, dt_seconds);
        robot_state.traj_vel1 = 0.0f;
        robot_state.traj_vel2 = 0.0f;
        robot_state.traj_vel3 = 0.0f;
    } else {
        // --- 步驟 D: 軌跡規劃 (Trajectory Planning) ---
        traj_joint1.update(robot_target.target_angle1_deg, dt_seconds);
        traj_joint2.update(robot_target.target_angle2_deg, dt_seconds);
        traj_joint3.update(robot_target.target_angle3_deg, dt_seconds);
        
        // 抓取軌跡狀態 (Pos, Vel, Acc)
        float traj_pos1 = traj_joint1.getPosition();
        robot_state.traj_vel1 = traj_joint1.getVelocity();
        float traj_acc1 = traj_joint1.getAcceleration();

        float traj_pos2 = traj_joint2.getPosition();
        robot_state.traj_vel2 = traj_joint2.getVelocity();
        float traj_acc2 = traj_joint2.getAcceleration();

        float traj_pos3 = traj_joint3.getPosition();
        robot_state.traj_vel3 = traj_joint3.getVelocity();
        float traj_acc3 = traj_joint3.getAcceleration();

        // --- 步驟 E: PID 計算 ---
        robot_state.pid_output_rpm1 = joint1_pid.update(traj_pos1, robot_state.traj_vel1, traj_acc1, robot_state.real_theta1, dt_seconds);
        robot_state.pid_output_rpm2 = joint2_pid.update(traj_pos2, robot_state.traj_vel2, traj_acc2, robot_state.real_theta2, dt_seconds);
        robot_state.pid_output_rpm3 = joint3_pid.update(traj_pos3, robot_state.traj_vel3, traj_acc3, robot_state.real_theta3, dt_seconds);
    }

    // --- 步驟 F: 極限開關硬體保護與角度動態校正 ---
    // 由於 3 軸的 home_search_rpm 皆為正值，代表「正轉」會朝極限開關壓過去
    // 若碰到開關，強制重新校正角度，並攔截正向的馬達輸出
    if (homing_joint1_done && HAL_GPIO_ReadPin(HOME_SWITCH_PORT, JOINT1_HOME_PIN) == GPIO_PIN_RESET &&
        robot_state.pid_output_rpm1 * home_search_rpm_m1 > 0.0f) {
        RobotSafety_TriggerFault(home_search_rpm_m1 >= 0 ? ROBOT_FAULT_POSITIVE_LIMIT : ROBOT_FAULT_NEGATIVE_LIMIT);
        return;
    }
    if (homing_joint2_done && HAL_GPIO_ReadPin(HOME_SWITCH_PORT, JOINT2_HOME_PIN) == GPIO_PIN_RESET &&
        robot_state.pid_output_rpm2 * home_search_rpm_m2 > 0.0f) {
        RobotSafety_TriggerFault(home_search_rpm_m2 >= 0 ? ROBOT_FAULT_POSITIVE_LIMIT : ROBOT_FAULT_NEGATIVE_LIMIT);
        return;
    }
    if (homing_joint3_done && HAL_GPIO_ReadPin(HOME_SWITCH_PORT, JOINT3_HOME_PIN) == GPIO_PIN_RESET &&
        robot_state.pid_output_rpm3 * home_search_rpm_m3 > 0.0f) {
        RobotSafety_TriggerFault(home_search_rpm_m3 >= 0 ? ROBOT_FAULT_POSITIVE_LIMIT : ROBOT_FAULT_NEGATIVE_LIMIT);
        return;
    }

    // --- 步驟 G: 輸出到底層 ---
    Motor_SetSpeed(&motor_joint_13pin, (int32_t)robot_state.pid_output_rpm1);
    Motor_SetSpeed(&motor_joint_13pin_2, (int32_t)robot_state.pid_output_rpm2);
    Motor_SetSpeed(&motor_joint_13pin_3, (int32_t)robot_state.pid_output_rpm3);
}

extern "C" void Robot_ControlTask_Tick(float dt_seconds) {
    if (Robot_IsHomingActive()) {
        Robot_Homing_Tick();
    } else {
        if (test_mode_selection == 0) {
            Robot_SquareTest_Tick();
            Robot_Loop(dt_seconds);
        } else {
            Robot_TestMode_Tick();
        }
    }
}
