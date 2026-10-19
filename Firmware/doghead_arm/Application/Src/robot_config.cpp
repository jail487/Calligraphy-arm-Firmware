#include "robot_config.h"
#include "trajectory_planner.h"
#include "nidec_motor_driver.h"
#include "safety_manager.h"
#include <cmath>

// ==========================================================
// 全域控制與回授變數定義
// ==========================================================
RobotState_t robot_state = {0};

// ==========================================================
// C++ 類別實體定義
// ==========================================================
// 參數: (Kp, Ki, Kd, Kv_速度前饋, Ka_加速度前饋, max_rpm)
// 註解說明：
// 馬達未經減速機的 Spec 硬體最大轉速為 5000 RPM。
// 現階段帶載 Bringup 測試統一綁定 BRINGUP_MAX_MOTOR_RPM (150 RPM) 以防止 PID 輸出超過安全檢查導致 INVALID_COMMAND 鎖死。
// 未來測試穩定後，只需修改 robot_config.h 的 BRINGUP_MAX_MOTOR_RPM 宏至 5000.0f 即可解鎖規格全速。
// 依據三軸空間立體 Benchmark 最佳冠軍參數 (總分 88.3 分，跟隨誤差 < 0.8°，穩態誤差 0.01°，Ka=0 消除超調衝擊)
PositionController joint1_pid(8.3f, 0.32f, 0.026f, 1.20f, 0.00f, BRINGUP_MAX_MOTOR_RPM);
PositionController joint2_pid(5.8f, 0.22f, 0.026f, 1.20f, 0.00f, BRINGUP_MAX_MOTOR_RPM);
PositionController joint3_pid(7.5f, 0.25f, 0.025f, 1.00f, 0.00f, BRINGUP_MAX_MOTOR_RPM);

// ==========================================================
// API 實作
// ==========================================================
extern "C" bool Robot_AreJointTargetsValid(float theta1_deg, float theta2_deg, float theta3_deg) {
    return std::isfinite(theta1_deg) && std::isfinite(theta2_deg) && std::isfinite(theta3_deg) &&
           theta1_deg >= JOINT1_MIN_DEG && theta1_deg <= JOINT1_MAX_DEG &&
           theta2_deg >= JOINT2_MIN_DEG && theta2_deg <= JOINT2_MAX_DEG &&
           theta3_deg >= JOINT3_MIN_DEG && theta3_deg <= JOINT3_MAX_DEG;
}

extern "C" void Robot_SetPidGains(uint8_t axis, float kp, float ki, float kd, float kv, float ka) {
    if (axis == 0 || axis == 10 || axis == 11) {
        joint1_pid.setGains(kp, ki, kd, kv, ka);
    }
    if (axis == 1 || axis == 10 || axis == 12) {
        joint2_pid.setGains(kp, ki, kd, kv, ka);
    }
    if (axis == 2 || axis == 10 || axis == 13) {
        joint3_pid.setGains(kp, ki, kd, kv, ka);
    }
}

