/**
 * @file robot_arm_core.cpp
 * @brief 機器手臂核心邏輯 (v4.0 - 運動學與驅動整合)
 */

#include "mainpp.h"
#include "pid_controller.hpp"
#include "nidec_motor_driver.h"
#include "kinematics.hpp"
#include <queue>
#include <cmath>
#include "main.h"

// ==========================================================
// 控制層全域變數宣告 (用於 Live Expressions)
// ==========================================================
volatile int32_t test_target_rpm_m1 = 100;  
volatile int32_t test_target_rpm_m2 = 100;
volatile int32_t test_target_rpm_m3 = 100;

volatile int32_t test_mode_selection = 1;
volatile int32_t mode1_target_rpm = 100;

volatile float pid_target_theta1 = 0.0f;
volatile float pid_target_theta2 = 0.0f;
volatile float pid_target_theta3 = 0.0f;

volatile float real_theta1 = 0.0f;
volatile float real_theta2 = 0.0f;
volatile float real_theta3 = 0.0f;

volatile float real_rpm1 = 0.0f;
volatile float real_rpm2 = 0.0f;
volatile float real_rpm3 = 0.0f;

volatile float pid_output_rpm1 = 0.0f;
volatile float pid_output_rpm2 = 0.0f;
volatile float pid_output_rpm3 = 0.0f;

// ==========================================================
// 機構參數設定 (請依照實際硬體測量修改！)
// 單位: mm
// ==========================================================
// 假設五連桿參數
#define LINK_L1  100.0f  // 主動臂 (連接馬達)
#define LINK_L2  150.0f  // 從動臂 (連接末端)
#define MOTOR_DIST_D 60.0f // 兩馬達間距

// 建立運動學解算器實體
FiveBarKinematics kinematics(LINK_L1, LINK_L2, MOTOR_DIST_D);

// ==========================================================
// 軌跡規劃器 (Trapezoidal Velocity Profile)
// ==========================================================
class TrapezoidalPlanner {
public:
    TrapezoidalPlanner() { reset(); }

    void reset() {
        _current_pos = 0.0f;
        _current_vel = 0.0f;
        _target_pos = 0.0f;
        _max_vel = 90.0f; // deg/s   <-- [驗證用] 降低速度，原 180
        _max_acc = 180.0f; // deg/s^2 <-- [驗證用] 降低加速度 (溫和模式)，原 360
    }

    // 設定新的目標與移動參數
    void setTarget(float start_pos, float target_pos, float max_vel, float max_acc) {
        _current_pos = start_pos;
        _target_pos = target_pos;
        _max_vel = fabsf(max_vel);
        _max_acc = fabsf(max_acc);
    }

    // 每個時間步呼叫一次，回傳當下應該在的 [位置, 速度, 加速度]
    // 這是一個即時的 Bang-Bang 生成器
    // [Modified] 為了相容原本架構，加入 target_pos 參數，若目標改變會自動呼叫 setTarget 更新
    void update(float target_pos_in, float dt) {
        // 如果目標位置改變了 (且差異大於浮點數誤差)，則更新目標
        if (fabsf(target_pos_in - _target_pos) > 0.001f) {
             // 保持目前的 max_vel/acc 設定，只更新目標
             setTarget(_current_pos, target_pos_in, _max_vel, _max_acc);
        }

        // 1. 計算剩餘距離
        float dist_to_target = _target_pos - _current_pos;
        
        // 2. 判斷需要減速的煞車距離
        // s = (v^2) / (2a)
        float stop_dist = (_current_vel * _current_vel) / (2.0f * _max_acc);

        // 決定加速度方向
        float accel_cmd = 0.0f;
        
        if (dist_to_target > 0) {
            // 正向移動
            if (_current_vel < 0) {
                // 如果當前正在倒退，必須先煞車
                accel_cmd = _max_acc;
            } else if (dist_to_target > stop_dist) {
                // 還沒到煞車點，全力加速直到達限速
                if (_current_vel < _max_vel) accel_cmd = _max_acc;
                else accel_cmd = 0.0f;
            } else {
                // 已進入煞車區，全力減速
                accel_cmd = -_max_acc;
            }
        } else {
            // 負向移動
            if (_current_vel > 0) {
                // 如果當前正在前進，必須先煞車
                accel_cmd = -_max_acc;
            } else if (-dist_to_target > stop_dist) {
                // 還沒到煞車點 (注意符號)，加速負向
                if (_current_vel > -_max_vel) accel_cmd = -_max_acc;
                else accel_cmd = 0.0f;
            } else {
                // 進入煞車區
                accel_cmd = _max_acc;
            }
        }

        // 3. 積分計算下一個狀態
        // v = v0 + a*t
        _current_vel += accel_cmd * dt;
        
        // p = p0 + v*t + 0.5*a*t^2 (為了簡化與穩定，用 p = p + v*dt)
        _current_pos += _current_vel * dt;

        // 4. 到達目標後的收斂處理 (避免在目標點震盪)
        // 如果非常接近目標且速度很慢，直接鎖定
        if (fabsf(dist_to_target) < 0.1f && fabsf(_current_vel) < 5.0f) {
            _current_pos = _target_pos;
            _current_vel = 0.0f;
            accel_cmd = 0.0f;
        }

        _last_acc = accel_cmd;
    }

    float getPosition() const { return _current_pos; }
    float getVelocity() const { return _current_vel; }
    float getAcceleration() const { return _last_acc; }

private:
    float _current_pos;
    float _current_vel;
    float _last_acc;
    
    float _target_pos;
    float _max_vel;
    float _max_acc;
};

// 為每個關節建立梯形規劃器
TrapezoidalPlanner traj_joint1; // Motor 1
TrapezoidalPlanner traj_joint2; // Motor 2
TrapezoidalPlanner traj_joint3; // Motor 3

// ==========================================================
// PID 控制器與變數 (含前饋參數)
// ==========================================================
// 參數說明: (Kp, Ki, Kd, Kv_速度前饋, Ka_加速度前饋, max_rpm)
// Kv: 對於速度控制馬達，通常設為 1.0 (直接對應速度指令)
// Ka: 加速度補償係數，用於補償慣量，建議從 0.05~0.2 開始調整

// 關節 1 (13-Pin 馬達 - 24H702U030)
PositionController joint1_pid(5.0f, 0.1f, 0.0f, 1.0f, 0.1f, 3000.0f);
// 關節 2 (13-Pin 馬達 - 24H702U030) [NEW]
PositionController joint2_pid(5.0f, 0.1f, 0.0f, 1.0f, 0.1f, 3000.0f);

// 關節 3 (13-Pin 馬達)
PositionController joint3_pid(8.0f, 0.2f, 0.0f, 1.0f, 0.15f, 4000.0f);

// 測試目標 (座標模式)
float target_x = 0.0f;
float target_y = 100.0f; // 預設停在前方半徑
float target_z = 150.0f; // 預設Z高度
bool ik_mode_enabled = false;

// 底部旋轉中心到馬達1的偏移距離 (單位: mm，若M1在中心軸上則為 0)
// 若有實體尺寸差異請根據硬體修改
const float BASE_R_OFFSET = 0.0f;

// PID 模式目標角度
float target_theta1_deg = 0.0f;
float target_theta2_deg = 0.0f; // Motor 2 (was 8-pin, now 13-pin)
float target_theta3_deg = 0.0f;

// PVT 緩衝區 (未來使用，目前先留著結構)
struct TrajectoryPoint {
    float theta1, theta2, z, duration;
};
std::queue<TrajectoryPoint> traj_buffer;


// ==========================================================
// 1. 初始化
// ==========================================================
extern "C" void Robot_Init(void) {
    // 呼叫 C 語言底層驅動初始化
    Motor_System_Config();

    // 重置 PID 狀態
    joint1_pid.reset();
    joint2_pid.reset();
    joint3_pid.reset();
    
    // 重置軌跡規劃器
    traj_joint1.reset();
    traj_joint2.reset();
    traj_joint3.reset();

    // 預設目標設為當前位置 (防止開機暴衝)
    // 注意：這裡假設開機時已經在某個合理位置，且已手動歸零
    // 如果沒有歸零，Encoder 值會是 0，IK 可能解不出來
    target_x = 0.0f;
    target_y = 150.0f;
}

// ==========================================================
// 2. 設定目標 API (給 main.c 測試用)
// ==========================================================
extern "C" void Robot_SetTargetPosition(float x, float y, float z) {
    target_x = x;
    target_y = y;
    target_z = z;
    ik_mode_enabled = true;
}

// ==========================================================
// 測試模式 API
// ==========================================================
extern "C" void Robot_SetTestMode(bool enable) {
    // 統一使用 test_mode_selection 作為唯一模式來源
    // false -> PID 模式 (0)
    // true  -> 測試模式 (1: 定速模式)
    test_mode_selection = enable ? 1 : 0;
    if (enable) {
        ik_mode_enabled = false;  // 測試模式下關閉運動學控制
    }
}

extern "C" void Robot_SetTestSpeed(int32_t rpm_motor1, int32_t rpm_motor2, int32_t rpm_motor3) {
    // 統一寫入 FreeRTOS 控制任務使用的目標變數
    test_target_rpm_m1 = rpm_motor1;
    test_target_rpm_m2 = rpm_motor2;
    test_target_rpm_m3 = rpm_motor3;
}

extern "C" void Robot_SetTargetAngles(float theta1_deg, float theta2_deg) {
    target_theta1_deg = theta1_deg;
    target_theta2_deg = theta2_deg;
}

extern "C" void Robot_SetTargetAngles3(float theta1, float theta2, float theta3) {
    target_theta1_deg = theta1;
    target_theta2_deg = theta2;
    target_theta3_deg = theta3;
}
extern "C" void Robot_Loop(float dt_seconds) {
    // --- 測試模式：直接控制馬達速度 (Legacy Mode 1/2/3) ---
    if (test_mode_selection > 0) {
        // ... (Legacy code omitted for brevity)
        return; 
    }
    
    // --- 步驟 A: 讀取感測器 (Feedback) ---
    Motor_Update(&motor_joint_13pin);
    Motor_Update(&motor_joint_13pin_2);
    Motor_Update(&motor_joint_13pin_3);

    // 取得真實角度 (Degree)
    real_theta1 = Motor_GetAngle(&motor_joint_13pin);
    real_theta2 = Motor_GetAngle(&motor_joint_13pin_2);
    real_theta3 = Motor_GetAngle(&motor_joint_13pin_3);

    // --- 步驟 B: 計算目標角度 (Setpoint) ---
    float target_angle1_deg = target_theta1_deg; 
    float target_angle2_deg = target_theta2_deg;
    float target_angle3_deg = target_theta3_deg;

    // 完整的 3-DOF 逆運動學 (XYZ 轉換到 M1, M2, M3 角度)
    if (ik_mode_enabled) {
        MotorAngles3D solution = kinematics.solveIK3D({target_x, target_y, target_z}, BASE_R_OFFSET);
        if (solution.is_reachable) {
            target_angle1_deg = FiveBarKinematics::rad2deg(solution.theta1);
            target_angle2_deg = FiveBarKinematics::rad2deg(solution.theta2);
            target_angle3_deg = FiveBarKinematics::rad2deg(solution.theta3);
        }
    }

    // --- 步驟 C: 安全檢查 (FK) --- 
    Point3D current_pos = kinematics.solveFK3D(
        FiveBarKinematics::deg2rad(real_theta1),
        FiveBarKinematics::deg2rad(real_theta2),
        FiveBarKinematics::deg2rad(real_theta3),
        BASE_R_OFFSET
    );
    // 邊界檢查: Z 低於 10 判定為觸底 (依據你的座標系，如果 Y 是高度需自行調整，這邊以 Z 為高度，水平面是 XY)
    if (current_pos.z < 10.0f && ik_mode_enabled) {
        Motor_Stop(&motor_joint_13pin);
        Motor_Stop(&motor_joint_13pin_2);
        Motor_Stop(&motor_joint_13pin_3);
        return; 
    }

    // --- 步驟 D: 軌跡規劃 (Trajectory Planning) ---
    traj_joint1.update(target_angle1_deg, dt_seconds);
    traj_joint2.update(target_angle2_deg, dt_seconds);
    traj_joint3.update(target_angle3_deg, dt_seconds);
    
    // 抓取軌跡狀態 (Pos, Vel, Acc)
    float traj_pos1 = traj_joint1.getPosition();
    float traj_vel1 = traj_joint1.getVelocity();
    float traj_acc1 = traj_joint1.getAcceleration();

    float traj_pos2 = traj_joint2.getPosition();
    float traj_vel2 = traj_joint2.getVelocity();
    float traj_acc2 = traj_joint2.getAcceleration();

    float traj_pos3 = traj_joint3.getPosition();
    float traj_vel3 = traj_joint3.getVelocity();
    float traj_acc3 = traj_joint3.getAcceleration();

    // --- 步驟 E: PID 計算 ---
    Motor_Start(&motor_joint_13pin);
    Motor_Start(&motor_joint_13pin_2);
    Motor_Start(&motor_joint_13pin_3);

    pid_output_rpm1 = joint1_pid.update(traj_pos1, traj_vel1, traj_acc1, real_theta1, dt_seconds);
    pid_output_rpm2 = joint2_pid.update(traj_pos2, traj_vel2, traj_acc2, real_theta2, dt_seconds);
    pid_output_rpm3 = joint3_pid.update(traj_pos3, traj_vel3, traj_acc3, real_theta3, dt_seconds);

    // --- 步驟 F: 輸出到底層 ---
    // [Fix] 反轉第一軸馬達輸出方向，解決 PID 正回授背向目標旋轉的問題
    Motor_SetSpeed(&motor_joint_13pin, (int32_t)-pid_output_rpm1);
    Motor_SetSpeed(&motor_joint_13pin_2, (int32_t)pid_output_rpm2);
    Motor_SetSpeed(&motor_joint_13pin_3, (int32_t)pid_output_rpm3);
}

// ==========================================================
// PID 控制迴圈（含前饋，不含運動學）
// ==========================================================
extern "C" void Robot_PID_Loop(float dt_seconds) {
    // 步驟 1: 讀取編碼器
    Motor_Update(&motor_joint_13pin);
    Motor_Update(&motor_joint_13pin_2);
    Motor_Update(&motor_joint_13pin_3);

    // 更新全域變數
    real_theta1 = Motor_GetAngle(&motor_joint_13pin);
    real_theta2 = Motor_GetAngle(&motor_joint_13pin_2);
    real_theta3 = Motor_GetAngle(&motor_joint_13pin_3);

    // 步驟 2: 軌跡規劃（計算速度與加速度前饋）
    traj_joint1.update(target_theta1_deg, dt_seconds);
    traj_joint2.update(target_theta2_deg, dt_seconds);
    traj_joint3.update(target_theta3_deg, dt_seconds);
    
    // [Modified] 抓取即時軌跡位置
    float traj_pos1 = traj_joint1.getPosition();
    float traj_vel1 = traj_joint1.getVelocity();
    float traj_acc1 = traj_joint1.getAcceleration();

    float traj_pos2 = traj_joint2.getPosition();
    float traj_vel2 = traj_joint2.getVelocity();
    float traj_acc2 = traj_joint2.getAcceleration();

    float traj_pos3 = traj_joint3.getPosition();
    float traj_vel3 = traj_joint3.getVelocity();
    float traj_acc3 = traj_joint3.getAcceleration();

    // 步驟 3: PID 計算（含前饋）
    Motor_Start(&motor_joint_13pin);
    Motor_Start(&motor_joint_13pin_2);
    Motor_Start(&motor_joint_13pin_3);

    // 計算並存入全域變數
    // [Modified] PID 目標改為軌跡位置
    pid_output_rpm1 = joint1_pid.update(traj_pos1, traj_vel1, traj_acc1, real_theta1, dt_seconds);
    pid_output_rpm2 = joint2_pid.update(traj_pos2, traj_vel2, traj_acc2, real_theta2, dt_seconds);
    pid_output_rpm3 = joint3_pid.update(traj_pos3, traj_vel3, traj_acc3, real_theta3, dt_seconds);

    // 步驟 4: 輸出到馬達
    // [Fix] 反轉第一軸馬達輸出方向，解決 PID 正回授背向目標旋轉的問題
    Motor_SetSpeed(&motor_joint_13pin, (int32_t)-pid_output_rpm1);
    Motor_SetSpeed(&motor_joint_13pin_2, (int32_t)pid_output_rpm2);
    Motor_SetSpeed(&motor_joint_13pin_3, (int32_t)pid_output_rpm3);
}
