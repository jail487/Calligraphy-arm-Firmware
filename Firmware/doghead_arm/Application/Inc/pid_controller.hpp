/**
 * @file pid_controller.hpp
 * @brief 支援前饋控制的 PID 控制器 (防飽和、防突刺、死區補償工業優化版)
 */
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cmath> // 需要用到 fabsf

class PositionController {
public:
    /**
     * @brief 建構子新增 max_integral 和 deadband 參數
     * @param kp 比例增益
     * @param ki 積分增益
     * @param kd 微分增益
     * @param kv 速度前饋增益 (Feedforward Velocity)
     * @param ka 加速度前饋增益 (Feedforward Acceleration)
     * @param max_rpm 最大輸出轉速限制 (RPM)
     * @param max_i 積分上限 (預設 1000.0f，防止積分飽和)
     * @param deadband 死區補償 (預設 15.0f RPM，克服靜摩擦力)
     */
    PositionController(float kp, float ki, float kd, float kv, float ka, float max_rpm, float max_i = 1000.0f, float deadband = 15.0f)
        : _kp(kp), _ki(ki), _kd(kd), _kv(kv), _ka(ka), _max_output(max_rpm), _max_integral(max_i), _deadband_rpm(deadband) {
        reset(0.0f);
    }

    /**
     * @brief 重置控制器狀態
     * @param initial_pos 初始位置，傳入當下實際位置可避免啟動時產生微分突刺
     */
    void reset(float initial_pos = 0.0f) {
        _integral = 0.0f;
        _prev_current_pos = initial_pos; 
    }

    /**
     * @brief 更新控制器
     * @param target_pos 目標位置 (Degree)
     * @param target_vel 目標速度 (Degree/s) - 來自 PVT 或軌跡規劃器
     * @param target_acc 目標加速度 (Degree/s^2) - 來自 PVT 或軌跡規劃器
     * @param current_pos 實際位置 (Degree)
     * @param dt 時間差 (s)
     * @return 馬達轉速命令 (RPM)
     */
    float update(float target_pos, float target_vel, float target_acc, float current_pos, float dt) {
        // 防止除以零錯誤
        if (dt <= 0.0f) return 0.0f;

        // ==========================================================
        // 1. 回授控制 (Feedback): 處理位置誤差
        // ==========================================================
        float error = target_pos - current_pos;
        
        // P 項
        float p_out = _kp * error;
        
        // I 項 (🚨 優化 1: 積分抗飽和 Integral Anti-Windup)
        float candidate_integral = _integral + error * dt;
        if (candidate_integral > _max_integral) candidate_integral = _max_integral;
        if (candidate_integral < -_max_integral) candidate_integral = -_max_integral;
        float i_out = _ki * candidate_integral;
        
        // D 項 (🚨 優化 2: 對實測值微分 Derivative on Measurement)
        // 注意：這裡是 -(現在位置 - 過去位置) / dt，藉此消除目標跳變產生的突刺
        float derivative = -(current_pos - _prev_current_pos) / dt;
        float d_out = _kd * derivative;
        
        float feedback_rpm = p_out + i_out + d_out;

        // ==========================================================
        // 2. 前饋控制 (Feedforward): 預測並提供基礎動力
        // ==========================================================
        // 將角速度 (Deg/s) 轉換為 RPM: (Deg/s) / 360 * 60 = RPM
        float ff_vel_rpm = (target_vel / 360.0f) * 60.0f * _kv;
        // 加速度前饋 (補償慣性)
        float ff_acc_rpm = target_acc * _ka;

        // ==========================================================
        // 3. 總輸出計算與死區補償
        // ==========================================================
        float output = feedback_rpm + ff_vel_rpm + ff_acc_rpm;

        // 🚨 優化 3: 死區補償 (Deadband Compensation)，克服靜摩擦力
        // 只有在系統真正需要出力時才加上補償，避免在原地一直抖動
        if (output > 0.1f) {
            output += _deadband_rpm;
        } else if (output < -0.1f) {
            output -= _deadband_rpm;
        }

        // ==========================================================
        // 4. 最終輸出限幅 (Output Clamping)
        // ==========================================================
        // Do not integrate farther into saturation; this prevents a large
        // residual command when the target or safety state changes.
        bool saturated_high = output > _max_output;
        bool saturated_low = output < -_max_output;
        if ((!saturated_high || error < 0.0f) && (!saturated_low || error > 0.0f)) {
            _integral = candidate_integral;
        }
        if (output > _max_output) {
            output = _max_output;
        } else if (output < -_max_output) {
            output = -_max_output;
        }

        // 記錄這次的「實際位置」給下一次算微分用
        _prev_current_pos = current_pos;
        
        return output;
    }

private:
    // PID 參數
    float _kp, _ki, _kd;
    // 前饋參數
    float _kv, _ka;
    // 限制參數
    float _max_output;
    float _max_integral; // 積分天花板
    float _deadband_rpm; // 靜摩擦力補償 RPM
    
    // 狀態變數
    float _integral;
    float _prev_current_pos; // 記錄上一次的實際位置
};

#endif // PID_CONTROLLER_HPP
