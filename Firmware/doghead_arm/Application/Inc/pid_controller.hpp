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
    PositionController(float kp, float ki, float kd, float kv, float ka, float max_rpm, float max_i = 100.0f, float deadband = 0.0f)
        : _kp(kp), _ki(ki), _kd(kd), _kv(kv), _ka(ka), _max_output(max_rpm), _max_integral(max_i), _deadband_rpm(deadband),
          _prev_derivative(0.0f) {
        reset(0.0f);
    }

    /**
     * @brief 重置控制器狀態
     * @param initial_pos 初始位置，傳入當下實際位置可避免啟動時產生微分突刺
     */
    void reset(float initial_pos = 0.0f) {
        _integral = 0.0f;
        _prev_current_pos = initial_pos; 
        _prev_derivative = 0.0f;
    }

    /**
     * @brief 線上動態更新 PID 與前饋參數 (免重新編譯燒錄)
     */
    void setGains(float kp, float ki, float kd, float kv = -1.0f, float ka = -1.0f) {
        if (kp >= 0.0f) _kp = kp;
        if (ki >= 0.0f) _ki = ki;
        if (kd >= 0.0f) _kd = kd;
        if (kv >= 0.0f) _kv = kv;
        if (ka >= 0.0f) _ka = ka;
    }
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

    /**
     * @brief 更新控制器
     * @param target_pos 目標位置 (Degree)
     * @param target_vel 目標速度 (Degree/s) - 來自 PVT 或軌跡規劃器
     * @param target_acc 目標加速度 (Degree/s^2) - 來自 PVT 或軌跡規劃器
     * @param current_pos 實際位置 (Degree)
     * @param dt 時間差 (s)
     * @return 關節轉速命令 (Joint-space output-shaft RPM; gear ratio is NOT pre-applied here)
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
        
        // I 項 (🚨 積分抗飽和 + 穩態防靜摩擦自適應加速)
        float integral_mult = 1.0f;
        if (fabsf(target_vel) < 0.1f && fabsf(error) < 1.0f && fabsf(error) > 0.02f) {
            integral_mult = 2.0f; // 穩態微小誤差時加倍積分效率，迅速消滅 0.1° 殘留
        }
        float candidate_integral = _integral + (error * integral_mult) * dt;
        if (candidate_integral > _max_integral) candidate_integral = _max_integral;
        if (candidate_integral < -_max_integral) candidate_integral = -_max_integral;
        float i_out = _ki * candidate_integral;
        
        // D 項 (🚨 實測值微分 + 25Hz 低通濾波徹底抑制編碼器量化階躍雜訊)
        float raw_derivative = -(current_pos - _prev_current_pos) / dt;
        const float alpha = 0.10f; // 平滑係數
        float filtered_derivative = (1.0f - alpha) * _prev_derivative + alpha * raw_derivative;
        _prev_derivative = filtered_derivative;
        float d_out = _kd * filtered_derivative;
        
        float feedback_rpm = p_out + i_out + d_out;

        // ==========================================================
        // 2. 前饋控制 (Feedforward): 預測並提供基礎動力
        // ==========================================================
        // 將角速度 (Deg/s) 轉換為關節 RPM: (Deg/s) / 360 * 60 = RPM
        float ff_vel_rpm = (target_vel / 360.0f) * 60.0f * _kv;
        // 加速度前饋 (補償慣性)
        float ff_acc_rpm = target_acc * _ka;

        // ==========================================================
        // 3. 總輸出計算 (已移除破壞平滑性的 1710 RPM 不連續死區跳躍)
        // ==========================================================
        float output = feedback_rpm + ff_vel_rpm + ff_acc_rpm;

        if (_deadband_rpm > 0.0f) {
            if (output > 0.1f) output += _deadband_rpm;
            else if (output < -0.1f) output -= _deadband_rpm;
        }

        // ==========================================================
        // 4. 最終輸出限幅 (Output Clamping)
        // ==========================================================
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
    float _prev_derivative;  // 記錄上一次濾波後的微分值
};

#endif // PID_CONTROLLER_HPP
