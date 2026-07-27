#include "trajectory_planner.h"
#include <cmath>

TrapezoidalPlanner::TrapezoidalPlanner() {
    reset();
}

void TrapezoidalPlanner::reset() {
    _current_pos = 0.0f;
    _current_vel = 0.0f;
    _target_pos = 0.0f;
    _max_vel = 90.0f;  // deg/s = 0.25 rps (過減速機後的關節輸出軸)
    _max_acc = 180.0f; // deg/s^2 <-- 降低加速度 (溫和模式)，原 360
    _last_acc = 0.0f;
}

void TrapezoidalPlanner::setTarget(float start_pos, float target_pos, float max_vel, float max_acc) {
    _current_pos = start_pos;
    _target_pos = target_pos;
    _max_vel = std::fmax(std::fabs(max_vel), 0.001f);
    _max_acc = std::fmax(std::fabs(max_acc), 0.001f);
}

void TrapezoidalPlanner::update(float target_pos_in, float dt) {
    if (!std::isfinite(target_pos_in) || !std::isfinite(dt) || dt <= 0.0f) {
        _current_vel = 0.0f;
        _last_acc = 0.0f;
        return;
    }
    // 如果目標位置改變了 (且差異大於浮點數誤差)，則更新目標
    if (std::fabs(target_pos_in - _target_pos) > 0.001f) {
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
    float next_pos = _current_pos + _current_vel * dt;
    if ((dist_to_target > 0.0f && next_pos >= _target_pos) ||
        (dist_to_target < 0.0f && next_pos <= _target_pos)) {
        _current_pos = _target_pos;
        _current_vel = 0.0f;
        accel_cmd = 0.0f;
    } else {
        _current_pos = next_pos;
    }

    // 4. 到達目標後的收斂處理 (避免在目標點震盪)
    if (std::fabs(dist_to_target) < 0.1f && std::fabs(_current_vel) < 5.0f) {
        _current_pos = _target_pos;
        _current_vel = 0.0f;
        accel_cmd = 0.0f;
    }

    _last_acc = accel_cmd;
}

// 實體定義
TrapezoidalPlanner traj_joint1;
TrapezoidalPlanner traj_joint2;
TrapezoidalPlanner traj_joint3;
