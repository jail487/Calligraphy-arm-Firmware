/**
 * @file kinematics.cpp
 * @brief 雙平行四邊形機構 (Double Parallelogram Mechanism) 運動學實作
 */

#include "kinematics.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 輔助函式：限制 acos 輸入範圍，避免 NaN
static float clip(float n, float lower, float upper) {
    return std::fmax(lower, std::fmin(n, upper));
}

MotorAngles FiveBarKinematics::solveIK(Point2D P, int mode) {
    MotorAngles result;
    result.is_reachable = false;
    result.theta1 = 0;
    result.theta2 = 0;

    // 雙平行四邊形機構的幾何等效於一條長度分別為 L1, L2 的串聯雙節機械臂
    // 以左馬達 (0,0) 為原點，目標 P(x,y)

    // 1. 計算原點到目標點的距離
    float dist = std::sqrt(P.x * P.x + P.y * P.y);

    // 2. 檢查是否超出工作範圍 (太遠摸不到，或太近折疊不起來)
    if (dist > (L1 + L2) || dist < std::fabs(L1 - L2)) {
        return result; // Unreachable
    }

    // 3. 基礎幾何角度計算
    // alpha: 原點到目標連線 與 X軸正向 的夾角
    float alpha = std::atan2(P.y, P.x);

    // 餘弦定理求大臂內角 (gamma1)
    float cos_gamma1 = (L1 * L1 + dist * dist - L2 * L2) / (2 * L1 * dist);
    float gamma1 = std::acos(clip(cos_gamma1, -1.0f, 1.0f));

    // 餘弦定理求小臂內角 (gamma2)
    float cos_gamma2 = (L2 * L2 + dist * dist - L1 * L1) / (2 * L2 * dist);
    float gamma2 = std::acos(clip(cos_gamma2, -1.0f, 1.0f));

    // 4. 計算兩顆馬達的最終角度
    // mode 決定手臂姿態 (1 或 -1，決定手肘是往外突還是往內折)
    result.theta1 = alpha + (float)mode * gamma1;
    result.theta2 = alpha - (float)mode * gamma2;

    // --- 軟體極限保護區 (請依照你實機的極限取消註解並修改數值) ---
    /*
    float min_theta1_rad = 80.0f * (M_PI / 180.0f);   // 假設左臂不能低於 80 度
    float max_theta1_rad = 180.0f * (M_PI / 180.0f);  // 假設左臂不能大於 180 度
    if (result.theta1 < min_theta1_rad || result.theta1 > max_theta1_rad) {
        // 如果算出來會撞機，直接判定為無法到達
        result.is_reachable = false;
        return result;
    }
    */
    // -----------------------------------------------------------------

    result.is_reachable = true;
    return result;
}

Point2D FiveBarKinematics::solveFK(float theta1, float theta2) {
    Point2D P;

    // 雙平行四邊形的正運動學：向量相加
    P.x = L1 * std::cos(theta1) + L2 * std::cos(theta2);
    P.y = L1 * std::sin(theta1) + L2 * std::sin(theta2);

    return P;
}

MotorAngles3D FiveBarKinematics::solveIK3D(Point3D target, float r_offset, float hint_theta1_deg, float hint_theta2_deg) {
    (void)hint_theta1_deg;
    (void)hint_theta2_deg;

    MotorAngles3D result;
    result.theta1 = 0;
    result.theta2 = 0;
    result.theta3 = 0;
    result.is_reachable = false;

    // 1. 處理底部旋轉 (Motor 3) 的角度
    if (std::fabs(target.x) < 0.001f && std::fabs(target.y) < 0.001f) {
        result.theta3 = 0.0f; // 奇異點，暫定指正前方
    } else {
        result.theta3 = std::atan2(target.y, target.x);
    }

    // 2. 將 3D 座標降維到平面
    float R = std::sqrt(target.x * target.x + target.y * target.y);
    Point2D plane_target;
    plane_target.x = R - r_offset;
    plane_target.y = target.z;

    // 3. 呼叫新的 2D IK (mode = 1 代表手肘向上/向外)
    MotorAngles angles2d = solveIK(plane_target, 1);
    result.theta1 = angles2d.theta1;
    result.theta2 = angles2d.theta2;
    result.is_reachable = angles2d.is_reachable;

    return result;
}

Point3D FiveBarKinematics::solveFK3D(float theta1, float theta2, float theta3, float r_offset) {
    Point3D p3d = {0, 0, 0};

    // 1. 取得面內座標 (r, z)
    Point2D p2d = solveFK(theta1, theta2);

    if (p2d.x == 0 && p2d.y == 0) return p3d;

    // 2. 平面座標還原為 3D
    float R = p2d.x + r_offset;
    p3d.x = R * std::cos(theta3);
    p3d.y = R * std::sin(theta3);
    p3d.z = p2d.y;

    return p3d;
}

