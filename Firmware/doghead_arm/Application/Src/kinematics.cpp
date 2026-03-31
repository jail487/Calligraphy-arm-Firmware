/**
 * @file kinematics.cpp
 * @brief 五連桿運動學實作
 */

#include "kinematics.hpp"
#include <cmath>

// 輔助函式：限制 acos 輸入範圍，避免 NaN
static float clip(float n, float lower, float upper) {
    return std::fmax(lower, std::fmin(n, upper));
}

MotorAngles FiveBarKinematics::solveIK(Point2D P, int mode) {
    MotorAngles result;
    result.is_reachable = false;
    result.theta1 = 0;
    result.theta2 = 0;

    // --- 1. 計算左臂角度 (Theta 1) ---
    // 以左馬達 (0,0) 為原點，目標 P(x,y)
    float dist_L = std::sqrt(P.x * P.x + P.y * P.y);

    // 檢查是否超出範圍 (兩臂拉直或摺疊)
    if (dist_L > (L1 + L2) || dist_L < std::fabs(L1 - L2)) {
        return result; // Unreachable
    }

    // 餘弦定理求內角
    // L2^2 = L1^2 + dist^2 - 2*L1*dist*cos(beta)
    float alpha_L = std::atan2(P.y, P.x);
    float cos_beta_L = (L1 * L1 + dist_L * dist_L - L2 * L2) / (2 * L1 * dist_L);
    float beta_L = std::acos(clip(cos_beta_L, -1.0f, 1.0f));

    // mode 決定手肘是向左彎還是向右彎 (通常取 +)
    result.theta1 = alpha_L + (float)mode * beta_L;


    // --- 2. 計算右臂角度 (Theta 2) ---
    // 以右馬達 (D,0) 為原點，將 P 轉換到右馬達座標系 -> P'(x-D, y)
    float x_R = P.x - D;
    float y_R = P.y;
    float dist_R = std::sqrt(x_R * x_R + y_R * y_R);

    if (dist_R > (L1 + L2) || dist_R < std::fabs(L1 - L2)) {
        return result; // Unreachable
    }

    float alpha_R = std::atan2(y_R, x_R);
    float cos_beta_R = (L1 * L1 + dist_R * dist_R - L2 * L2) / (2 * L1 * dist_R);
    float beta_R = std::acos(clip(cos_beta_R, -1.0f, 1.0f));

    // 右臂的手肘方向通常與左臂相反 (對稱)，所以這裡是 alpha - beta
    // 但具體取決於你的 mode 定義，這裡假設 mode=1 是 "手肘皆向外"
    result.theta2 = alpha_R - (float)mode * beta_R;

    result.is_reachable = true;
    return result;
}

Point2D FiveBarKinematics::solveFK(float theta1, float theta2) {
    // 1. 算出兩個肘部 (Elbow) 座標
    float E1_x = L1 * std::cos(theta1);
    float E1_y = L1 * std::sin(theta1);

    float E2_x = D + L1 * std::cos(theta2);
    float E2_y = L1 * std::sin(theta2);

    // 2. 求兩個圓的交點 (以 E1, E2 為圓心，半徑皆為 L2)
    // 這是經典的雙圓交點問題
    float d2 = (E2_x - E1_x) * (E2_x - E1_x) + (E2_y - E1_y) * (E2_y - E1_y);
    float d = std::sqrt(d2);

    // 檢查是否有解
    if (d > (L2 + L2) || d == 0) {
        return {0, 0}; // 構型錯誤 (斷裂或重疊)
    }

    // 簡化的幾何解法
    // 找出兩圓連線的中點 M
    float a = (d2) / (2 * d); // 因為兩半徑相等 L2=L2，簡化公式
    float h = std::sqrt(std::fmax(0.0f, L2 * L2 - a * a));

    float x2 = E1_x + a * (E2_x - E1_x) / d;
    float y2 = E1_y + a * (E2_y - E1_y) / d;

    // 兩個交點，取決於手臂是向前伸還是向後
    // 書法機通常是向前伸 (Y > ElbowY)，這裡取其中一個解
    Point2D P;
    P.x = x2 - h * (E2_y - E1_y) / d;
    P.y = y2 + h * (E2_x - E1_x) / d;

    // 如果算出來 Y 是負的 (往後指)，可能要取另一個解 (+h 改 -h)
    if (P.y < 0) {
         P.x = x2 + h * (E2_y - E1_y) / d;
         P.y = y2 - h * (E2_x - E1_x) / d;
    }

    return P;
}

MotorAngles3D FiveBarKinematics::solveIK3D(Point3D target, float r_offset) {
    MotorAngles3D result;
    result.theta1 = 0;
    result.theta2 = 0;
    result.theta3 = 0;
    result.is_reachable = false;

    // 1. 處理底部旋轉 (Motor 3) 的角度
    // 當 X, Y 都為 0 時為奇異點 (Singularity)，此時 theta3 其實任意值都行，暫且維持 0 或上一狀態。
    if (std::fabs(target.x) < 0.001f && std::fabs(target.y) < 0.001f) {
        result.theta3 = 0.0f; // 奇異點，暫定指正前方
    } else {
        result.theta3 = std::atan2(target.y, target.x);
    }

    // 2. 將 3D 座標降維到五連桿所在平面
    // R 是末端在水平面的投影距離，r_offset 處理馬達1與旋轉中心的水平偏移。
    float R = std::sqrt(target.x * target.x + target.y * target.y);
    Point2D plane_target;
    plane_target.x = R - r_offset; // 平面上的 X
    plane_target.y = target.z;     // 平面上的 Y (高度)

    // 3. 呼叫 2D IK
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
    
    // 如果底層無解，直接回傳
    if (p2d.x == 0 && p2d.y == 0) return p3d;

    // 2. 平面座標還原為 3D
    float R = p2d.x + r_offset;
    p3d.x = R * std::cos(theta3);
    p3d.y = R * std::sin(theta3);
    p3d.z = p2d.y;

    return p3d;
}

