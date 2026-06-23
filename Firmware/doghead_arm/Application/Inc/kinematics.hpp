/**
 * @file kinematics.hpp
 * @brief 平面五連桿機構 正逆運動學解算器
 */

#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <cmath>

struct Point2D {
    float x;
    float y;
};

// 3D 空間座標
struct Point3D {
    float x;
    float y;
    float z;
};

struct MotorAngles {
    float theta1; // 左馬達角度 (Rad)
    float theta2; // 右馬達角度 (Rad)
    bool is_reachable; // 是否在工作範圍內
};

// 3D 解算結果
struct MotorAngles3D {
    float theta1; 
    float theta2; 
    float theta3; // 底部旋轉角度 (Rad)
    bool is_reachable; 
};

class FiveBarKinematics {
public:
    /**
     * @brief 建構子
     * @param l1 主動臂長度 (連接馬達的短臂)
     * @param l2 從動臂長度 (連接末端的長臂)
     * @param d  兩馬達軸心間距
     */
    FiveBarKinematics(float l1, float l2, float d)
        : L1(l1), L2(l2), D(d) {}

    /**
     * @brief 逆向運動學 (IK): (x, y) -> (theta1, theta2)
     * @param target 末端座標
     * @param solution_mode 手肘模式 (1: 手肘向外/上, -1: 手肘向內/下) -> 書法機通常選 1
     * @return 計算出的角度
     */
    MotorAngles solveIK(Point2D target, int solution_mode = 1);

    /**
     * @brief 正向運動學 (FK): (theta1, theta2) -> (x, y)
     * @param angles 兩馬達角度 (Rad)
     * @return 末端座標 (若無解返回 0,0)
     */
    Point2D solveFK(float theta1, float theta2);

    /**
     * @brief 3D 逆向運動學 (IK): (x, y, z) -> (theta1, theta2, theta3)
     * 考慮底部旋轉關節加平面五連桿的結構
     * 此處預設五連桿平面的原點(Motor 1)在底座旋轉軸心上方，或自定義平移(offset)。
     */
    MotorAngles3D solveIK3D(Point3D target, float r_offset = 0.0f, float hint_theta1_deg = 0.0f, float hint_theta2_deg = 0.0f);

    /**
     * @brief 3D 正向運動學 (FK): (theta1, theta2, theta3) -> (x, y, z)
     */
    Point3D solveFK3D(float theta1, float theta2, float theta3, float r_offset = 0.0f);

    // 輔助：角度轉換
    static float deg2rad(float deg) { return deg * 0.0174532925f; }
    static float rad2deg(float rad) { return rad * 57.2957795f; }

private:
    float L1, L2, D;
};

#endif // KINEMATICS_HPP
