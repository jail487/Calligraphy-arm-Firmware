"""
robot_constants.py
Doghead Calligraphy Arm - Central Software Constants (Single Source of Truth)

This file mirrors the physical and communication definitions from Firmware:
- robot_config.h
- homing_calibration.h
- motor_parameters.h

All software nodes (calibration tuners, trajectory planners, jog scripts)
MUST import their constants from this module instead of hardcoding magic numbers.
"""

import math

# ==========================================================
# 1. 系統安全狀態碼 (Robot Safety State)
# ==========================================================
ROBOT_STATE_SAFE    = 0  # 安全待命 (致能切斷)
ROBOT_STATE_HOMING  = 1  # 實體微動開關自動歸零中
ROBOT_STATE_READY   = 2  # 系統就緒 (已歸零，隨時可運作)
ROBOT_STATE_RUNNING = 3  # 軌跡/運動執行中
ROBOT_STATE_FAULT   = 4  # 系統故障 (軟硬體限位/超時保護)
ROBOT_STATE_ESTOP   = 5  # 緊急停機 (E-Stop)

SAFETY_STATE_MAP = {
    ROBOT_STATE_SAFE:    "ROBOT_STATE_SAFE (安全待命)",
    ROBOT_STATE_HOMING:  "ROBOT_STATE_HOMING (實體微動開關歸零校正中)",
    ROBOT_STATE_READY:   "ROBOT_STATE_READY (就緒)",
    ROBOT_STATE_RUNNING: "ROBOT_STATE_RUNNING (運轉執行中)",
    ROBOT_STATE_FAULT:   "ROBOT_STATE_FAULT (系統故障 / 異常保護)",
    ROBOT_STATE_ESTOP:   "ROBOT_STATE_ESTOP (緊急停機)"
}

# ==========================================================
# 2. 遠端通訊控制通道 (McuMotorTestCommand.axis)
# ==========================================================
CMD_AXIS_MOTOR_ALL         = 0
CMD_AXIS_MOTOR_J1          = 1
CMD_AXIS_MOTOR_J2          = 2
CMD_AXIS_MOTOR_J3          = 3

CMD_AXIS_PID_BASE          = 10
CMD_AXIS_PID_ALL           = 10
CMD_AXIS_PID_J1            = 11
CMD_AXIS_PID_J2            = 12
CMD_AXIS_PID_J3            = 13

CMD_AXIS_FEEDFORWARD_BASE  = 20
CMD_AXIS_FEEDFORWARD_ALL   = 20
CMD_AXIS_FEEDFORWARD_J1    = 21
CMD_AXIS_FEEDFORWARD_J2    = 22
CMD_AXIS_FEEDFORWARD_J3    = 23

# 特殊系統級遠端硬體命令
CMD_AXIS_SYSTEM_HOMING     = 99  # 遠端觸發 STM32 硬體微動開關全自動歸零

# 關節致能遮罩 (Joint Enable Masks)
ENABLE_MASK_NONE           = 0x00
ENABLE_MASK_J1             = 0x01
ENABLE_MASK_J2             = 0x02
ENABLE_MASK_J3             = 0x04
ENABLE_MASK_ALL_JOINTS     = 0x07  # 0b111 (J1, J2, J3 全致能)

# ==========================================================
# 3. 關節軟體極限與安全裕度 (Joint Limits & Tolerances)
# ==========================================================
JOINT1_MIN_DEG = 0.0
JOINT1_MAX_DEG = 111.0

JOINT2_MIN_DEG = 105.0
JOINT2_MAX_DEG = 180.0

JOINT3_MIN_DEG = 0.0
JOINT3_MAX_DEG = 90.0

# 軟體安全檢查容許之動態微調公差 (deg)
JOINT_LIMIT_TOLERANCE_DEG = 5.0

# 判定關節角度是否合規之輔助函式
def is_joint_target_valid(q1_deg: float, q2_deg: float, q3_deg: float, tolerance: float = JOINT_LIMIT_TOLERANCE_DEG) -> bool:
    """檢查給定之關節角度是否位於帶有公差的安全極限範圍內"""
    q1_ok = (JOINT1_MIN_DEG - tolerance) <= q1_deg <= (JOINT1_MAX_DEG + tolerance)
    q2_ok = (JOINT2_MIN_DEG - tolerance) <= q2_deg <= (JOINT2_MAX_DEG + tolerance)
    q3_ok = (JOINT3_MIN_DEG - tolerance) <= q3_deg <= (JOINT3_MAX_DEG + tolerance)
    return q1_ok and q2_ok and q3_ok

# ==========================================================
# 4. 微動開關觸發角度與待命姿態 (Standby Pose)
# ==========================================================
HOME_SWITCH_THETA1_DEG  = 111.0
HOME_SWITCH_THETA2_DEG  = 180.0
HOME_SWITCH_THETA3_DEG  = 90.0

HOME_STANDBY_THETA1_DEG = 111.0
HOME_STANDBY_THETA2_DEG = 180.0
HOME_STANDBY_THETA3_DEG = 45.0   # 底座待命預設角度 (45.0° 正前方對稱軸，對齊 robot_config.h)

# ==========================================================
# 5. 機械手臂運動幾何參數 (Kinematic Geometry)
# ==========================================================
LINK_R1_M  = 0.1139 # 大臂長度 (m)
LINK_R2_M  = 0.1139 # 小臂長度 (m)
LINK_R3_M  = 0.0880 # 筆尖垂直下垂偏移 (m) [向下 88.0 mm]
TOOL_OFFSET_FWD_M = 0.0330 # 筆尖水平向前延伸偏移 (m) [向前 33.5 mm]

LINK_R1_CM = LINK_R1_M * 100.0
LINK_R2_CM = LINK_R2_M * 100.0
LINK_R3_CM = LINK_R3_M * 100.0
TOOL_OFFSET_FWD_CM = TOOL_OFFSET_FWD_M * 100.0

# ==========================================================
# 6. 馬達與傳動機構參數 (Actuator & Drivetrain)
# ==========================================================
GEAR_RATIO       = 114.0
ENCODER_PPR      = 100.0
PULSES_PER_REV   = 400.0 # 4x 正交解碼後的每圈脈衝數

# ==========================================================
# 7. 關節動態安全極限 (Joint Dynamic Safety Limits)
# ==========================================================
# 來自 Firmware/robot_config.h 的絕對極限
JOINT_PVA_MAX_VEL_RAD_S     = 4.593
JOINT_PVA_MAX_ACCEL_RAD_S2  = 15.0

# 軟體端軌跡產生器專用之安全上限 (保留裕度以防被韌體剔除)
SOFTWARE_SAFE_VEL_RAD_S     = 4.0
SOFTWARE_SAFE_ACCEL_RAD_S2  = 10.0

# ==========================================================
# 8. 最佳基準 PID 與速度前饋參數 (3D Benchmark Winner)
# ==========================================================
DEFAULT_PID_GAINS = {
    "j1": {"kp": 8.3, "ki": 0.32, "kd": 0.026, "kv": 1.20, "ka": 0.00},
    "j2": {"kp": 5.8, "ki": 0.22, "kd": 0.026, "kv": 1.20, "ka": 0.00},
    "j3": {"kp": 7.5, "ki": 0.25, "kd": 0.025, "kv": 1.00, "ka": 0.00},
}

