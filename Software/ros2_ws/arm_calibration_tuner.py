#!/usr/bin/env python3
"""
Doghead Arm - 3-DOF (X, Y, Z + J1, J2, J3) 空間立體閉環控制與多軸獨立調參校準工具
---------------------------------------------------------------------------------
對應論文 2-DOF 3-Parallelogram PPM 機構 + 底座旋轉軸 (Joint 3 / Base):
- R1: 大臂長度 (140.0 mm)
- R2: 小臂長度 (140.0 mm)
- R3: 末端垂直偏移 (0.0 mm)
- J1_Offset, J2_Offset, J3_Offset 角度偏移補償

核心功能：
1. 【三軸同時作動 3D Benchmark】：大臂、小臂、底座在同一時間同時旋轉，模擬真實 3D 空間運動。
2. 【三軸獨立評分與調參】：50Hz 高速採集 J1、J2、J3 的各項量化指標，分別給出各軸得分與 3D 綜合分數。
3. 【單軸專項測試模式】：支援單獨測試 J1、J2 或 J3 的階躍響應。
4. 【3D 直角座標 Jog 微調】：支援鍵盤 w/s (前後X), a/d (左右Y), r/f (上下Z) 空間即時微調。
5. 【三軸獨立 PID/前饋在線注入】：即時動態下發各軸獨立的 Kp, Ki, Kd, Kv, Ka 參數至 STM32。
"""

import sys
import os
import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from doghead_interfaces.msg import (
    McuJointSetpoint,
    McuState,
    McuSessionControl,
    McuDiagnostics,
    McuMotorTestCommand
)

# 引用全專案唯一標準常數 (Single Source of Truth)
from robot_constants import (
    SAFETY_STATE_MAP,
    ROBOT_STATE_HOMING,
    ROBOT_STATE_READY,
    ROBOT_STATE_FAULT,
    CMD_AXIS_PID_ALL,
    CMD_AXIS_PID_J1,
    CMD_AXIS_PID_J2,
    CMD_AXIS_PID_J3,
    CMD_AXIS_FEEDFORWARD_ALL,
    CMD_AXIS_FEEDFORWARD_J1,
    CMD_AXIS_FEEDFORWARD_J2,
    CMD_AXIS_FEEDFORWARD_J3,
    CMD_AXIS_SYSTEM_HOMING,
    is_joint_target_valid
)

# 引用書法軌跡映射與規劃模組
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from trajectory_mapper import TrajectoryMapper
    from trajectory_planner import TrajectoryPlanner
except ImportError:
    TrajectoryMapper = None
    TrajectoryPlanner = None

FAULT_CODE_MAP = {
    0: "0 (NONE) [無故障]",
    1: "1 (HOMING_TIMEOUT) [歸零逾時]",
    2: "2 (HOMING_TRAVEL_LIMIT) [歸零行程超限]",
    3: "3 (POSITIVE_LIMIT) [正向限位觸發 ⚠️]",
    4: "4 (NEGATIVE_LIMIT) [反向限位觸發 ⚠️]",
    5: "5 (INVALID_COMMAND) [非法指令]",
    6: "6 (TRAJECTORY) [軌跡異常]",
    7: "7 (SOFTWARE_LIMIT) [軟體限位超限]",
    8: "8 (DEADLINE_MISS) [週期逾時]"
}

class TriAxisCalibrationTuner(Node):
    def __init__(self):
        super().__init__('triaxis_calibration_tuner')
        
        # 連桿幾何長度 (公尺)
        self.R1 = 0.1400  # 大臂長度 (14.00 cm)
        self.R2 = 0.1400  # 小臂長度 (14.00 cm)
        self.R3 = 0.0000  # 末端垂直偏移 (0.00 cm)
        
        # 兼容別名
        self.L1 = self.R1
        self.L2 = self.R2
        self.L3 = self.R3
        
        # 關節零點偏移補償 (度)
        self.j1_offset_deg = 0.0  # 大臂補償
        self.j2_offset_deg = 0.0  # 小臂補償
        self.j3_offset_deg = 0.0  # 底座補償
        
        # 速度上限設定 (預設: 15.0 deg/s ≈ 馬達 28.5 RPM)
        self.max_joint_vel_deg_s = 15.0 
        
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 直連 MCU 底層發佈器 (獨佔控制)
        self.mcu_setpoint_pub = self.create_publisher(
            McuJointSetpoint, '/doghead/mcu/joint_setpoint', 10
        )
        self.session_control_pub = self.create_publisher(
            McuSessionControl, '/doghead/mcu/session_control', 10
        )
        self.motor_test_pub = self.create_publisher(
            McuMotorTestCommand, '/doghead/mcu/motor_test_command', 10
        )
        
        # 訂閱器
        self.state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_mcu_state, qos
        )
        self.diag_sub = self.create_subscription(
            McuDiagnostics, '/doghead/mcu/diagnostics', self._on_mcu_diagnostics, 10
        )
        
        self.measured_q = [0.0, 0.0, 0.0]
        self.measured_vel = [0.0, 0.0, 0.0]
        self.commanded_rpm = [0.0, 0.0, 0.0]
        self.boot_id = 0
        self.session_epoch = 1
        self.session_id = 0
        self.is_armed = False
        self.has_feedback = False
        
        # MCU 狀態指標
        self.safety_state = 0
        self.control_mode = 0
        self.physical_enable_mask = 0
        self.active_fault = 0
        self.rejected_command_total = 0
        self.session_mismatch_total = 0
        self.invalid_command_total = 0
        self.command_timeout_total = 0
        self.stale_sequence_total = 0
        self.command_age_ms = 0
        
        self.is_executing = False
        self.traj_start_q = [0.0, 0.0, 0.0]
        self.traj_target_q = [0.0, 0.0, 0.0]
        self.traj_duration = 2.0
        self.traj_start_time = 0.0
        self.seq = 1

        # 三軸 PID Benchmark 數據採集
        self.is_recording_benchmark = False
        self.benchmark_samples = []
        self.benchmark_current_target_q = [0.0, 0.0, 0.0]
        self.benchmark_start_t = 0.0
        
        self.timer = self.create_timer(0.02, self._control_loop)

    def _on_mcu_state(self, msg: McuState):
        if len(msg.measured_position_rad) >= 3 and math.isfinite(msg.measured_position_rad[0]):
            self.measured_q = [float(v) for v in msg.measured_position_rad[:3]]
            if len(msg.measured_velocity_rad_s) >= 3:
                self.measured_vel = [float(v) for v in msg.measured_velocity_rad_s[:3]]
            if len(msg.commanded_motor_rpm) >= 3:
                self.commanded_rpm = [float(v) for v in msg.commanded_motor_rpm[:3]]
                
            self.boot_id = msg.boot_id
            self.safety_state = msg.safety_state
            self.control_mode = msg.control_mode
            self.physical_enable_mask = msg.physical_enable_mask
            self.active_fault = msg.fault_code
            self.rejected_command_total = msg.rejected_command_total
            self.command_age_ms = msg.command_age_ms
            self.has_feedback = True

            # 自動保持 Armed Session
            expected_session = ((self.boot_id & 0xFFFF) << 16) | (self.session_epoch & 0xFFFF)
            if self.boot_id != 0 and (not self.is_armed or (self.session_id & 0xFFFF0000) != (expected_session & 0xFFFF0000)):
                self.session_id = expected_session
                ctrl = McuSessionControl()
                ctrl.session_id = self.session_id
                ctrl.armed = True
                self.session_control_pub.publish(ctrl)
                self.is_armed = True

            # 50Hz 高速採集三軸取樣點
            if self.is_recording_benchmark:
                sample = {
                    't': time.time() - self.benchmark_start_t,
                    'q1_meas': math.degrees(self.measured_q[0]),
                    'q2_meas': math.degrees(self.measured_q[1]),
                    'q3_meas': math.degrees(self.measured_q[2]),
                    'v1_meas': math.degrees(self.measured_vel[0]),
                    'v2_meas': math.degrees(self.measured_vel[1]),
                    'v3_meas': math.degrees(self.measured_vel[2]),
                    'rpm1_cmd': self.commanded_rpm[0],
                    'rpm2_cmd': self.commanded_rpm[1],
                    'rpm3_cmd': self.commanded_rpm[2],
                    'q1_target': math.degrees(self.benchmark_current_target_q[0]),
                    'q2_target': math.degrees(self.benchmark_current_target_q[1]),
                    'q3_target': math.degrees(self.benchmark_current_target_q[2]),
                }
                self.benchmark_samples.append(sample)

    def _on_mcu_diagnostics(self, msg: McuDiagnostics):
        self.session_mismatch_total = msg.session_mismatch_total
        self.invalid_command_total = msg.invalid_command_total
        self.command_timeout_total = msg.command_timeout_total
        self.stale_sequence_total = msg.stale_sequence_total

    def arm_now(self, force_new: bool = False):
        if self.boot_id != 0:
            if force_new:
                self.session_epoch = (self.session_epoch + 1) & 0xFFFF
                if self.session_epoch == 0:
                    self.session_epoch = 1
            self.session_id = ((self.boot_id & 0xFFFF) << 16) | (self.session_epoch & 0xFFFF)
            ctrl = McuSessionControl()
            ctrl.session_id = self.session_id
            ctrl.armed = True
            self.session_control_pub.publish(ctrl)
            time.sleep(0.05)

    def clear_fault(self):
        print("\n>>> 正在向 STM32 發送故障清除指令 (Clear Fault & Re-Arm)...")
        self.arm_now(force_new=True)
        time.sleep(0.1)
        if self.safety_state != 4:
            print("✅ 故障已成功清除，系統恢復就緒狀態！\n")
        else:
            print("⚠️ 故障清除已送出，若開關仍處於硬體壓迫狀態請手動將手臂移開。\n")

    def calculate_3d_fk(self, q1_rad, q2_rad, q3_rad):
        """3-DOF 正運動學 (FK)：[q1 大臂, q2 小臂, q3 底座] -> 末端空間座標 (X, Y, Z) 公分"""
        eff_q1 = q1_rad + math.radians(self.j1_offset_deg)
        eff_q2 = q2_rad + math.radians(self.j2_offset_deg)
        eff_q3 = q3_rad + math.radians(self.j3_offset_deg)
        
        r_plane_m = self.R1 * math.cos(eff_q1) - self.R2 * math.cos(eff_q2)
        x_cm = (r_plane_m * math.cos(eff_q3)) * 100.0
        y_cm = (r_plane_m * math.sin(eff_q3)) * 100.0
        z_cm = (self.R1 * math.sin(eff_q1) - self.R2 * math.sin(eff_q2) - self.R3) * 100.0
        return x_cm, y_cm, z_cm

    def calculate_3d_ik(self, x_cm, y_cm, z_cm):
        """3-DOF 逆運動學 (IK)：目標空間座標 (X, Y, Z) 公分 -> 關節角 [q1, q2, q3] 弧度"""
        x_m = x_cm / 100.0
        y_m = y_cm / 100.0
        z_m = z_cm / 100.0
        
        # 1. 底座角度 q3
        q3_eff = math.atan2(y_m, x_m)
        
        # 2. 平面半徑 r
        r_plane = math.hypot(x_m, y_m)
        z_eff = z_m + self.R3
        D_sq = r_plane * r_plane + z_eff * z_eff
        D = math.sqrt(D_sq)
        
        max_reach = self.R1 + self.R2
        min_reach = abs(self.R1 - self.R2)
        if D > max_reach:
            return False, 0.0, 0.0, 0.0, (
                f"目標點 (X={x_cm:.2f}, Y={y_cm:.2f}, Z={z_cm:.2f})cm 距離原點 {D*100:.2f}cm，超出最大半徑 ({max_reach*100:.2f}cm)"
            )
        if D < min_reach:
            return False, 0.0, 0.0, 0.0, (
                f"目標點太近 (小於內部死區 {min_reach*100:.2f}cm)"
            )
            
        # 3. 求解大臂 q1 (Elbow-Up)
        cos_psi1 = (D_sq + self.R1**2 - self.R2**2) / (2.0 * self.R1 * D)
        cos_psi1 = max(-1.0, min(1.0, cos_psi1))
        psi1 = math.acos(cos_psi1)
        phi = math.atan2(z_eff, r_plane)
        
        q1_eff = phi + psi1
        
        # 4. 向量閉合求解小臂 q2
        dx2 = self.R1 * math.cos(q1_eff) - r_plane
        dy2 = self.R1 * math.sin(q1_eff) - z_eff
        q2_eff = math.atan2(dy2, dx2)
        if q2_eff < 0.0:
            q2_eff += 2.0 * math.pi
            
        # 扣除 offset
        q1 = q1_eff - math.radians(self.j1_offset_deg)
        q2 = q2_eff - math.radians(self.j2_offset_deg)
        q3 = q3_eff - math.radians(self.j3_offset_deg)
        
        return True, q1, q2, q3, "OK"

    def _control_loop(self):
        if not self.is_executing or not self.has_feedback:
            return
            
        elapsed = time.monotonic() - self.traj_start_time
        tau = min(1.0, max(0.0, elapsed / self.traj_duration))
        
        # 五次多項式平滑 S 曲線 (Quintic S-curve)
        s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
        s_dot = (30.0 * (tau ** 2) - 60.0 * (tau ** 3) + 30.0 * (tau ** 4)) / max(0.1, self.traj_duration)
        s_ddot = (60.0 * tau - 180.0 * (tau ** 2) + 120.0 * (tau ** 3)) / max(0.01, self.traj_duration ** 2)
        
        cmd_pos = [0.0, 0.0, 0.0]
        cmd_vel = [0.0, 0.0, 0.0]
        cmd_acc = [0.0, 0.0, 0.0]
        
        for i in range(3):
            delta = self.traj_target_q[i] - self.traj_start_q[i]
            cmd_pos[i] = self.traj_start_q[i] + delta * s
            cmd_vel[i] = delta * s_dot
            cmd_acc[i] = delta * s_ddot
            
        self.benchmark_current_target_q = list(cmd_pos)
        
        msg = McuJointSetpoint()
        msg.protocol_version = 1
        msg.session_id = self.session_id
        msg.sequence = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        msg.valid_for_ms = 100
        msg.enable_mask = 7 # 111b: J1, J2, J3 三軸同時致能
        msg.position_rad = [float(v) for v in cmd_pos]
        msg.velocity_rad_s = [float(v) for v in cmd_vel]
        msg.acceleration_rad_s2 = [float(v) for v in cmd_acc]
        
        self.mcu_setpoint_pub.publish(msg)
        
        if tau >= 1.0:
            self.is_executing = False

    def move_to_joints_rad(self, t_q1_rad, t_q2_rad, t_q3_rad, duration_sec=2.0):
        """三軸關節角度插補運動 (J1, J2, J3 在同一時間平滑同步運動)"""
        if not self.has_feedback:
            print("⚠️ 尚未接收到 MCU 回授狀態，無法下達移動指令！")
            return False
            
        t_q1_deg = math.degrees(t_q1_rad)
        t_q2_deg = math.degrees(t_q2_rad)
        t_q3_deg = math.degrees(t_q3_rad)
        
        # 安全邊界軟體檢查 (依據 robot_constants SSoT 定義)
        if not is_joint_target_valid(t_q1_deg, t_q2_deg, t_q3_deg):
            print(f"❌ 目標關節角度超出安全邊界！(J1={t_q1_deg:.1f}°, J2={t_q2_deg:.1f}°, J3={t_q3_deg:.1f}°)")
            return False
            
        self.arm_now()
        
        cur_q = list(self.measured_q)
        max_delta_deg = max(
            abs(math.degrees(t_q1_rad - cur_q[0])),
            abs(math.degrees(t_q2_rad - cur_q[1])),
            abs(math.degrees(t_q3_rad - cur_q[2]))
        )
        
        min_sec_needed = max_delta_deg / max(1.0, self.max_joint_vel_deg_s)
        actual_duration = max(duration_sec, min_sec_needed)
        
        self.traj_start_q = cur_q
        self.traj_target_q = [t_q1_rad, t_q2_rad, t_q3_rad]
        self.traj_duration = actual_duration
        self.traj_start_time = time.monotonic()
        self.traj_virtual_time = 0.0
        self.benchmark_current_target_q = None
        self.is_executing = True
        
        time.sleep(actual_duration + 0.1)
        return True

    def move_to_xyz(self, x_cm, y_cm, z_cm, duration_sec=2.0):
        """3D 直角座標平滑運動 (X, Y, Z -> 3軸同時同步運轉)"""
        ok, q1, q2, q3, err = self.calculate_3d_ik(x_cm, y_cm, z_cm)
        if not ok:
            print(f"❌ 3D 逆運動學失敗: {err}")
            return False
        return self.move_to_joints_rad(q1, q2, q3, duration_sec)

    def set_pid_gains(self, kp: float, ki: float, kd: float, axis: int = CMD_AXIS_PID_ALL):
        """線上動態設定 PID (axis: CMD_AXIS_PID_*)"""
        cmd = McuMotorTestCommand()
        cmd.protocol_version = 1
        cmd.session_id = self.session_id if self.session_id != 0 else 1
        cmd.sequence = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        cmd.valid_for_ms = 500
        cmd.axis = axis
        cmd.target_motor_rpm = float(kp)
        cmd.requested_accel_rpm_s = float(ki)
        cmd.requested_jerk_rpm_s2 = float(kd)
        cmd.enable = True
        self.motor_test_pub.publish(cmd)
        time.sleep(0.05)

    def set_feedforward_gains(self, kv: float, ka: float = 0.05, axis: int = CMD_AXIS_FEEDFORWARD_ALL):
        """線上動態設定前饋 (axis: CMD_AXIS_FEEDFORWARD_*)"""
        cmd = McuMotorTestCommand()
        cmd.protocol_version = 1
        cmd.session_id = self.session_id if self.session_id != 0 else 1
        cmd.sequence = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        cmd.valid_for_ms = 500
        cmd.axis = axis
        cmd.target_motor_rpm = float(kv)
        cmd.requested_accel_rpm_s = float(ka)
        cmd.enable = True
        self.motor_test_pub.publish(cmd)
        time.sleep(0.05)

    def trigger_hardware_homing(self, timeout_sec=40.0):
        """遠端觸發 STM32 硬體微動開關全自動歸零並等待完成回報 (SSoT State Machine)"""
        if not self.has_feedback:
            print("⚠️ 尚未接收到 MCU 回授狀態，無法發送歸零指令！")
            return False

        print("\n" + "=" * 76)
        print("  🔄 啟動 STM32 硬體微動開關自動歸零 (SSoT State Machine)")
        print("=" * 76)
        print(">>> [1/3] 發送遠端硬體歸零命令給 STM32 (CMD_AXIS_SYSTEM_HOMING)...")
        cmd = McuMotorTestCommand()
        cmd.protocol_version = 1
        cmd.session_id = self.session_id if self.session_id != 0 else 1
        cmd.sequence = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        cmd.valid_for_ms = 1000
        cmd.axis = CMD_AXIS_SYSTEM_HOMING
        cmd.enable = True
        self.motor_test_pub.publish(cmd)

        print(">>> [2/3] 等待 STM32 狀態機進入 ROBOT_STATE_HOMING...")
        start_wait = time.time()
        entered_homing = False
        while time.time() - start_wait < 3.0:
            if self.safety_state == ROBOT_STATE_HOMING:
                entered_homing = True
                break
            time.sleep(0.05)

        if not entered_homing and self.safety_state != ROBOT_STATE_HOMING:
            # 再次嘗試發送一次以防 Micro-ROS 封包丟失
            self.motor_test_pub.publish(cmd)

        print(">>> [3/3] STM32 正在執行硬體碰開關搜尋與待命角度回退，請稍候...")
        homing_start_t = time.time()
        while time.time() - homing_start_t < timeout_sec:
            if self.safety_state == ROBOT_STATE_READY:
                print("\n✅ STM32 回報：全流程硬體微動開關歸零已成功完成！系統就緒 (READY)！")
                print(f"   【當前就緒角度】: J1={math.degrees(self.measured_q[0]):.1f}°, J2={math.degrees(self.measured_q[1]):.1f}°, J3={math.degrees(self.measured_q[2]):.1f}°")
                return True
            elif self.safety_state == ROBOT_STATE_FAULT:
                print("\n❌ STM32 回報：歸零過程觸發故障保護 (FAULT)！")
                return False
            time.sleep(0.2)

        print("\n❌ 歸零逾時！未能於預定時間內收到 STM32 完成回報。")
        return False

    def evaluate_axis_metrics(self, q_meas, q_targets, v_meas, t_target_deg):
        """單軸指標評估演算法 (計算 Jitter, Wobble, MaxErr, SSErr, Overshoot, 得分)"""
        errs = [abs(m - tg) for m, tg in zip(q_meas, q_targets)]
        max_err = max(errs) if errs else 0.0
        ss_err = sum(errs[-15:]) / 15.0 if len(errs) >= 15 else (errs[-1] if errs else 0.0)
        
        avg_err = sum(errs) / len(errs) if errs else 0.0
        wobble = math.sqrt(sum((x - avg_err)**2 for x in errs) / len(errs)) if errs else 0.0
        
        dv = [v_meas[i] - v_meas[i-1] for i in range(1, len(v_meas))]
        jitter_rms = math.sqrt(sum(x*x for x in dv) / len(dv)) if dv else 0.0
        
        start_q = q_meas[0] if q_meas else 0.0
        total_delta = abs(t_target_deg - start_q)
        if total_delta > 1.0 and q_meas:
            if t_target_deg > start_q:
                overshoot = max(0.0, (max(q_meas) - t_target_deg) / total_delta * 100.0)
            else:
                overshoot = max(0.0, (t_target_deg - min(q_meas)) / total_delta * 100.0)
        else:
            overshoot = 0.0
            
        score = 100.0
        score -= min(45.0, max(0.0, jitter_rms - 1.0) * 5.0)  # 書法對高頻發抖極度敏感，加重扣分上限至45
        score -= min(15.0, wobble * 20.0)                      # 低頻起伏傷害較小，降至15
        score -= min(35.0, max_err * 3.5)                      # 輪廓變形傷害很大，加重上限至35
        score -= min(20.0, ss_err * 35.0)                      # 穩態誤差
        score -= min(15.0, overshoot * 1.5)                    # 超調
        score = max(0.0, min(100.0, score))
        
        return {
            "score": score,
            "jitter": jitter_rms,
            "wobble": wobble,
            "max_err": max_err,
            "ss_err": ss_err,
            "overshoot": overshoot
        }

    def run_motion_benchmark_3d(self, target_x_cm: float, target_y_cm: float, target_z_cm: float, duration_sec: float = 2.0):
        """【三軸同時作動】3D 空間立體運動 Benchmark 並產出三軸獨立評估報告"""
        if not self.has_feedback:
            print("❌ 尚未接收到 MCU 回授，無法執行 Benchmark！")
            return 0.0, {}
            
        cur_x, cur_y, cur_z = self.calculate_3d_fk(self.measured_q[0], self.measured_q[1], self.measured_q[2])
        ok, t_q1_rad, t_q2_rad, t_q3_rad, err = self.calculate_3d_ik(target_x_cm, target_y_cm, target_z_cm)
        if not ok:
            print(f"❌ 3D 目標位置逆解失敗: {err}")
            return 0.0, {}
            
        t_q1_deg = math.degrees(t_q1_rad)
        t_q2_deg = math.degrees(t_q2_rad)
        t_q3_deg = math.degrees(t_q3_rad)
        
        print("\n" + "=" * 76)
        print("  🚀 啟動 3D 空間立體三軸連動 Benchmark 採集 (50Hz 高速記錄)")
        print(f"  • 起點坐標 : (X={cur_x:5.2f}, Y={cur_y:5.2f}, Z={cur_z:5.2f}) cm")
        print(f"  • 終點坐標 : (X={target_x_cm:5.2f}, Y={target_y_cm:5.2f}, Z={target_z_cm:5.2f}) cm")
        print(f"  • 目標關節 : J1(大臂)={t_q1_deg:5.1f}°, J2(小臂)={t_q2_deg:5.1f}°, J3(底座)={t_q3_deg:5.1f}°")
        print("=" * 76)
        
        self.benchmark_samples.clear()
        self.benchmark_start_t = time.time()
        self.is_recording_benchmark = True
        
        ok_move = self.move_to_joints_rad(t_q1_rad, t_q2_rad, t_q3_rad, duration_sec=duration_sec)
        
        # 額外留 0.5 秒採集穩態
        time.sleep(0.5)
        self.is_recording_benchmark = False
        
        if not ok_move or len(self.benchmark_samples) < 20:
            print("❌ 採樣失敗或樣本數過少！")
            return 0.0, {}
            
        # 提取三軸數據
        q1_meas = [s['q1_meas'] for s in self.benchmark_samples]
        q1_tgts = [s['q1_target'] for s in self.benchmark_samples]
        v1_meas = [s['v1_meas'] for s in self.benchmark_samples]
        
        q2_meas = [s['q2_meas'] for s in self.benchmark_samples]
        q2_tgts = [s['q2_target'] for s in self.benchmark_samples]
        v2_meas = [s['v2_meas'] for s in self.benchmark_samples]
        
        q3_meas = [s['q3_meas'] for s in self.benchmark_samples]
        q3_tgts = [s['q3_target'] for s in self.benchmark_samples]
        v3_meas = [s['v3_meas'] for s in self.benchmark_samples]
        
        # 三軸分別獨立評估
        m1 = self.evaluate_axis_metrics(q1_meas, q1_tgts, v1_meas, t_q1_deg)
        m2 = self.evaluate_axis_metrics(q2_meas, q2_tgts, v2_meas, t_q2_deg)
        m3 = self.evaluate_axis_metrics(q3_meas, q3_tgts, v3_meas, t_q3_deg)
        
        # 3D 綜合評分 (三軸等權重平均)
        overall_score = (m1['score'] + m2['score'] + m3['score']) / 3.0
        
        print("\n" + "=" * 76)
        print("                  📊 三軸獨立運動性能量化評估報告")
        print("=" * 76)
        print(f"  • 採集總樣本數      : {len(self.benchmark_samples)} 筆 (歷時 {self.benchmark_samples[-1]['t']:.2f} 秒)")
        print("-" * 76)
        print(f"  【Joint 1 大臂】   : 得分 {m1['score']:5.1f} 分 | 抖動={m1['jitter']:4.2f}°/s², 穩態誤差={m1['ss_err']:4.2f}°, 最大跟隨={m1['max_err']:4.2f}°, 超調={m1['overshoot']:4.1f}%")
        print(f"  【Joint 2 小臂】   : 得分 {m2['score']:5.1f} 分 | 抖動={m2['jitter']:4.2f}°/s², 穩態誤差={m2['ss_err']:4.2f}°, 最大跟隨={m2['max_err']:4.2f}°, 超調={m2['overshoot']:4.1f}%")
        print(f"  【Joint 3 底座】   : 得分 {m3['score']:5.1f} 分 | 抖動={m3['jitter']:4.2f}°/s², 穩態誤差={m3['ss_err']:4.2f}°, 最大跟隨={m3['max_err']:4.2f}°, 超調={m3['overshoot']:4.1f}%")
        print("-" * 76)
        print(f"  🏆 3D 空間綜合性能總分 : 【 {overall_score:5.1f} / 100 分 】")
        print("=" * 76)
        
        # 保存 CSV
        try:
            import csv
            with open("triaxis_benchmark_latest.csv", 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'time_sec',
                    'q1_target', 'q1_meas', 'q1_err', 'v1_meas', 'rpm1_cmd',
                    'q2_target', 'q2_meas', 'q2_err', 'v2_meas', 'rpm2_cmd',
                    'q3_target', 'q3_meas', 'q3_err', 'v3_meas', 'rpm3_cmd',
                ])
                for s in self.benchmark_samples:
                    writer.writerow([
                        f"{s['t']:.3f}",
                        f"{s['q1_target']:.2f}", f"{s['q1_meas']:.2f}", f"{abs(s['q1_meas']-s['q1_target']):.2f}", f"{s['v1_meas']:.2f}", f"{s['rpm1_cmd']:.1f}",
                        f"{s['q2_target']:.2f}", f"{s['q2_meas']:.2f}", f"{abs(s['q2_meas']-s['q2_target']):.2f}", f"{s['v2_meas']:.2f}", f"{s['rpm2_cmd']:.1f}",
                        f"{s['q3_target']:.2f}", f"{s['q3_meas']:.2f}", f"{abs(s['q3_meas']-s['q3_target']):.2f}", f"{s['v3_meas']:.2f}", f"{s['rpm3_cmd']:.1f}",
                    ])
            print("  💾 完整 50Hz 3D 遙測響應數據已保存至: triaxis_benchmark_latest.csv\n")
        except Exception as e:
            print(f"  ⚠️ CSV 保存失敗: {e}\n")
            
        return overall_score, {"j1": m1, "j2": m2, "j3": m3}

    def run_triaxis_grid_search(self):
        """【三軸全自動網格搜索】自動循環測試多組「三軸各自獨立」的候選 PID 參數"""
        if not self.has_feedback:
            print("❌ 尚未接收到 MCU 回授，無法執行測試！")
            return
            
        # 3D 空間立體測試行程 (起點 -> 終點，三軸同時大幅動作，J3 擺動範圍約 19.7° ~ 56.3°，安全限制在 0°~90° 內)
        p1 = (14.0, 5.0, 3.5)   # X=14cm, Y=5cm, Z=3.5cm (q3 ≈ 19.7°)
        p2 = (10.0, 15.0, 3.5)  # X=10cm, Y=15cm, Z=3.5cm (q3 ≈ 56.3°)
        
        print(f"\n>>> 正在將手臂平滑移至 3D 空間基準起點 (X={p1[0]:.1f}, Y={p1[1]:.1f}, Z={p1[2]:.1f})cm...")
        self.move_to_xyz(p1[0], p1[1], p1[2], duration_sec=2.5)
        time.sleep(1.0)
        
        candidates = [
            {
                "name": "1. 柔和基準組 (75.7分冠軍基準: J1/J2超優, J3保守)",
                "j1": {"kp": 8.0, "ki": 0.30, "kd": 0.025, "kv": 2.0, "ka": 0.00},
                "j2": {"kp": 5.5, "ki": 0.20, "kd": 0.025, "kv": 2.0, "ka": 0.00},
                "j3": {"kp": 6.0, "ki": 0.20, "kd": 0.020, "kv": 1.8, "ka": 0.00},
            },
            {
                "name": "2. 底座提速追隨組 (保持J1/J2黃金參數, 單獨強化J3拉力消除滯後)",
                "j1": {"kp": 8.0, "ki": 0.30, "kd": 0.025, "kv": 2.0, "ka": 0.00},
                "j2": {"kp": 5.5, "ki": 0.20, "kd": 0.025, "kv": 2.0, "ka": 0.00},
                "j3": {"kp": 7.0, "ki": 0.22, "kd": 0.022, "kv": 1.95, "ka": 0.00},
            },
            {
                "name": "3. 三軸微剛性低誤差組 (三軸同步適度加強速度前饋, 壓制最大跟隨誤差<1°)",
                "j1": {"kp": 8.3, "ki": 0.32, "kd": 0.026, "kv": 2.05, "ka": 0.00},
                "j2": {"kp": 5.8, "ki": 0.22, "kd": 0.026, "kv": 2.05, "ka": 0.00},
                "j3": {"kp": 7.5, "ki": 0.25, "kd": 0.025, "kv": 2.05, "ka": 0.00},
            },
            {
                "name": "4. 高阻尼柔順無超調組 (適度提高微分阻尼 Kd, 確保 0 超調與零穩態誤差)",
                "j1": {"kp": 8.0, "ki": 0.28, "kd": 0.030, "kv": 2.0, "ka": 0.00},
                "j2": {"kp": 5.6, "ki": 0.20, "kd": 0.030, "kv": 2.0, "ka": 0.00},
                "j3": {"kp": 6.8, "ki": 0.20, "kd": 0.026, "kv": 1.90, "ka": 0.00},
            }
        ]
        
        print("\n" + "=" * 76)
        print("  🚀 啟動 3D 空間三軸同時作動 + 獨立 PID 自動網格搜尋 (Auto Grid Search)")
        print(f"  • 候選組數 : 共 {len(candidates)} 組三軸獨立配置")
        print(f"  • 測試行程 : ({p1[0]}, {p1[1]}, {p1[2]})cm <--> ({p2[0]}, {p2[1]}, {p2[2]})cm (三軸同時全力連動)")
        print("=" * 76)
        
        results = []
        for idx, cand in enumerate(candidates, 1):
            print(f"\n[{idx}/{len(candidates)}] 正在注入候選參數: {cand['name']}")
            # 獨立設定 J1
            self.set_pid_gains(cand['j1']['kp'], cand['j1']['ki'], cand['j1']['kd'], axis=11)
            self.set_feedforward_gains(cand['j1']['kv'], cand['j1']['ka'], axis=21)
            # 獨立設定 J2
            self.set_pid_gains(cand['j2']['kp'], cand['j2']['ki'], cand['j2']['kd'], axis=12)
            self.set_feedforward_gains(cand['j2']['kv'], cand['j2']['ka'], axis=22)
            # 獨立設定 J3
            self.set_pid_gains(cand['j3']['kp'], cand['j3']['ki'], cand['j3']['kd'], axis=13)
            self.set_feedforward_gains(cand['j3']['kv'], cand['j3']['ka'], axis=23)
            
            time.sleep(0.3)
            
            # 去程
            score_fwd, m_fwd = self.run_motion_benchmark_3d(p2[0], p2[1], p2[2], duration_sec=2.0)
            time.sleep(0.5)
            # 回程
            score_rev, m_rev = self.run_motion_benchmark_3d(p1[0], p1[1], p1[2], duration_sec=2.0)
            time.sleep(0.5)
            
            j1_fwd = m_fwd.get('j1', {}).get('score', 0.0) if m_fwd else 0.0
            j2_fwd = m_fwd.get('j2', {}).get('score', 0.0) if m_fwd else 0.0
            j3_fwd = m_fwd.get('j3', {}).get('score', 0.0) if m_fwd else 0.0
            
            j1_rev = m_rev.get('j1', {}).get('score', 0.0) if m_rev else 0.0
            j2_rev = m_rev.get('j2', {}).get('score', 0.0) if m_rev else 0.0
            j3_rev = m_rev.get('j3', {}).get('score', 0.0) if m_rev else 0.0

            avg_score = (score_fwd + score_rev) / 2.0
            results.append({
                "cand": cand,
                "score": avg_score,
                "j1_score": (j1_fwd + j1_rev) / 2.0,
                "j2_score": (j2_fwd + j2_rev) / 2.0,
                "j3_score": (j3_fwd + j3_rev) / 2.0,
            })
            
        if not results:
            print("❌ 無有效測試結果！")
            return
            
        results.sort(key=lambda x: x['score'], reverse=True)
        
        print("\n" + "=" * 76)
        print("                🏆 3D 三軸獨立調參自動網格搜索排行榜")
        print("=" * 76)
        for rank, res in enumerate(results, 1):
            c = res['cand']
            print(f"  No.{rank} 【總分: {res['score']:5.1f} 分】: {c['name']}")
            print(f"       ├─ J1(大臂): {res['j1_score']:5.1f}分 (Kp={c['j1']['kp']}, Ki={c['j1']['ki']}, Kv={c['j1']['kv']}, Ka={c['j1']['ka']})")
            print(f"       ├─ J2(小臂): {res['j2_score']:5.1f}分 (Kp={c['j2']['kp']}, Ki={c['j2']['ki']}, Kv={c['j2']['kv']}, Ka={c['j2']['ka']})")
            print(f"       └─ J3(底座): {res['j3_score']:5.1f}分 (Kp={c['j3']['kp']}, Ki={c['j3']['ki']}, Kv={c['j3']['kv']}, Ka={c['j3']['ka']})")
        print("=" * 76 + "\n")
        
        best = results[0]['cand']
        print(f"🌟 自動將 STM32 設定為最佳冠軍組: {best['name']}")
        self.set_pid_gains(best['j1']['kp'], best['j1']['ki'], best['j1']['kd'], axis=11)
        self.set_feedforward_gains(best['j1']['kv'], best['j1']['ka'], axis=21)
        self.set_pid_gains(best['j2']['kp'], best['j2']['ki'], best['j2']['kd'], axis=12)
        self.set_feedforward_gains(best['j2']['kv'], best['j2']['ka'], axis=22)
        self.set_pid_gains(best['j3']['kp'], best['j3']['ki'], best['j3']['kd'], axis=13)
        self.set_feedforward_gains(best['j3']['kv'], best['j3']['ka'], axis=23)

    def stop_arm(self):
        msg = McuJointSetpoint()
        msg.protocol_version = 1
        msg.session_id = self.session_id
        msg.sequence = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        msg.valid_for_ms = 100
        msg.enable_mask = 0
        msg.position_rad = [float(v) for v in self.measured_q]
        msg.velocity_rad_s = [0.0, 0.0, 0.0]
        msg.acceleration_rad_s2 = [0.0, 0.0, 0.0]
        self.mcu_setpoint_pub.publish(msg)
        self.is_executing = False

    def execute_calligraphy_json(self, json_path=None):
        """載入 robot_path.json 並執行 50Hz 書法筆劃軌跡"""
        if not self.has_feedback:
            print("⚠️ 尚未收到 MCU 回授，無法執行！")
            return
            
        if TrajectoryMapper is None or TrajectoryPlanner is None:
            print("❌ 找不到 TrajectoryMapper 或 TrajectoryPlanner 模組！")
            return
            
        cur_dir = os.path.dirname(os.path.abspath(__file__))
        if not json_path:
            candidates = [
                os.path.join(cur_dir, "robot_path.json"),
                os.path.join(os.path.dirname(cur_dir), "robot_path.json"),
                r"C:\Users\jacks\Documents\project\GraduationProject\BBSMG_ws\samples\waypoint_path\robot_path.json"
            ]
            for p in candidates:
                if os.path.exists(p):
                    json_path = p
                    break
                    
        if not json_path or not os.path.exists(json_path):
            print(f"❌ 找不到書法筆劃檔案: {json_path}")
            return
            
        print(f"\n[載入] 正在讀取 JSON: {os.path.basename(json_path)}")
        mapper = TrajectoryMapper(z_write_cm=3.5, z_travel_cm=8.0, margin_cm=1.5)
        raw_points = mapper.map_trajectory(json_path)
        
        print("-" * 76)
        print("  [1] 🌟 完整書寫整顆「永」字 (全 5 筆劃連貫書寫，含自動提筆與下筆，約 26 秒) [預設]")
        print("  [2] 🎯 僅測試第一筆劃 (最頂部的「點」，快速驗證響應，約 1 秒)")
        print("-" * 76)
        mode_choice = input("請選擇書寫模式 [1/2, 預設 1]: ").strip()
        
        planner = TrajectoryPlanner(dt=0.02)
        if mode_choice == '2':
            write_segment = []
            for pt in raw_points:
                if pt['type'] == 'write':
                    write_segment.append(pt)
                elif len(write_segment) > 0:
                    break
            print("\n正在規劃第一筆劃 PVA 軌跡...")
            pva_50hz = planner.plan_segment(write_segment, vmax=3.0, amax=12.0, kin_solver=mapper.kin)
            mode_desc = "第一筆劃 (單筆調適)"
        else:
            print("\n正在進行整顆「永」字全路徑 50Hz PVA 軌跡規劃 (5筆劃連貫+自動提按筆)...")
            pva_50hz = planner.plan_full_trajectory(raw_points, kin_solver=mapper.kin, v_write=3.0, v_travel=5.0, a_write=12.0, a_travel=15.0)
            mode_desc = "完整「永」字 (全 5 筆劃連貫書寫)"
            
        if not pva_50hz:
            print("❌ 規劃失敗或筆劃過短。")
            return
            
        print(f"✅ 規劃完成！【{mode_desc}】共 {len(pva_50hz)} 個時間序列點 (50Hz)，預計耗時 {pva_50hz[-1]['t']:.2f} 秒。")
        confirm = input("\n👉 請確認周圍安全，按 Enter 鍵啟動手臂並執行 (輸入 q 取消): ").strip()
        if confirm.lower() == 'q':
            print("已取消執行。")
            return
            
        start_pt = pva_50hz[0]
        print("\n[執行] 1. 平滑移動至筆劃起點...")
        ok = self.move_to_joints_rad(start_pt['q1'], start_pt['q2'], start_pt['q3'], duration_sec=2.5)
        if not ok:
            print("❌ 移動至起點失敗！")
            return
            
        time.sleep(0.5)
        print(f"[執行] 2. 開始 50Hz PVA 書法筆劃執行 (共 {len(pva_50hz)} 點)...")
        self.arm_now()
        
        recordings = []
        for pt in pva_50hz:
        idx = 0
        while idx < len(pva_50hz):
            loop_start = time.monotonic()
            pt = pva_50hz[idx]
            
            # Closed-Loop Clock Sync: 檢查跟隨誤差
            max_err_rad = 0.0
            if recordings: # 第一個點直接發送
                max_err_rad = max(
                    abs(pt['q1'] - self.measured_q[0]),
                    abs(pt['q2'] - self.measured_q[1]),
                    abs(pt['q3'] - self.measured_q[2])
                )
                
            # 如果落後超過 3.0 度，就原地等待 (不增加 idx，但仍發送當前舊點以維持心跳)
            if max_err_rad > math.radians(3.0):
                # 誤差過大，原地等待手臂跟上
                pass 
            else:
                # 順利跟上，推進時間軸
                idx += 1

            msg = McuJointSetpoint()
            msg.protocol_version = 1
            msg.session_id = self.session_id
            msg.sequence = self.seq
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            msg.valid_for_ms = 100
            msg.enable_mask = 7
            msg.position_rad = [float(pt['q1']), float(pt['q2']), float(pt['q3'])]
            msg.velocity_rad_s = [float(pt['v_q1']), float(pt['v_q2']), float(pt['v_q3'])]
            msg.acceleration_rad_s2 = [float(pt['a_q1']), float(pt['a_q2']), float(pt['a_q3'])]
            self.mcu_setpoint_pub.publish(msg)
            
            recordings.append({
                't': pt['t'],
                'target_q': [pt['q1'], pt['q2'], pt['q3']],
                'meas_q': list(self.measured_q),
                'target_v': [pt['v_q1'], pt['v_q2'], pt['v_q3']],
                'meas_v': list(self.measured_vel)
            })
            
            elapsed = time.monotonic() - loop_start
            sleep_time = 0.02 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
        print("✅ 筆劃執行完成！正在繪製 3D 疊加調適圖表...")
        
        # 繪圖並儲存
        import matplotlib
        if not os.environ.get('DISPLAY'):
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        
        t = [r['t'] for r in recordings]
        fig, axs = plt.subplots(3, 2, figsize=(15, 10), sharex=True)
        fig.suptitle("Calligraphy Stroke Tuning: Target vs Measured (50Hz PVA)", fontsize=16)
        
        names = ["J1 (Shoulder)", "J2 (Elbow)", "J3 (Base)"]
        for i in range(3):
            t_q = [math.degrees(r['target_q'][i]) for r in recordings]
            m_q = [math.degrees(r['meas_q'][i]) for r in recordings]
            t_v = [math.degrees(r['target_v'][i]) for r in recordings]
            m_v = [math.degrees(r['meas_v'][i]) for r in recordings]
            
            axs[i, 0].plot(t, t_q, 'r--', label=f'Target {names[i]} (deg)', linewidth=2)
            axs[i, 0].plot(t, m_q, 'b-', label=f'Measured {names[i]} (deg)', linewidth=1.5)
            axs[i, 0].set_title(f"{names[i]} Position")
            axs[i, 0].grid(True)
            axs[i, 0].legend()
            
            axs[i, 1].plot(t, t_v, 'r--', label=f'Target V{i+1} (deg/s)', linewidth=2)
            axs[i, 1].plot(t, m_v, 'b-', label=f'Measured V{i+1} (deg/s)', linewidth=1.5)
            axs[i, 1].set_title(f"{names[i]} Velocity (Tracking)")
            axs[i, 1].grid(True)
            axs[i, 1].legend()
            
        axs[2, 0].set_xlabel('Time (s)')
        axs[2, 1].set_xlabel('Time (s)')
        plt.tight_layout()
        save_path = os.path.join(cur_dir, "calligraphy_stroke_overlay.png")
        plt.savefig(save_path, dpi=120)
        print(f"📊 追蹤評估圖表已儲存至: {save_path}\n")



def main():
    rclpy.init()
    node = TriAxisCalibrationTuner()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print("\n" + "=" * 76)
    print("  🦾 Doghead Arm - 3-DOF 空間立體閉環控制與三軸獨立調參系統")
    print("=" * 76)
    print("正在連接 STM32 micro-ROS Agent 並等待回授 (50Hz)...")
    
    for _ in range(30):
        if node.has_feedback:
            break
        time.sleep(0.1)
        
    if not node.has_feedback:
        print("⚠️ 尚未收到 MCU 回授！請確認 win_vcp_bridge.py 與 micro-ROS Agent 是否運行。")
    else:
        print("🟢 成功連接 STM32 晶片！")
        
    try:
        while rclpy.ok():
            cur_x, cur_y, cur_z = node.calculate_3d_fk(node.measured_q[0], node.measured_q[1], node.measured_q[2])
            j1_deg = math.degrees(node.measured_q[0])
            j2_deg = math.degrees(node.measured_q[1])
            j3_deg = math.degrees(node.measured_q[2])
            
            print("\n" + "=" * 76)
            print("                       🎛️  3-DOF 主控制選單")
            print("=" * 76)
            print(f"  【當前末端 3D 坐標】: X(前後)={cur_x:5.2f} cm, Y(左右)={cur_y:5.2f} cm, Z(上下)={cur_z:5.2f} cm")
            print(f"  【當前關節 3D 角度】: J1(大臂)={j1_deg:5.1f}°, J2(小臂)={j2_deg:5.1f}°, J3(底座)={j3_deg:5.1f}°")
            print(f"  【安全與硬體狀態】  : {SAFETY_STATE_MAP.get(node.safety_state, '未知')} | 遮罩={node.physical_enable_mask}")
            print("-" * 76)
            print("  [0] 🚀 【三軸同時作動】3D 空間立體往復運動 Benchmark (自動產出三軸成績單)")
            print("  [1] 🌟 【三軸自動網格搜索】Auto Grid Search (四組三軸獨立候選參數衝刺高分)")
            print("  [2] 🕹️  【3D 直角座標 Jog 鍵盤微調模式】(w/s 前後, a/d 左右, r/f 上下)")
            print("  [3] 🎯 【單軸獨立專項步階測試】(可自選單獨測試 J1、J2 或 J3)")
            print("  [4] ⚙️  【三軸獨立 PID / 前饋參數手動注入】(即時設定 STM32 內部數值)")
            print("  [5] 📐 【校準零點 Offset 補償】(J1, J2, J3 角度微調補償)")
            print("  [6] 🔄 【遠端觸發硬體全自動歸零】(STM32 微動開關 SSoT 狀態機)")
            print("  [7] 🧹 【清除故障狀態】(Clear Fault & Re-Arm)")
            print("  [8] 🖌️  【執行書法 JSON 筆劃軌跡】(載入 robot_path.json 執行 50Hz PVA 與圖表繪製)")
            print("  [q] 退出系統")
            print("=" * 76)
            
            choice = input("請選擇功能項目: ").strip()
            
            if choice == '0':
                tx = float(input(f"請輸入目標 X (cm, 目前 {cur_x:.1f}): ").strip() or cur_x)
                ty = float(input(f"請輸入目標 Y (cm, 目前 {cur_y:.1f}): ").strip() or cur_y)
                tz = float(input(f"請輸入目標 Z (cm, 目前 {cur_z:.1f}): ").strip() or cur_z)
                node.run_motion_benchmark_3d(tx, ty, tz, duration_sec=2.0)
                input("按 Enter 鍵返回主選單...")
                
            elif choice == '1':
                node.run_triaxis_grid_search()
                input("按 Enter 鍵返回主選單...")
                
            elif choice == '2':
                print("\n" + "=" * 76)
                print("  🕹️ 3D 直角座標鍵盤 Jog 模式 (距離單位: 公分 cm，預設 1.0 cm)")
                print("=" * 76)
                print("  • 前後 X : 'w' (+X 往前) | 's' (-X 往後)")
                print("  • 左右 Y : 'a' (+Y 往左) | 'd' (-Y 往右)")
                print("  • 上下 Z : 'r' (+Z 往上) | 'f' (-Z 往下)")
                print("  • 絕對坐標移動 : 'goto X Y Z' (例如 'goto 14 0 5')")
                print("  • 返回主選單   : 'b' 或 'q'")
                print("-" * 76)
                
                while True:
                    cx, cy, cz = node.calculate_3d_fk(node.measured_q[0], node.measured_q[1], node.measured_q[2])
                    print(f"\r[3D末端] X={cx:5.2f} cm, Y={cy:5.2f} cm, Z={cz:5.2f} cm | 指令 > ", end="", flush=True)
                    cmd = input().strip().lower()
                    if cmd in ('b', 'q', 'exit'):
                        break
                    if not cmd:
                        continue
                    tokens = cmd.split()
                    act = tokens[0]
                    step = float(tokens[1]) if len(tokens) > 1 and tokens[1].replace('.','',1).isdigit() else 1.0
                    
                    tx, ty, tz = cx, cy, cz
                    if act in ('w', '+x'): tx += step
                    elif act in ('s', '-x'): tx -= step
                    elif act in ('a', '+y'): ty += step
                    elif act in ('d', '-y'): ty -= step
                    elif act in ('r', '+z'): tz += step
                    elif act in ('f', '-z'): tz -= step
                    elif act == 'goto' and len(tokens) >= 4:
                        tx, ty, tz = float(tokens[1]), float(tokens[2]), float(tokens[3])
                    else:
                        print("\n未知指令。支援: w, s, a, d, r, f, goto X Y Z, b")
                        continue
                        
                    node.move_to_xyz(tx, ty, tz, duration_sec=1.0)
                    
            elif choice == '3':
                ax_choice = input("請選擇要單獨測試的軸 [1=J1 大臂, 2=J2 小臂, 3=J3 底座]: ").strip()
                deg_step = float(input("請輸入階躍角度變化量 (度，例如 +10.0 或 -5.0): ").strip() or 10.0)
                cur_j = [math.degrees(v) for v in node.measured_q]
                if ax_choice == '1': cur_j[0] += deg_step
                elif ax_choice == '2': cur_j[1] += deg_step
                elif ax_choice == '3': cur_j[2] += deg_step
                node.move_to_joints_rad(math.radians(cur_j[0]), math.radians(cur_j[1]), math.radians(cur_j[2]), duration_sec=1.5)
                input("測試完成，按 Enter 鍵返回...")
                
            elif choice == '4':
                ax_in = input("請選擇要設定的軸 [10=三軸全部, 11=J1大臂, 12=J2小臂, 13=J3底座]: ").strip()
                kp_in = float(input("請輸入 Kp: ").strip())
                ki_in = float(input("請輸入 Ki: ").strip())
                kd_in = float(input("請輸入 Kd: ").strip())
                kv_in = float(input("請輸入 Kv (速度前饋, 預設 2.0): ").strip() or 2.0)
                ka_in = float(input("請輸入 Ka (加速度前饋, 預設 0.03): ").strip() or 0.03)
                
                axis_val = int(ax_in) if ax_in in ('10', '11', '12', '13') else 10
                node.set_pid_gains(kp_in, ki_in, kd_in, axis=axis_val)
                ff_axis = 20 if axis_val == 10 else (axis_val + 10)
                node.set_feedforward_gains(kv_in, ka_in, axis=ff_axis)
                print("✅ 參數已即時注入 STM32！")
                
            elif choice == '5':
                print(f"目前補償: J1={node.j1_offset_deg:+.2f}°, J2={node.j2_offset_deg:+.2f}°, J3={node.j3_offset_deg:+.2f}°")
                ax = input("選擇要補償的軸 [1/2/3]: ").strip()
                val = float(input("輸入補償角度變化量 (度): ").strip())
                if ax == '1': node.j1_offset_deg += val
                elif ax == '2': node.j2_offset_deg += val
                elif ax == '3': node.j3_offset_deg += val
                print("✅ 補償已更新！")
                
            elif choice == '6':
                node.trigger_hardware_homing()
                
            elif choice == '7':
                node.clear_fault()
                
            elif choice == '8':
                node.execute_calligraphy_json()
                input("按 Enter 鍵返回主選單...")
                
            elif choice.lower() == 'q':
                break
                
    except KeyboardInterrupt:
        print("\n中斷控制...")
    finally:
        node.stop_arm()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
