#!/usr/bin/env python3
"""
Doghead Arm - Kinematic Feedback Controller (上位機運動學閉環控制主控台)
-------------------------------------------------------------------------
Features:
1. Full Closed-loop Feedback:
   - Reads /doghead/mcu/state for real-time joint positions & velocities.
   - Computes Forward Kinematics (FK) to show real-time End-Effector (X, Y, Z).
2. Trajectory Generation:
   - Generates quintic (5th-order) smooth PVA trajectories from measured pose to target.
   - Prevents sudden velocity jerks.
3. Interactive Modes:
   - [1] Cartesian PTP Mode: Move to target (X, Y, Z) in cm/m.
   - [2] Joint PTP Mode: Move to target joint angles (J1, J2, J3) in degrees.
   - [3] Step Jog Mode: Interactive XYZ keyboard nudges (e.g. +1cm, -1cm).
"""

import sys
import os
import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from doghead_interfaces.msg import McuJointSetpoint, McuState
from std_srvs.srv import Trigger

from robot_constants import HOME_STANDBY_THETA1_DEG, HOME_STANDBY_THETA2_DEG, HOME_STANDBY_THETA3_DEG

# Import Kinematics Solver from doghead_motion package
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'doghead_motion'))
from doghead_motion.kinematics import DogheadKinematics, DEFAULT_LINK_LENGTHS, DEFAULT_JOINT_LIMITS
from doghead_motion.trajectory_generator import JointPvaTrajectoryGenerator

class KinematicFeedbackController(Node):
    def __init__(self):
        super().__init__('kinematic_feedback_controller')
        
        self.kinematics = DogheadKinematics(DEFAULT_LINK_LENGTHS, DEFAULT_JOINT_LIMITS)
        self.traj_gen = JointPvaTrajectoryGenerator()
        
        # QoS profiles
        best_effort_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.setpoint_pub = self.create_publisher(
            McuJointSetpoint, '/doghead/command/joint_setpoint', 10
        )
        self.state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_mcu_state, best_effort_qos
        )
        
        # Current measured state
        self.measured_q = [None, None, None]     # [rad]
        self.measured_v = [0.0, 0.0, 0.0]         # [rad/s]
        self.current_xyz = [0.0, 0.0, 0.0]        # [m]
        self.has_feedback = False
        
        # Trajectory Execution State
        self.is_executing = False
        self.traj_start_q = [0.0, 0.0, 0.0]
        self.traj_target_q = [0.0, 0.0, 0.0]
        self.traj_duration = 3.0
        self.traj_start_time = 0.0
        self.seq = 1
        
        # 50 Hz Control Loop Timer
        self.control_timer = self.create_timer(0.02, self._control_loop)
        self.get_logger().info("Kinematic Feedback Controller initialized.")

    def _on_mcu_state(self, msg: McuState):
        if len(msg.measured_position_rad) >= 3 and math.isfinite(msg.measured_position_rad[0]):
            self.measured_q = [float(v) for v in msg.measured_position_rad]
            self.measured_v = [float(v) for v in msg.measured_velocity_rad_s]
            self.has_feedback = True
            
            # Compute FK
            ok, xyz, _ = self.kinematics.forward_kinematics(
                self.measured_q[0], self.measured_q[1], self.measured_q[2]
            )
            if ok:
                self.current_xyz = list(xyz)

    def _control_loop(self):
        if not self.is_executing or not self.has_feedback:
            return
            
        elapsed = time.monotonic() - self.traj_start_time
        tau = min(1.0, max(0.0, elapsed / self.traj_duration))
        
        # 5th order polynomial blend: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
        s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
        # s_dot(tau) = (30*tau^2 - 60*tau^3 + 30*tau^4) / duration
        s_dot = (30.0 * (tau ** 2) - 60.0 * (tau ** 3) + 30.0 * (tau ** 4)) / max(0.1, self.traj_duration)
        
        cmd_pos = [0.0, 0.0, 0.0]
        cmd_vel = [0.0, 0.0, 0.0]
        
        for i in range(3):
            delta = self.traj_target_q[i] - self.traj_start_q[i]
            cmd_pos[i] = self.traj_start_q[i] + delta * s
            cmd_vel[i] = delta * s_dot

        # Build McuJointSetpoint
        msg = McuJointSetpoint()
        msg.protocol_version = 1
        msg.session_id = 0
        msg.sequence = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        msg.valid_for_ms = 250
        msg.enable_mask = 0x07 # Enable all 3 axes
        msg.position_rad = cmd_pos
        msg.velocity_rad_s = cmd_vel
        msg.acceleration_rad_s2 = [0.0, 0.0, 0.0]
        
        self.setpoint_pub.publish(msg)
        
        if tau >= 1.0:
            # Reached target trajectory end
            self.is_executing = False

    def move_to_joint_angles(self, target_deg: list, duration_sec: float = 3.0) -> bool:
        """Command arm to target joint angles (in degrees) with feedback starting point."""
        if not self.has_feedback:
            print("\n[ERROR] No telemetry feedback from MCU! Cannot execute motion.")
            return False
            
        target_rad = [math.radians(d) for d in target_deg]
        
        # Check joint limits
        for i in range(3):
            q_min = self.kinematics.limits[f'q{i+1}_min']
            q_max = self.kinematics.limits[f'q{i+1}_max']
            if not (q_min <= target_rad[i] <= q_max):
                print(f"\n[ERROR] Joint {i+1} target {target_deg[i]:.1f}° exceeds limit [{math.degrees(q_min):.1f}°, {math.degrees(q_max):.1f}°]")
                return False

        # Use current real measured positions as trajectory start (closed-loop start)
        self.traj_start_q = list(self.measured_q)
        self.traj_target_q = list(target_rad)
        self.traj_duration = max(1.0, float(duration_sec))
        self.traj_start_time = time.monotonic()
        self.is_executing = True
        return True

    def move_to_cartesian_xyz(self, x_m: float, y_m: float, z_m: float, duration_sec: float = 3.0) -> bool:
        """Command arm to Cartesian target position (X, Y, Z in meters)."""
        ok, ik_solution, msg = self.kinematics.inverse_kinematics(x_m, y_m, z_m)
        if not ok:
            print(f"\n[IK Error] Target ({x_m*100:.1f}, {y_m*100:.1f}, {z_m*100:.1f}) cm unreachable: {msg}")
            return False
            
        target_deg = [math.degrees(q) for q in ik_solution]
        print(f"\n[IK Solved] Target XYZ -> Joint Angles: J1={target_deg[0]:.2f}°, J2={target_deg[1]:.2f}°, J3={target_deg[2]:.2f}°")
        return self.move_to_joint_angles(target_deg, duration_sec)

    def stop_arm(self):
        self.is_executing = False
        msg = McuJointSetpoint()
        msg.protocol_version = 1
        msg.valid_for_ms = 100
        msg.enable_mask = 0 # Disarm/disable
        if self.has_feedback:
            msg.position_rad = list(self.measured_q)
        self.setpoint_pub.publish(msg)

def arm_driver(node):
    arm_cli = node.create_client(Trigger, '/doghead/driver/arm')
    if arm_cli.wait_for_service(timeout_sec=2.0):
        req = Trigger.Request()
        fut = arm_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        if fut.done() and fut.result().success:
            print(f"[Driver] System Armed: {fut.result().message}")
        else:
            print("[Driver] Arm request completed or bypassed.")

def print_status_bar(controller: KinematicFeedbackController):
    if not controller.has_feedback:
        return
    q_deg = [math.degrees(q) for q in controller.measured_q]
    xyz_cm = [p * 100.0 for p in controller.current_xyz]
    exec_status = "MOVING >>" if controller.is_executing else "IDLE"
    print(f"\r[{exec_status}] Pos: J1={q_deg[0]:5.1f}°, J2={q_deg[1]:5.1f}°, J3={q_deg[2]:5.1f}° | End-Effector XYZ=({xyz_cm[0]:5.1f}, {xyz_cm[1]:5.1f}, {xyz_cm[2]:5.1f}) cm", end="", flush=True)

def main():
    rclpy.init()
    controller = KinematicFeedbackController()
    
    # Run ROS 2 spin in background thread so CLI can take user input
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()
    
    # Arm Driver
    arm_driver(controller)
    
    print("\n" + "=" * 70)
    print("      DOGHEAD ARM - 運動學回授控制主控台 (Kinematics Feedback Control)")
    print("=" * 70)
    print("正在等待 MCU 關節反饋數據...")
    while not controller.has_feedback:
        time.sleep(0.1)
    
    print("\n[Connected] 成功接收到關節編碼器反饋！\n")
    
    try:
        while rclpy.ok():
            print("\n" + "-" * 70)
            q_deg = [math.degrees(q) for q in controller.measured_q]
            xyz_cm = [p * 100.0 for p in controller.current_xyz]
            print(f"【當前實際狀態】:")
            print(f"  • 關節角度: J1(大臂) = {q_deg[0]:6.2f}°,  J2(小臂) = {q_deg[1]:6.2f}°,  J3(基座) = {q_deg[2]:6.2f}°")
            print(f"  • 末端空間: X        = {xyz_cm[0]:6.2f} cm, Y        = {xyz_cm[1]:6.2f} cm, Z        = {xyz_cm[2]:6.2f} cm")
            print("-" * 70)
            print("請選擇操作模式:")
            print("  [1] 關節空間點對點移動 (Joint PTP: 指定 J1大臂, J2小臂, J3基座 角度)")
            print("  [2] 末端直角座標移動 (Cartesian PTP: 指定 X, Y, Z 空間座標)")
            print("  [3] 末端微調步進 (Jog Nudge: XYZ 方向每次移動 1~2 cm)")
            print("  [4] 回到基準姿態 Home (J1=111°, J2=180°, J3=0°)")
            print("  [q] 離開程式 (Quit)")
            
            choice = input("\n請輸入選項 [1/2/3/4/q]: ").strip()
            
            if choice == '1':
                raw = input("請輸入目標角度 [J1大臂 J2小臂 J3基座] (度，如 '111 170 0'): ").strip()
                try:
                    parts = [float(x) for x in raw.split()]
                    if len(parts) != 3:
                        print("輸入格式錯誤，需為 3 個數值。")
                        continue
                    dur = input("預計移動時間 (秒，預設 3.0): ").strip()
                    dur_val = float(dur) if dur else 3.0
                    print(f"\n>>> 規劃平滑軌跡移動至: {parts} deg (歷時 {dur_val}s)...")
                    controller.move_to_joint_angles(parts, dur_val)
                    time.sleep(dur_val + 0.5)
                except ValueError:
                    print("數值解析錯誤。")
                    
            elif choice == '2':
                raw = input("請輸入目標末端座標 [X Y Z] (單位: 公分 cm，如 '25.0 0.0 5.0'): ").strip()
                try:
                    parts = [float(x) for x in raw.split()]
                    if len(parts) != 3:
                        print("輸入格式錯誤，需為 3 個數值。")
                        continue
                    x_m, y_m, z_m = parts[0]/100.0, parts[1]/100.0, parts[2]/100.0
                    dur = input("預計移動時間 (秒，預設 3.0): ").strip()
                    dur_val = float(dur) if dur else 3.0
                    print(f"\n>>> 逆運動學求解並平滑移動至空間座標: ({parts[0]}, {parts[1]}, {parts[2]}) cm...")
                    controller.move_to_cartesian_xyz(x_m, y_m, z_m, dur_val)
                    time.sleep(dur_val + 0.5)
                except ValueError:
                    print("數值解析錯誤。")
                    
            elif choice == '3':
                print("\n【末端微調模式】(輸入方向與距離，例如: '+x 2' 代表 X 軸前進 2cm):")
                print("支援指令: +x, -x, +y, -y, +z, -z, 或 'b' 返回主選單")
                while True:
                    cmd = input("微調指令 > ").strip().lower()
                    if cmd == 'b' or cmd == 'exit':
                        break
                    tokens = cmd.split()
                    if len(tokens) == 0:
                        continue
                    direction = tokens[0]
                    step_cm = float(tokens[1]) if len(tokens) > 1 else 1.0
                    step_m = step_cm / 100.0
                    
                    cur_x, cur_y, cur_z = controller.current_xyz
                    if direction == '+x': cur_x += step_m
                    elif direction == '-x': cur_x -= step_m
                    elif direction == '+y': cur_y += step_m
                    elif direction == '-y': cur_y -= step_m
                    elif direction == '+z': cur_z += step_m
                    elif direction == '-z': cur_z -= step_m
                    else:
                        print("未知指令。請用 +x, -x, +y, -y, +z, -z 或 'b'")
                        continue
                    
                    controller.move_to_cartesian_xyz(cur_x, cur_y, cur_z, duration_sec=1.5)
                    time.sleep(1.6)
                    
            elif choice == '4':
                print(f"\n>>> 移動至 Home 姿態 ({HOME_STANDBY_THETA1_DEG}°, {HOME_STANDBY_THETA2_DEG}°, {HOME_STANDBY_THETA3_DEG}°)...")
                controller.move_to_joint_angles([HOME_STANDBY_THETA1_DEG, HOME_STANDBY_THETA2_DEG, HOME_STANDBY_THETA3_DEG], duration_sec=3.5)
                time.sleep(4.0)
                
            elif choice.lower() == 'q':
                break
                
    except KeyboardInterrupt:
        print("\n中斷控制...")
    finally:
        print("\n停止馬達輸出...")
        controller.stop_arm()
        time.sleep(0.3)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
