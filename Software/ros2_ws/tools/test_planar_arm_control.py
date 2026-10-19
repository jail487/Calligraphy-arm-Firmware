#!/usr/bin/env python3
"""
Doghead Arm - 雙軸 (大臂 J1 + 小臂 J2) 姿態回授控制與連桿參數校準工具
----------------------------------------------------------------------
專門用於「未接底座馬達 (Motor 3)」時，單獨或雙軸協同測試 Motor 1 (大臂) 與 Motor 2 (小臂)。

功能特點：
1. 僅啟用 Motor 1 & 2 (enable_mask = 0x03，底座不通電)。
2. 提供預設標準校準姿態 (90度垂直、直臂、特定角度等)。
3. 即時顯示 2D 平面長度與高度 (Reach R, Height Z)，方便拿直尺比對實體機械長度。
4. 支援自訂連桿長度 L1, L2 與目標角度。
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

from robot_constants import LINK_R1_M, LINK_R2_M, HOME_STANDBY_THETA1_DEG, HOME_STANDBY_THETA2_DEG

class PlanarArmController(Node):
    def __init__(self, l1=LINK_R1_M, l2=LINK_R2_M, l3=0.0):
        super().__init__('planar_arm_controller')
        
        self.L1 = l1  # 大臂長度 (m)
        self.L2 = l2  # 小臂長度 (m)
        self.L3 = l3  # 末端筆尖長度 (m)
        
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.setpoint_pub = self.create_publisher(
            McuJointSetpoint, '/doghead/command/joint_setpoint', 10
        )
        self.state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_mcu_state, qos
        )
        
        self.measured_q = [None, None, None]
        self.has_feedback = False
        
        self.is_executing = False
        self.traj_start_q = [0.0, 0.0, 0.0]
        self.traj_target_q = [0.0, 0.0, 0.0]
        self.traj_duration = 3.0
        self.traj_start_time = 0.0
        self.seq = 1
        
        # 50 Hz 控制循環
        self.timer = self.create_timer(0.02, self._control_loop)

    def _on_mcu_state(self, msg: McuState):
        if len(msg.measured_position_rad) >= 3 and math.isfinite(msg.measured_position_rad[0]):
            self.measured_q = [float(v) for v in msg.measured_position_rad]
            self.has_feedback = True

    def calculate_2d_fk(self, q1_rad, q2_rad):
        """計算大臂和小臂在垂直平面上的理論半徑 R 與高度 Z (公分)"""
        r_elbow = self.L1 * math.cos(q1_rad) * 100.0
        z_elbow = self.L1 * math.sin(q1_rad) * 100.0
        
        r_wrist = (self.L1 * math.cos(q1_rad) + self.L2 * math.cos(q1_rad + q2_rad)) * 100.0
        z_wrist = (self.L1 * math.sin(q1_rad) + self.L2 * math.sin(q1_rad + q2_rad)) * 100.0
        z_tip = z_wrist - self.L3 * 100.0
        return (r_elbow, z_elbow), (r_wrist, z_wrist), (r_wrist, z_tip)

    def _control_loop(self):
        if not self.is_executing or not self.has_feedback:
            return
            
        elapsed = time.monotonic() - self.traj_start_time
        tau = min(1.0, max(0.0, elapsed / self.traj_duration))
        
        # 五次多項式平滑曲線 (Quintic Polynomial)
        s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
        s_dot = (30.0 * (tau ** 2) - 60.0 * (tau ** 3) + 30.0 * (tau ** 4)) / max(0.1, self.traj_duration)
        
        cmd_pos = [0.0, 0.0, 0.0]
        cmd_vel = [0.0, 0.0, 0.0]
        
        for i in range(3):
            delta = self.traj_target_q[i] - self.traj_start_q[i]
            cmd_pos[i] = self.traj_start_q[i] + delta * s
            cmd_vel[i] = delta * s_dot

        msg = McuJointSetpoint()
        msg.protocol_version = 1
        msg.session_id = 0
        msg.sequence = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        msg.valid_for_ms = 250
        msg.enable_mask = 0x03  # 關鍵：只啟用 Motor 1 (大臂) 與 Motor 2 (小臂)，底座 (0x04) 關閉
        msg.position_rad = cmd_pos
        msg.velocity_rad_s = cmd_vel
        msg.acceleration_rad_s2 = [0.0, 0.0, 0.0]
        
        self.setpoint_pub.publish(msg)
        
        if tau >= 1.0:
            self.is_executing = False

    def move_to_deg(self, j1_deg: float, j2_deg: float, duration_sec: float = 3.0):
        if not self.has_feedback:
            print("\n[ERROR] 尚未收到 MCU 回授訊號！")
            return False
            
        self.traj_start_q = list(self.measured_q)
        # J3 (底座) 保持當前測量角度不變
        self.traj_target_q = [math.radians(j1_deg), math.radians(j2_deg), self.measured_q[2]]
        self.traj_duration = max(1.0, float(duration_sec))
        self.traj_start_time = time.monotonic()
        self.is_executing = True
        return True

    def stop_arm(self):
        self.is_executing = False
        msg = McuJointSetpoint()
        msg.protocol_version = 1
        msg.valid_for_ms = 100
        msg.enable_mask = 0
        self.setpoint_pub.publish(msg)

def arm_driver(node):
    arm_cli = node.create_client(Trigger, '/doghead/driver/arm')
    if arm_cli.wait_for_service(timeout_sec=2.0):
        req = Trigger.Request()
        fut = arm_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        if fut.done() and fut.result().success:
            print(f"[Driver] 系統已 Armed: {fut.result().message}")

def main():
    rclpy.init()
    
    # 預設連桿長度
    l1_cm = 20.0
    l2_cm = 18.0
    l3_cm = 10.0
    
    controller = PlanarArmController(l1=l1_cm/100.0, l2=l2_cm/100.0, l3=l3_cm/100.0)
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()
    
    arm_driver(controller)
    
    print("\n" + "=" * 72)
    print("      DOGHEAD ARM - 雙軸 (大臂 J1 + 小臂 J2) 姿態回授控制與校準")
    print("=" * 72)
    print("正在等待 MCU 關節回授數據...")
    while not controller.has_feedback:
        time.sleep(0.1)
        
    print("[Connected] 成功接收到關節回授！(已限制只驅動 Motor 1 & Motor 2)\n")
    
    try:
        while rclpy.ok():
            q1_deg = math.degrees(controller.measured_q[0])
            q2_deg = math.degrees(controller.measured_q[1])
            q3_deg = math.degrees(controller.measured_q[2])
            
            elbow_pt, wrist_pt, tip_pt = controller.calculate_2d_fk(
                controller.measured_q[0], controller.measured_q[1]
            )
            
            print("-" * 72)
            print(f"【當前實際回授與幾何位置】:")
            print(f"  • 關節角度: J1(大臂) = {q1_deg:6.2f}°,  J2(小臂) = {q2_deg:6.2f}°  (J3基座未通電={q3_deg:5.1f}°)")
            print(f"  • 連桿參數: L1(大臂) = {controller.L1*100:.1f} cm, L2(小臂) = {controller.L2*100:.1f} cm, L3(筆尖) = {controller.L3*100:.1f} cm")
            print(f"  • 幾何理論位置: 腕部半徑 R = {wrist_pt[0]:5.2f} cm, 筆尖高度 Z = {tip_pt[1]:5.2f} cm")
            print("-" * 72)
            print("請選擇姿態測試項目:")
            print(f"  [1] 回到 Home 基準姿態 (J1={HOME_STANDBY_THETA1_DEG}°, J2={HOME_STANDBY_THETA2_DEG}° - 直臂)")
            print("  [2] 測試單軸 J1(大臂) 移動 (保持 J2 不變)")
            print("  [3] 測試單軸 J2(小臂) 移動 (保持 J1 不變)")
            print("  [4] 自訂給定 J1 與 J2 目標角度 (自選移動秒數)")
            print("  [5] 連桿直角校準姿態 (J1=90.0° 垂直向上, J2=90.0° 直角)")
            print("  [6] 修改連桿長度參數 (L1, L2, L3 實測值更新)")
            print("  [q] 離開程式")
            
            choice = input("\n請選擇 [1-6/q]: ").strip()
            
            if choice == '1':
                print(f"\n>>> 移動至 Home 姿態 (J1={HOME_STANDBY_THETA1_DEG}°, J2={HOME_STANDBY_THETA2_DEG}°)...")
                controller.move_to_deg(HOME_STANDBY_THETA1_DEG, HOME_STANDBY_THETA2_DEG, duration_sec=3.0)
                time.sleep(3.2)
                
            elif choice == '2':
                target_j1 = input("請輸入 J1(大臂) 目標角度 (度，例如 90 或 120): ").strip()
                try:
                    j1_val = float(target_j1)
                    controller.move_to_deg(j1_val, q2_deg, duration_sec=2.5)
                    time.sleep(2.7)
                except ValueError:
                    print("輸入格式錯誤。")
                    
            elif choice == '3':
                target_j2 = input("請輸入 J2(小臂) 目標角度 (度，例如 150 或 120): ").strip()
                try:
                    j2_val = float(target_j2)
                    controller.move_to_deg(q1_deg, j2_val, duration_sec=2.5)
                    time.sleep(2.7)
                except ValueError:
                    print("輸入格式錯誤。")
                    
            elif choice == '4':
                raw = input("請輸入 [J1_deg J2_deg] (以空白隔開，例如 '100 150'): ").strip()
                try:
                    parts = [float(x) for x in raw.split()]
                    if len(parts) != 2:
                        print("需輸入 2 個數值。")
                        continue
                    dur = input("移動時間 (秒，預設 3.0): ").strip()
                    dur_val = float(dur) if dur else 3.0
                    controller.move_to_deg(parts[0], parts[1], duration_sec=dur_val)
                    time.sleep(dur_val + 0.2)
                except ValueError:
                    print("輸入格式錯誤。")
                    
            elif choice == '5':
                print("\n>>> 移動至直角校準姿態 (J1=90°, J2=90°)...")
                controller.move_to_deg(90.0, 90.0, duration_sec=3.5)
                time.sleep(3.7)
                
            elif choice == '6':
                print("\n【更新連桿長度參數 (單位: 公分 cm)】")
                try:
                    new_l1 = input(f"L1 大臂長度 [目前 {controller.L1*100:.1f} cm]: ").strip()
                    if new_l1: controller.L1 = float(new_l1) / 100.0
                    new_l2 = input(f"L2 小臂長度 [目前 {controller.L2*100:.1f} cm]: ").strip()
                    if new_l2: controller.L2 = float(new_l2) / 100.0
                    new_l3 = input(f"L3 筆尖長度 [目前 {controller.L3*100:.1f} cm]: ").strip()
                    if new_l3: controller.L3 = float(new_l3) / 100.0
                    print(f"連桿長度已更新為: L1={controller.L1*100:.1f}cm, L2={controller.L2*100:.1f}cm, L3={controller.L3*100:.1f}cm")
                except ValueError:
                    print("數值錯誤，未修改。")
                    
            elif choice.lower() == 'q':
                break
                
    except KeyboardInterrupt:
        print("\n中斷控制...")
    finally:
        print("\n安全停止馬達...")
        controller.stop_arm()
        time.sleep(0.3)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
