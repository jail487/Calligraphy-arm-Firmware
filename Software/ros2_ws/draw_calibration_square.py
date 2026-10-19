#!/usr/bin/env python3
"""
Doghead Arm - 實體紙面 / 空中懸空正方形校準繪製腳本 (Draw Calibration Square)
=============================================================================
核心功能：
1. 自動定位：正方形中心嚴格對齊手臂歸零待命方向正前方 (SSoT: HOME_STANDBY_THETA3_DEG = 45.0°)。
2. 雙模式支援：
   - 模式 [1] 🕊️ 空中懸空純平畫正方形：純空中平移 (預設 Z = -8.0 cm，懸空約 5cm)，全程同一高度，無下探動作，專門驗證平面是否水平或下沉。
   - 模式 [2] 🖌️ 實體紙面微壓書寫：基準 Z = -13.30 cm，微壓下探 2mm (Z = -13.50 cm)，提筆 Z = -11.00 cm。
3. 50Hz 確定性連續 PVA 規劃 (零停頓、零卡頓、零抖動)。
4. 繪製完成後自動產出含 Z 軸下沉檢驗圖 (square_test_overlay.png) 與高精度遙測日誌 (square_test_log.csv)。
"""

import sys
import os
import math
import time
import argparse
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# 導入自定義 ROS 2 訊息與專案常數
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src', 'doghead_motion', 'doghead_motion'))

from doghead_interfaces.msg import (
    McuJointSetpoint,
    McuSessionControl,
    McuState,
    McuDiagnostics
)
from robot_constants import (
    HOME_STANDBY_THETA3_DEG,
    ENABLE_MASK_ALL_JOINTS,
    SOFTWARE_SAFE_VEL_RAD_S,
    SOFTWARE_SAFE_ACCEL_RAD_S2,
    SAFETY_STATE_MAP,
    ROBOT_STATE_READY,
    ROBOT_STATE_HOMING,
    ROBOT_STATE_FAULT
)
from kinematics import DogheadKinematics
from trajectory_planner import TrajectoryPlanner

# Matplotlib 無介面渲染防護
import matplotlib
if not os.environ.get('DISPLAY'):
    matplotlib.use('Agg')
import matplotlib.pyplot as plt


class CalibrationSquareDrawer(Node):
    def __init__(self):
        super().__init__('calibration_square_drawer')
        
        # 運動學解算器與路徑規劃器
        self.kin = DogheadKinematics()
        self.planner = TrajectoryPlanner(dt=0.02)
        
        # ROS 2 發布者
        self.session_control_pub = self.create_publisher(
            McuSessionControl, '/doghead/mcu/session_control', 10
        )
        self.mcu_setpoint_pub = self.create_publisher(
            McuJointSetpoint, '/doghead/mcu/joint_setpoint', 10
        )
        
        # ROS 2 訂閱者 (50Hz 遙測)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_mcu_state, qos
        )
        self.diag_sub = self.create_subscription(
            McuDiagnostics, '/doghead/mcu/diagnostics', self._on_mcu_diagnostics, qos
        )
        
        # 狀態機與連線旗標
        self.has_feedback = False
        self.measured_q = [0.0, 0.0, 0.0]
        self.measured_v = [0.0, 0.0, 0.0]
        self.boot_id = 0
        self.session_epoch = 1
        self.session_id = 1
        self.is_armed = False
        self.seq = 1
        self.safety_state = 0
        
        # 遙測日誌紀錄
        self.recordings = []

    def _on_mcu_state(self, msg: McuState):
        if len(msg.measured_position_rad) >= 3 and math.isfinite(msg.measured_position_rad[0]):
            self.measured_q = [float(v) for v in msg.measured_position_rad[:3]]
            if len(msg.measured_velocity_rad_s) >= 3:
                self.measured_v = [float(v) for v in msg.measured_velocity_rad_s[:3]]
            self.boot_id = msg.boot_id
            self.safety_state = msg.safety_state
            self.has_feedback = True

            # 自動同步 Session ID
            expected_session = ((self.boot_id & 0xFFFF) << 16) | (self.session_epoch & 0xFFFF)
            if self.boot_id != 0 and (not self.is_armed or (self.session_id & 0xFFFF0000) != (expected_session & 0xFFFF0000)):
                self.session_id = expected_session
                self.arm_motors()

    def _on_mcu_diagnostics(self, msg: McuDiagnostics):
        pass

    def arm_motors(self):
        if self.boot_id != 0:
            self.session_id = ((self.boot_id & 0xFFFF) << 16) | (self.session_epoch & 0xFFFF)
        ctrl = McuSessionControl()
        ctrl.protocol_version = 1
        ctrl.session_id = self.session_id
        ctrl.armed = True
        self.session_control_pub.publish(ctrl)
        self.is_armed = True

    def disarm_motors(self):
        ctrl = McuSessionControl()
        ctrl.protocol_version = 1
        ctrl.session_id = self.session_id
        ctrl.armed = False
        self.session_control_pub.publish(ctrl)
        self.is_armed = False

    def smooth_move_to_joints(self, target_q, duration_sec=2.5):
        """三次方曲線平滑移動至指定關節角度，防止起始衝擊"""
        if not self.has_feedback:
            self.get_logger().error("尚未收到 MCU 回授，無法移動！")
            return False

        start_q = list(self.measured_q)
        steps = int(duration_sec * 50)
        dt = 0.02
        self.arm_motors()
        time.sleep(0.05)

        for i in range(steps):
            t_ratio = i / float(steps)
            ease = t_ratio * t_ratio * (3.0 - 2.0 * t_ratio)
            cur_q = [
                start_q[j] + ease * (target_q[j] - start_q[j])
                for j in range(3)
            ]

            msg = McuJointSetpoint()
            msg.protocol_version = 1
            msg.session_id = self.session_id
            msg.sequence = self.seq
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            msg.valid_for_ms = 100
            msg.enable_mask = ENABLE_MASK_ALL_JOINTS
            msg.position_rad = [float(q) for q in cur_q]
            msg.velocity_rad_s = [0.0, 0.0, 0.0]
            msg.acceleration_rad_s2 = [0.0, 0.0, 0.0]
            self.mcu_setpoint_pub.publish(msg)
            time.sleep(dt)

        time.sleep(0.2)
        return True

    def build_air_square_trajectory(self, z_air_cm=-8.0, square_size_cm=6.0, r_center_cm=20.0,
                                   v_run=6.0, a_run=25.0, enable_blend=True):
        """
        純空中懸空正方形軌跡：全程維持單一等高 Z 平面，無任何上下探筆動作
        專門用於驗證機械手臂在純平面移動時是否水平，有無任何下沉或傾斜
        """
        standby_rad = math.radians(HOME_STANDBY_THETA3_DEG)
        cos_t = math.cos(standby_rad)
        sin_t = math.sin(standby_rad)

        cx = r_center_cm * cos_t
        cy = r_center_cm * sin_t
        half = square_size_cm / 2.0

        print("\n" + "=" * 76)
        print("  🕊️  【空中懸空純平正方形】參數配置 (無落筆下探，全程單一水平面)")
        print("=" * 76)
        print(f"  • 底座待命朝向 (SSoT) : {HOME_STANDBY_THETA3_DEG:.1f}°")
        print(f"  • 正方形中心坐標       : X={cx:.2f} cm, Y={cy:.2f} cm (距底座 R={r_center_cm:.1f} cm)")
        print(f"  • 正方形邊長尺寸       : {square_size_cm:.1f} x {square_size_cm:.1f} cm")
        print(f"  • 懸空固定高度 (Z)     : {z_air_cm:.2f} cm (全程嚴格維持此高度，懸空約 {abs(z_air_cm - (-13.3)):.1f} cm)")
        print(f"  • 移動速度設定         : V={v_run:.1f} cm/s, A={a_run:.1f} cm/s² (過彎圓角={enable_blend})")
        print("-" * 76)

        local_corners = [
            (-half, -half),  # 1. 內近角 (靠機器人右側)
            ( half, -half),  # 2. 外遠角 (靠外側右方)
            ( half,  half),  # 3. 外遠角 (靠外側左方)
            (-half,  half)   # 4. 內近角 (靠機器人左側)
        ]

        def get_blended_corners(corners, r_blend=0.005):
            n = len(corners)
            blended = []
            for i in range(n):
                p_prev = corners[(i - 1) % n]
                p_curr = corners[i]
                p_next = corners[(i + 1) % n]
                v_in = (p_prev[0] - p_curr[0], p_prev[1] - p_curr[1])
                l_in = math.hypot(*v_in)
                u_in = (v_in[0]/l_in, v_in[1]/l_in)
                v_out = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])
                l_out = math.hypot(*v_out)
                u_out = (v_out[0]/l_out, v_out[1]/l_out)
                p_start = (p_curr[0] + r_blend * u_in[0], p_curr[1] + r_blend * u_in[1])
                p_end = (p_curr[0] + r_blend * u_out[0], p_curr[1] + r_blend * u_out[1])
                p_mid = (p_curr[0] + 0.5*r_blend*(u_in[0] + u_out[0]), p_curr[1] + 0.5*r_blend*(u_in[1] + u_out[1]))
                blended.extend([p_start, p_mid, p_end])
            blended.append(blended[0])
            return blended

        corners_to_use = get_blended_corners(local_corners, r_blend=0.005) if enable_blend else (local_corners + [local_corners[0]])

        # 逆運動學可達性前置驗證
        print("🔍 頂點可達性驗證：")
        for i, (u, v) in enumerate(local_corners, 1):
            wx = cx + u * cos_t - v * sin_t
            wy = cy + u * sin_t + v * cos_t
            ok, j, msg = self.kin.inverse_kinematics(wx/100.0, wy/100.0, z_air_cm/100.0)
            if not ok:
                print(f"  ❌ 角點 #{i} (X={wx:.2f}, Y={wy:.2f}) 超出物理限制！原因: {msg}")
                return None
            print(f"  ✅ 角點 #{i}: X={wx:5.2f}, Y={wy:5.2f}, Z={z_air_cm:5.2f} cm | 關節角: J1={math.degrees(j[0]):5.1f}°, J2={math.degrees(j[1]):5.1f}°, J3={math.degrees(j[2]):5.1f}°")

        # 構建純平面閉合路徑
        pts = []
        for u, v in corners_to_use:
            wx = cx + u * cos_t - v * sin_t
            wy = cy + u * sin_t + v * cos_t
            pts.append({'type': 'write', 'x_cm': wx * 100.0, 'y_cm': wy * 100.0, 'z_cm': z_air_cm})

        # 50Hz PVA 規劃
        print(f"\n⏳ 正在生成 50Hz 連續平滑 PVA 軌跡 (V={v_run:.1f} cm/s, A={a_run:.1f} cm/s²)...")
        pva_50hz = self.planner.plan_segment(pts, vmax=v_run, amax=a_run, kin_solver=self.kin)
        if not pva_50hz:
            print("❌ 軌跡規劃失敗！")
            return None

        print(f"✅ 規劃完成！共 {len(pva_50hz)} 個時間序列點 (50Hz)，預計耗時 {pva_50hz[-1]['t']:.2f} 秒。")
        return pva_50hz
        if not pva_50hz:
            print("❌ 軌跡規劃失敗！")
            return None

        print(f"✅ 規劃完成！共 {len(pva_50hz)} 個時間序列點 (50Hz)，預計耗時 {pva_50hz[-1]['t']:.2f} 秒。")
        return pva_50hz

    def build_square_trajectory(self, z_touch_cm=-13.30, press_depth_cm=0.20, z_travel_cm=-11.00,
                               square_size_cm=6.0, r_center_cm=20.0, align_to_robot=True):
        """紙面微壓書寫正方形 (含提筆巡航與垂直下探)"""
        z_write_cm = z_touch_cm - press_depth_cm
        standby_rad = math.radians(HOME_STANDBY_THETA3_DEG)
        cos_t = math.cos(standby_rad)
        sin_t = math.sin(standby_rad)

        cx = r_center_cm * cos_t
        cy = r_center_cm * sin_t
        half = square_size_cm / 2.0

        print("\n" + "=" * 76)
        print("  🖌️  【紙面微壓正方形】參數配置 (含提筆移動與垂直下筆)")
        print("=" * 76)
        print(f"  • 底座待命朝向 (SSoT) : {HOME_STANDBY_THETA3_DEG:.1f}°")
        print(f"  • 正方形中心坐標       : X={cx:.2f} cm, Y={cy:.2f} cm (距底座 R={r_center_cm:.1f} cm)")
        print(f"  • 正方形邊長尺寸       : {square_size_cm:.1f} x {square_size_cm:.1f} cm")
        print(f"  • 實體紙面輕觸基準 (Z) : {z_touch_cm:.2f} cm")
        print(f"  • 下壓書寫深層深度 (Z) : {z_write_cm:.2f} cm (深入紙面下壓 {press_depth_cm*10:.1f} mm)")
        print(f"  • 提筆巡航空中高度 (Z) : {z_travel_cm:.2f} cm (空中抬起 {(z_travel_cm - z_touch_cm):.1f} cm)")
        print("-" * 76)

        local_corners = [
            (-half, -half),  # 1. 內近角
            ( half, -half),  # 2. 外遠角
            ( half,  half),  # 3. 外遠角
            (-half,  half)   # 4. 內近角
        ]
        corners_xy = []
        for u, v in local_corners:
            wx = cx + u * cos_t - v * sin_t
            wy = cy + u * sin_t + v * cos_t
            corners_xy.append((wx, wy))

        print("🔍 頂點與邊界可達性驗證：")
        for i, (wx, wy) in enumerate(corners_xy, 1):
            ok_w, j_w, msg_w = self.kin.inverse_kinematics(wx/100.0, wy/100.0, z_write_cm/100.0)
            ok_t, j_t, msg_t = self.kin.inverse_kinematics(wx/100.0, wy/100.0, z_travel_cm/100.0)
            if not ok_w or not ok_t:
                print(f"  ❌ 角點 #{i} (X={wx:.2f}, Y={wy:.2f}) 超出物理限制！原因: {msg_w or msg_t}")
                return None
            print(f"  ✅ 角點 #{i}: X={wx:5.2f}, Y={wy:5.2f} cm | 落筆關節角: J1={math.degrees(j_w[0]):5.1f}°, J2={math.degrees(j_w[1]):5.1f}°, J3={math.degrees(j_w[2]):5.1f}°")

        waypoints = []
        c0 = corners_xy[0]
        waypoints.append({'type': 'travel', 'x_cm': c0[0], 'y_cm': c0[1], 'z_cm': z_travel_cm})
        waypoints.append({'type': 'write',  'x_cm': c0[0], 'y_cm': c0[1], 'z_cm': z_write_cm})

        for i in range(1, 4):
            ci = corners_xy[i]
            waypoints.append({'type': 'write', 'x_cm': ci[0], 'y_cm': ci[1], 'z_cm': z_write_cm})

        waypoints.append({'type': 'write', 'x_cm': c0[0], 'y_cm': c0[1], 'z_cm': z_write_cm})
        waypoints.append({'type': 'travel', 'x_cm': c0[0], 'y_cm': c0[1], 'z_cm': z_travel_cm})

        print("\n⏳ 正在生成 50Hz 連續平滑 PVA 軌跡...")
        pva_50hz = self.planner.plan_full_trajectory(
            waypoints,
            kin_solver=self.kin,
            v_write=3.0,
            v_travel=4.5,
            a_write=10.0,
            a_travel=12.0
        )

        if not pva_50hz:
            print("❌ 軌跡規劃失敗！")
            return None

        print(f"✅ 軌跡規劃成功！共 {len(pva_50hz)} 個控制週期點 (50Hz)，預計耗時: {pva_50hz[-1]['t']:.2f} 秒。")
        return pva_50hz

    def execute_square(self, pva_50hz):
        """確定性 50Hz 連續執行正方形書寫 (零停頓、零卡頓)"""
        if not pva_50hz:
            return

        start_pt = pva_50hz[0]
        print("\n🚀 [步驟 1/3] 緩慢平滑移動至正方形起點...")
        ok = self.smooth_move_to_joints([start_pt['q1'], start_pt['q2'], start_pt['q3']], duration_sec=2.5)
        if not ok:
            print("❌ 移動至起點失敗！")
            return

        time.sleep(0.3)
        print(f"🖌️  [步驟 2/3] 開始繪製正方形 (50Hz PVA，共 {len(pva_50hz)} 點)...")
        self.arm_motors()
        self.recordings = []

        total_pts = len(pva_50hz)
        idx = 0
        
        while idx < total_pts:
            loop_start = time.monotonic()
            pt = pva_50hz[idx]

            max_err_rad = max(
                abs(pt['q1'] - self.measured_q[0]),
                abs(pt['q2'] - self.measured_q[1]),
                abs(pt['q3'] - self.measured_q[2])
            )
            if max_err_rad > math.radians(20.0):
                print(f"\n⚠️ 【緊急安全終止】關節追隨偏差過大 ({math.degrees(max_err_rad):.1f}° > 20.0°)，疑似機械受阻！")
                break

            idx += 1

            msg = McuJointSetpoint()
            msg.protocol_version = 1
            msg.session_id = self.session_id
            msg.sequence = self.seq
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            msg.valid_for_ms = 100
            msg.enable_mask = ENABLE_MASK_ALL_JOINTS
            msg.position_rad = [float(pt['q1']), float(pt['q2']), float(pt['q3'])]
            msg.velocity_rad_s = [float(pt['v_q1']), float(pt['v_q2']), float(pt['v_q3'])]
            msg.acceleration_rad_s2 = [float(pt['a_q1']), float(pt['a_q2']), float(pt['a_q3'])]
            self.mcu_setpoint_pub.publish(msg)

            # 計算目標與實測之 3D 直角座標
            _, (tx, ty, tz), _ = self.kin.forward_kinematics(pt['q1'], pt['q2'], pt['q3'])
            _, (mx, my, mz), _ = self.kin.forward_kinematics(self.measured_q[0], self.measured_q[1], self.measured_q[2])

            self.recordings.append({
                't': pt['t'],
                'target_q': [pt['q1'], pt['q2'], pt['q3']],
                'meas_q': list(self.measured_q),
                'target_v': [pt['v_q1'], pt['v_q2'], pt['v_q3']],
                'meas_v': list(self.measured_v),
                'target_xyz': [tx*100, ty*100, tz*100],
                'meas_xyz': [mx*100, my*100, mz*100]
            })

            elapsed = time.monotonic() - loop_start
            sleep_time = 0.02 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("\n🏁 [步驟 3/3] 正方形繪製圓滿完成！")
        self.plot_results()

    def plot_results(self):
        """繪製並輸出包含末端 Z 軸下沉/平面度檢驗與 3 軸位置追隨圖表"""
        if not self.recordings:
            return

        cur_dir = os.path.dirname(os.path.abspath(__file__))
        t = [r['t'] for r in self.recordings]

        # 計算 Z 軸高度統計
        tgt_z = [r['target_xyz'][2] for r in self.recordings]
        meas_z = [r['meas_xyz'][2] for r in self.recordings]
        z_diff = [m - tg for m, tg in zip(meas_z, tgt_z)]
        max_sag_mm = abs(min(z_diff)) * 10.0 if min(z_diff) < 0 else 0.0

        print("\n" + "=" * 76)
        print("  📊 【末端 Z 軸平面度與下沉量分析統計】")
        print("=" * 76)
        print(f"  • 目標 Z 軸範圍       : [{min(tgt_z):.2f} ~ {max(tgt_z):.2f}] cm")
        print(f"  • 實測 Z 軸高度範圍   : [{min(meas_z):.2f} ~ {max(meas_z):.2f}] cm")
        print(f"  • 最大下沉偏差 (Sag)  : {max_sag_mm:.1f} mm (負偏差代表低於理論水平面)")
        print(f"  • 平均高度偏差        : {(sum(z_diff)/len(z_diff))*10.0:+.1f} mm")
        print("-" * 76)

        fig, axs = plt.subplots(4, 2, figsize=(14, 12), sharex=False)
        fig.suptitle("Square Calibration Tracking & Flatness Inspection (50Hz PVA)", fontsize=15)

        # 1. Z 軸實測 vs 目標追隨
        axs[0, 0].plot(t, tgt_z, 'r--', label='Target Z (cm)', linewidth=2)
        axs[0, 0].plot(t, meas_z, 'b-', label='Measured Z (cm)', linewidth=1.5)
        axs[0, 0].set_title("Cartesian End-Effector Z (Height Flatness)")
        axs[0, 0].set_ylabel("Height Z (cm)")
        axs[0, 0].grid(True)
        axs[0, 0].legend(loc='upper right')

        # 2. X-Y 2D 平面輪廓軌跡
        tgt_x = [r['target_xyz'][0] for r in self.recordings]
        tgt_y = [r['target_xyz'][1] for r in self.recordings]
        meas_x = [r['meas_xyz'][0] for r in self.recordings]
        meas_y = [r['meas_xyz'][1] for r in self.recordings]
        axs[0, 1].plot(tgt_x, tgt_y, 'r--', label='Target Square (cm)', linewidth=2)
        axs[0, 1].plot(meas_x, meas_y, 'b-', label='Measured Square (cm)', linewidth=1.5)
        axs[0, 1].set_title("2D Cartesian (X-Y) Trajectory Contour")
        axs[0, 1].set_xlabel("X (cm)")
        axs[0, 1].set_ylabel("Y (cm)")
        axs[0, 1].set_aspect('equal', 'box')
        axs[0, 1].grid(True)
        axs[0, 1].legend(loc='upper right')

        # 3. 三軸關節角度追隨
        axis_names = ["J1 (Shoulder)", "J2 (Elbow)", "J3 (Base)"]
        for i in range(3):
            t_q = [math.degrees(r['target_q'][i]) for r in self.recordings]
            m_q = [math.degrees(r['meas_q'][i]) for r in self.recordings]
            t_v = [math.degrees(r['target_v'][i]) for r in self.recordings]
            m_v = [math.degrees(r['meas_v'][i]) for r in self.recordings]

            row = i + 1
            axs[row, 0].plot(t, t_q, 'r--', label=f'Target {axis_names[i]}', linewidth=2)
            axs[row, 0].plot(t, m_q, 'b-', label=f'Measured {axis_names[i]}', linewidth=1.5)
            axs[row, 0].set_title(f"{axis_names[i]} Position")
            axs[row, 0].set_ylabel("Angle (deg)")
            axs[row, 0].grid(True)
            axs[row, 0].legend(loc='upper right')

            axs[row, 1].plot(t, t_v, 'r--', label=f'Target V{i+1}', linewidth=2)
            axs[row, 1].plot(t, m_v, 'b-', label=f'Measured V{i+1}', linewidth=1.5)
            axs[row, 1].set_title(f"{axis_names[i]} Velocity")
            axs[row, 1].set_ylabel("Speed (deg/s)")
            axs[row, 1].grid(True)
            axs[row, 1].legend(loc='upper right')

        axs[3, 0].set_xlabel("Time (s)")
        axs[3, 1].set_xlabel("Time (s)")
        plt.tight_layout()

        img_path = os.path.join(cur_dir, "square_test_overlay.png")
        plt.savefig(img_path, dpi=120)
        print(f"📊 遙測追蹤疊加圖已保存至: {img_path}")

        # 儲存 CSV 日誌
        csv_path = os.path.join(cur_dir, "square_test_log.csv")
        try:
            import csv
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'time_s',
                    'tgt_x_cm', 'meas_x_cm', 'tgt_y_cm', 'meas_y_cm', 'tgt_z_cm', 'meas_z_cm', 'z_err_mm',
                    'q1_tgt_deg', 'q1_meas_deg', 'q1_err_deg',
                    'q2_tgt_deg', 'q2_meas_deg', 'q2_err_deg',
                    'q3_tgt_deg', 'q3_meas_deg', 'q3_err_deg'
                ])
                for r in self.recordings:
                    writer.writerow([
                        f"{r['t']:.3f}",
                        f"{r['target_xyz'][0]:.2f}", f"{r['meas_xyz'][0]:.2f}",
                        f"{r['target_xyz'][1]:.2f}", f"{r['meas_xyz'][1]:.2f}",
                        f"{r['target_xyz'][2]:.2f}", f"{r['meas_xyz'][2]:.2f}",
                        f"{(r['meas_xyz'][2] - r['target_xyz'][2])*10.0:.2f}",
                        f"{math.degrees(r['target_q'][0]):.3f}", f"{math.degrees(r['meas_q'][0]):.3f}", f"{abs(math.degrees(r['meas_q'][0] - r['target_q'][0])):.3f}",
                        f"{math.degrees(r['target_q'][1]):.3f}", f"{math.degrees(r['meas_q'][1]):.3f}", f"{abs(math.degrees(r['meas_q'][1] - r['target_q'][1])):.3f}",
                        f"{math.degrees(r['target_q'][2]):.3f}", f"{math.degrees(r['meas_q'][2]):.3f}", f"{abs(math.degrees(r['meas_q'][2] - r['target_q'][2])):.3f}"
                    ])
            print(f"💾 完整 50Hz 執行數據 CSV 已保存至: {csv_path}\n")
        except Exception as e:
            print(f"⚠️ 保存 CSV 失敗: {e}\n")


def main():
    parser = argparse.ArgumentParser(description="Doghead Arm Calibration Square Drawer")
    parser.add_argument('--mode', type=str, default='air', choices=['air', 'paper'], help="Drawing mode: 'air' (懸空純平) or 'paper' (紙面微壓)")
    parser.add_argument('--air_z', type=float, default=-8.0, help="Air floating height Z in cm (default: -8.0)")
    parser.add_argument('--touch_z', type=float, default=-13.30, help="Paper touch surface baseline Z in cm (default: -13.30)")
    parser.add_argument('--press', type=float, default=0.20, help="Press depth into paper in cm (default: 0.20 cm = 2.0 mm)")
    parser.add_argument('--travel_z', type=float, default=-11.00, help="Pen lift travel height in cm (default: -11.00)")
    parser.add_argument('--size', type=float, default=6.0, help="Square side length in cm (default: 6.0)")
    parser.add_argument('--radius', type=float, default=20.0, help="Distance from base center in cm (default: 20.0)")
    args = parser.parse_args()

    rclpy.init()
    drawer = CalibrationSquareDrawer()

    spin_thread = threading.Thread(target=rclpy.spin, args=(drawer,), daemon=True)
    spin_thread.start()

    print("\n" + "=" * 76)
    print("  🔲 Doghead Arm - 正方形校準繪製工具 (支援空中純平懸空 / 紙面微壓)")
    print("=" * 76)
    print("正在等待 STM32 MCU 遙測通訊 (50Hz)...")

    for _ in range(50):
        if drawer.has_feedback:
            break
        time.sleep(0.1)

    if not drawer.has_feedback:
        print("❌ 尚未收到 MCU 回授！請確認 win_vcp_bridge.py 或 micro-ROS Agent 是否啟動。")
        rclpy.shutdown()
        return

    print(f"🟢 成功連線 STM32！當前安全狀態: {SAFETY_STATE_MAP.get(drawer.safety_state, '未知')}")

    # 選擇模式
    if args.mode == 'air':
        pva_50hz = drawer.build_air_square_trajectory(
            z_air_cm=args.air_z,
            square_size_cm=args.size,
            r_center_cm=args.radius
        )
    else:
        pva_50hz = drawer.build_square_trajectory(
            z_touch_cm=args.touch_z,
            press_depth_cm=args.press,
            z_travel_cm=args.travel_z,
            square_size_cm=args.size,
            r_center_cm=args.radius,
            align_to_robot=True
        )

    if not pva_50hz:
        print("❌ 軌跡生成失敗，終止執行。")
        rclpy.shutdown()
        return

    try:
        confirm = input("\n👉 請確認周圍安全，按 Enter 鍵啟動手臂開始畫正方形 (輸入 q 放棄): ").strip()
        if confirm.lower() == 'q':
            print("已取消執行。")
        else:
            drawer.execute_square(pva_50hz)
    except KeyboardInterrupt:
        print("\n使用者中斷執行...")
    finally:
        drawer.disarm_motors()
        drawer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
