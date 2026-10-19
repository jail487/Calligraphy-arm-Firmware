#!/usr/bin/env python3
"""
Doghead Arm - 撖阡?蝝?偶???豢??啣神?單 (Draw Calligraphy '瘞? / Yong)
=============================================================================
?詨??寞改?
1. 摰?偶摮瘜?霈??robot_path.json 銝剔? 5 憭抒???暺帖???扎????綽?嚗 47 ?敺萇?暺?2. 50Hz ???撟單? PVA 閬?嚗????蝑楚??(travel) ??蝑撖?(write)嚗? Hanning 瞈暹郭瘨??????3. 撖阡??暹?蝝 3D 蝛粹?鋆?嚗窒?冽迤?孵耦撖行葫璅????Ｗ像?Ｘ蝔? (touch_z=-12.73, tilt=4.29, travel_z=-10.43)??4. 撟曆?瘥?鋆?嚗?撠?22簞 ???45簞 ?蔣?脰??瑕祝鋆?嚗Ⅱ靽蝝銝鼓?箇垢甇?? 60mm x 60mm ?偶????5. 憭芋撘?湛?
   - 璅∪? [1] ?? 摰?偶???詨神 (5 蝑???疵摰?嚗? 25 蝘?
   - 璅∪? [2] ? ?神蝚砌?蝑???敹恍?霅蝑?摨西?憓冽偌?箏◢)
"""

import sys
import os
import json
import math
import time
import argparse
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)
sys.path.insert(0, os.path.join(script_dir, 'src', 'doghead_motion', 'doghead_motion'))

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
    ROBOT_STATE_READY
)
from kinematics import DogheadKinematics
from trajectory_planner import TrajectoryPlanner

import matplotlib
if not os.environ.get('DISPLAY'):
    matplotlib.use('Agg')
import matplotlib.pyplot as plt


class CalligraphyYongDrawer(Node):
    def __init__(self):
        super().__init__('calligraphy_yong_drawer')
        self.kin = DogheadKinematics()
        self.planner = TrajectoryPlanner(dt=0.02)

        self.session_control_pub = self.create_publisher(
            McuSessionControl, '/doghead/mcu/session_control', 10
        )
        self.mcu_setpoint_pub = self.create_publisher(
            McuJointSetpoint, '/doghead/mcu/joint_setpoint', 10
        )

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_mcu_state, qos
        )
        self.diag_sub = self.create_subscription(
            McuDiagnostics, '/doghead/mcu/diagnostics', self._on_mcu_diagnostics, qos
        )

        self.has_feedback = False
        self.measured_q = [0.0, 0.0, 0.0]
        self.measured_v = [0.0, 0.0, 0.0]
        self.boot_id = 0
        self.session_epoch = max(1, int(time.time()) & 0xFFFF)
        self.session_id = 0
        self.is_armed = False
        self.seq = 1
        self.safety_state = 0
        self.recordings = []

    def _on_mcu_state(self, msg: McuState):
        if len(msg.measured_position_rad) >= 3 and math.isfinite(msg.measured_position_rad[0]):
            self.measured_q = [float(v) for v in msg.measured_position_rad[:3]]
            if len(msg.measured_velocity_rad_s) >= 3:
                self.measured_v = [float(v) for v in msg.measured_velocity_rad_s[:3]]
            self.boot_id = msg.boot_id
            self.safety_state = msg.safety_state
            self.has_feedback = True

            if hasattr(msg, 'last_accepted_command_sequence') and msg.last_accepted_command_sequence >= self.seq:
                self.seq = (msg.last_accepted_command_sequence + 1) & 0xFFFFFFFF

            expected_session = ((self.boot_id & 0xFFFF) << 16) | (self.session_epoch & 0xFFFF)
            if self.boot_id != 0 and (not self.is_armed or self.session_id != expected_session):
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
        if not self.has_feedback:
            return False
        self.arm_motors()
        time.sleep(0.1)
        start_q = list(self.measured_q)
        steps = int(duration_sec * 50)
        dt = 0.02
        self.arm_motors()
        time.sleep(0.05)

        for i in range(steps):
            t_ratio = i / float(steps)
            ease = t_ratio * t_ratio * (3.0 - 2.0 * t_ratio)
            d_ease = (6.0 * t_ratio * (1.0 - t_ratio)) / duration_sec
            cur_q = [
                start_q[j] + ease * (target_q[j] - start_q[j])
                for j in range(3)
            ]
            cur_v = [
                d_ease * (target_q[j] - start_q[j])
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
            msg.velocity_rad_s = [float(v) for v in cur_v]
            msg.acceleration_rad_s2 = [0.0, 0.0, 0.0]
            self.mcu_setpoint_pub.publish(msg)
            time.sleep(dt)

        time.sleep(0.2)
        return True

    def build_yong_trajectory(self, json_path, z_touch_cm=-12.73, z_travel_cm=-10.43,
                              r_center_cm=20.0, size_radial_cm=5.35, size_tangential_cm=8.485,
                              tilt_diff_cm=4.29, min_press_mm=1.8, max_press_mm=3.6,
                              v_write=2.0, v_travel=3.5, a_write=8.0, a_travel=10.0,
                              single_stroke=False):
        """讀取 JSON 筆劃並建構乾淨、無拖尾的 50Hz 3D 空間書法軌跡"""
        if not os.path.exists(json_path):
            print(f"書法路徑檔案不存在: {json_path}")
            return None

        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        raw_wps = data.get('waypoints', [])
        if not raw_wps:
            return None

        # 乾淨解析成獨立的筆劃 (Strokes)
        strokes = []
        curr_stroke = []
        for wp in raw_wps:
            if wp['type'] == 'travel':
                if curr_stroke:
                    strokes.append(curr_stroke)
                    curr_stroke = []
            else:
                curr_stroke.append(wp)
        if curr_stroke:
            strokes.append(curr_stroke)

        if single_stroke and strokes:
            strokes = [strokes[0]]

        standby_rad = math.radians(HOME_STANDBY_THETA3_DEG)
        cos_t = math.cos(standby_rad)
        sin_t = math.sin(standby_rad)
        cx = r_center_cm * cos_t
        cy = r_center_cm * sin_t

        r_span = 6.0 * math.sqrt(2.0)
        tilt_slope = (tilt_diff_cm / r_span) if r_span > 0 else 0.0
        travel_h_cm = z_travel_cm - z_touch_cm

        def map_coord(px, py, z_json):
            u_norm = (py / 1024.0) - 0.5
            v_norm = 0.5 - (px / 1024.0)
            u = u_norm * size_radial_cm
            v = v_norm * size_tangential_cm
            wx = cx + u * cos_t - v * sin_t
            wy = cy + u * sin_t + v * cos_t
            r_pt = math.hypot(wx, wy)
            delta_r = r_pt - r_center_cm
            z_paper = z_touch_cm + tilt_slope * delta_r
            press_ratio = max(0.0, min(1.0, (abs(z_json) - 1.0) / 3.0))
            press_depth_cm = (min_press_mm + press_ratio * (max_press_mm - min_press_mm)) / 10.0
            wz = z_paper - press_depth_cm
            return wx, wy, wz, z_paper + travel_h_cm

        physical_pts = []
        for s_idx, stroke in enumerate(strokes):
            if not stroke:
                continue

            # 1. 筆劃起點
            s_start = stroke[0]
            sx, sy, sz_write, sz_air = map_coord(s_start['x'], s_start['y'], s_start['z'])

            # [空中移動] 移動到該筆起點的正上方 (空中高度)
            physical_pts.append({'type': 'travel', 'x_cm': sx, 'y_cm': sy, 'z_cm': sz_air})
            # [空中停頓] 確保到達起點正上方時速度完全歸零
            physical_pts.append({'type': 'travel', 'x_cm': sx, 'y_cm': sy, 'z_cm': sz_air})

            # [垂直下筆] 原地垂直插進紙面至指定落筆深度 (不帶水平位移)
            physical_pts.append({'type': 'write', 'x_cm': sx, 'y_cm': sy, 'z_cm': sz_write})

            # 2. [沿紙書寫] 依序走完該筆中間各點
            for wp in stroke[1:]:
                wx, wy, wz_write, _ = map_coord(wp['x'], wp['y'], wp['z'])
                physical_pts.append({'type': 'write', 'x_cm': wx, 'y_cm': wy, 'z_cm': wz_write})

            # 3. 筆劃結尾
            s_end = stroke[-1]
            ex, ey, ez_end_write, ez_air = map_coord(s_end['x'], s_end['y'], s_end['z'])

            # [垂直提筆] 在收尾點原地垂直拔高到空中高度 (徹底脫離紙面，絕不帶水平速度起飛)
            physical_pts.append({'type': 'travel', 'x_cm': ex, 'y_cm': ey, 'z_cm': ez_air})
            # [空中靜止停頓] 停留一個點確保毛筆筆尖完全彈起離開紙面，絕不牽絲
            physical_pts.append({'type': 'travel', 'x_cm': ex, 'y_cm': ey, 'z_cm': ez_air})

        print(f"生成的嚴格離散起落筆物理點數: {len(physical_pts)}")
        pva_50hz = self.planner.plan_full_trajectory(
            physical_pts,
            kin_solver=self.kin,
            v_write=v_write,
            v_travel=v_travel,
            a_write=a_write,
            a_travel=a_travel
        )

        return pva_50hz

    def execute_calligraphy(self, pva_50hz):
        if not pva_50hz:
            return

        start_pt = pva_50hz[0]
        print("\n?? [甇仿? 1/3] 蝺拇撟單?蝘餃??喟洵銝蝑絲暺?(蝛箔葉??)...")
        ok = self.smooth_move_to_joints([start_pt['q1'], start_pt['q2'], start_pt['q3']], duration_sec=2.5)
        if not ok:
            print("??蝘餃??唾絲暺仃??")
            return

        time.sleep(0.3)
        print(f"??儭? [甇仿? 2/3] ???格神?詨神?偶?? (50Hz PVA嚗 {len(pva_50hz)} 暺?...")
        self.arm_motors()
        self.recordings = []

        total_pts = len(pva_50hz)
        start_time = time.monotonic()

        for i, pt in enumerate(pva_50hz):
            loop_start = time.monotonic()

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

            _, meas_xyz, _ = self.kin.forward_kinematics(self.measured_q[0], self.measured_q[1], self.measured_q[2])
            self.recordings.append({
                't': pt['t'],
                'target_xyz': (pt['x'], pt['y'], pt['z']),
                'meas_xyz': (meas_xyz[0]*100.0, meas_xyz[1]*100.0, meas_xyz[2]*100.0),
                'target_q': (pt['q1'], pt['q2'], pt['q3']),
                'meas_q': list(self.measured_q),
                'seg_type': pt.get('seg_type', 'write')
            })

            if i % 100 == 0 or i == total_pts - 1:
                prog = (i + 1) / float(total_pts) * 100.0
                print(f"  ???格神?脣漲: {prog:5.1f}% | 撖行???: J1={math.degrees(self.measured_q[0]):5.1f}簞, J2={math.degrees(self.measured_q[1]):5.1f}簞, J3={math.degrees(self.measured_q[2]):5.1f}簞 | Z={meas_xyz[2]*100.0:5.2f} cm")

            elapsed = time.monotonic() - loop_start
            sleep_t = 0.02 - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        pass
        self.plot_calligraphy_overlay()

    def plot_calligraphy_overlay(self):
        if not self.recordings:
            return

        t = [r['t'] for r in self.recordings]
        t_x = [r['target_xyz'][0] for r in self.recordings]
        m_x = [r['meas_xyz'][0] for r in self.recordings]
        t_y = [r['target_xyz'][1] for r in self.recordings]
        m_y = [r['meas_xyz'][1] for r in self.recordings]
        t_z = [r['target_xyz'][2] for r in self.recordings]
        m_z = [r['meas_xyz'][2] for r in self.recordings]

        fig, axs = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle("Calligraphy 'Yong' Execution & Flatness Inspection (50Hz PVA)", fontsize=16)

        # 1. 2D 頠楚
        axs[0, 0].plot(t_x, t_y, 'r--', label='Target Contour', linewidth=2)
        axs[0, 0].plot(m_x, m_y, 'b-', label='Measured Contour', linewidth=1.5)
        axs[0, 0].set_title("2D Cartesian (X-Y) Trajectory Contour")
        axs[0, 0].set_xlabel("X (cm)")
        axs[0, 0].set_ylabel("Y (cm)")
        axs[0, 0].grid(True)
        axs[0, 0].legend()
        axs[0, 0].axis('equal')

        # 2. Z 擃漲餈質馱
        axs[0, 1].plot(t, t_z, 'r--', label='Target Z', linewidth=2)
        axs[0, 1].plot(t, m_z, 'b-', label='Measured Z', linewidth=1.5)
        axs[0, 1].set_title("Z Height vs Time (Pen Lift & Press)")
        axs[0, 1].set_xlabel("Time (s)")
        axs[0, 1].set_ylabel("Z (cm)")
        axs[0, 1].grid(True)
        axs[0, 1].legend()

        # 3. 銝遘??閫漲
        for j_idx, ax_name in enumerate(['J1 (Shoulder)', 'J2 (Elbow)', 'J3 (Base)']):
            row = 1 + (j_idx // 2)
            col = j_idx % 2
            ax = axs[row, col]
            t_q = [math.degrees(r['target_q'][j_idx]) for r in self.recordings]
            m_q = [math.degrees(r['meas_q'][j_idx]) for r in self.recordings]
            ax.plot(t, t_q, 'r--', label=f'Target {ax_name}', linewidth=2)
            ax.plot(t, m_q, 'b-', label=f'Measured {ax_name}', linewidth=1.5)
            ax.set_title(f"{ax_name} Position (deg)")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Angle (deg)")
            ax.grid(True)
            ax.legend()

        # 4. 銝?隤文榆
        z_errors = [(r['meas_xyz'][2] - r['target_xyz'][2]) * 10.0 for r in self.recordings]
        axs[2, 1].plot(t, z_errors, 'm-', label='Z Tracking Error (mm)', linewidth=1.5)
        axs[2, 1].axhline(0, color='gray', linestyle='--')
        axs[2, 1].set_title("Z Axis Flatness Error (mm)")
        axs[2, 1].set_xlabel("Time (s)")
        axs[2, 1].set_ylabel("Error (mm)")
        axs[2, 1].grid(True)
        axs[2, 1].legend()

        plt.tight_layout()
        save_path = os.path.join(os.path.dirname(__file__), 'calligraphy_stroke_overlay.png')
        plt.savefig(save_path, dpi=120)
        print(f"\n?? ?葫餈質馱???歇?脣??? {save_path}")


def main():
    parser = argparse.ArgumentParser(description="Doghead Arm Calligraphy 'Yong' Drawer")
    parser.add_argument('--json', type=str, default=os.path.join(os.path.dirname(__file__), 'robot_path.json'), help="Path to character trajectory JSON")
    parser.add_argument('--touch_z', type=float, default=-12.73, help="Paper touch surface baseline Z in cm (calibrated: -12.73)")
    parser.add_argument('--travel_z', type=float, default=-10.43, help="Pen lift travel height in cm (calibrated: -10.43)")
    parser.add_argument('--tilt', type=float, default=4.29, help="Front-to-back tilt difference in cm (calibrated: 4.29)")
    parser.add_argument('--radius', type=float, default=20.0, help="Distance from base center in cm (default: 20.0)")
    parser.add_argument('--size_r', type=float, default=5.35, help="Radial side length (along arm) in cm (calibrated: 5.35)")
    parser.add_argument('--size_t', type=float, default=8.485, help="Tangential side length (across arm) in cm (calibrated: 8.485)")
    parser.add_argument('--min_press', type=float, default=1.8, help="Minimum press depth in mm (default: 1.0)")
    parser.add_argument('--max_press', type=float, default=3.6, help="Maximum press depth for heavy strokes in mm (default: 2.5)")
    parser.add_argument('--speed', type=float, default=2.0, help="Movement velocity in cm/s (default: 2.0)")
    parser.add_argument('--accel', type=float, default=8.0, help="Movement acceleration in cm/s^2 (default: 8.0)")
    parser.add_argument('--single_stroke', action='store_true', default=False, help="Only test stroke 1 ('Dot') for safety verification")
    parser.add_argument('--auto_start', action='store_true', default=False, help="Directly execute without interactive prompt")
    args = parser.parse_args()

    rclpy.init()
    drawer = CalligraphyYongDrawer()

    spin_thread = threading.Thread(target=rclpy.spin, args=(drawer,), daemon=True)
    spin_thread.start()

    print("\n" + "=" * 76)
    print("  ??儭? Doghead Arm - 撖阡?蝝?偶???豢??啣神撌亙")
    print("=" * 76)
    print("甇?蝑? STM32 MCU ?葫?? (50Hz)...")

    for _ in range(50):
        if drawer.has_feedback:
            break
        time.sleep(0.1)

    if not drawer.has_feedback:
        pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return

    print(f"? ????? STM32嚗???函??? {SAFETY_STATE_MAP.get(drawer.safety_state, '?芰')}")

    pva_50hz = drawer.build_yong_trajectory(
        json_path=args.json,
        z_touch_cm=args.touch_z,
        z_travel_cm=args.travel_z,
        r_center_cm=args.radius,
        size_radial_cm=args.size_r,
        size_tangential_cm=args.size_t,
        tilt_diff_cm=args.tilt,
        min_press_mm=args.min_press,
        max_press_mm=args.max_press,
        v_write=args.speed,
        v_travel=3.5,
        a_write=args.accel,
        a_travel=10.0,
        single_stroke=args.single_stroke
    )

    if not pva_50hz:
        pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return

    try:
        if args.auto_start:
            drawer.execute_calligraphy(pva_50hz)
        else:
            confirm = input("\n?? 隢Ⅱ隤?蝑◢瘞游?頞喋?撘萄歇?芸末嚗? Enter ?萄????瘥?(頛詨 q ?暹?): ").strip()
            if confirm.lower() == 'q':
                return
            else:
                drawer.execute_calligraphy(pva_50hz)
    except KeyboardInterrupt:
        print("\n雿輻?葉?瑕銵?..")
    finally:
        drawer.disarm_motors()
        drawer.destroy_node()
        rclpy.try_shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()



