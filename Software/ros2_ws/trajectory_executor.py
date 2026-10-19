import sys
import os
import math
import time
import threading
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

# 匯入您專案的自定義 ROS 2 訊息
from doghead_interfaces.msg import McuJointSetpoint, McuSessionControl, McuState

# 匯入我們寫好的規劃器與映射器
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectory_mapper import TrajectoryMapper
from trajectory_planner import TrajectoryPlanner
from robot_constants import ENABLE_MASK_ALL_JOINTS


class TrajectoryExecutorNode(Node):
    def __init__(self):
        super().__init__('trajectory_executor')
        
        # 發布器
        self.session_control_pub = self.create_publisher(
            McuSessionControl, '/doghead/mcu/session_control', 10
        )
        self.mcu_setpoint_pub = self.create_publisher(
            McuJointSetpoint, '/doghead/mcu/joint_setpoint', 10
        )
        
        # 訂閱器 (Telemetry)
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_mcu_state, qos
        )
        
        # 狀態變數
        self.has_feedback = False
        self.measured_q = [0.0, 0.0, 0.0]
        self.measured_v = [0.0, 0.0, 0.0]
        self.session_id = 1
        self.recording = []

    def _on_mcu_state(self, msg: McuState):
        self.has_feedback = True
        self.session_id = msg.session_id if msg.session_id != 0 else 1
        self.measured_q = list(msg.measured_position_rad)
        self.measured_v = list(msg.measured_velocity_rad_s)

    def arm_motors(self):
        ctrl = McuSessionControl()
        ctrl.session_id = self.session_id
        ctrl.armed = True
        self.session_control_pub.publish(ctrl)
        time.sleep(0.1)
        self.session_control_pub.publish(ctrl)
        
    def disarm_motors(self):
        ctrl = McuSessionControl()
        ctrl.session_id = self.session_id
        ctrl.armed = False
        self.session_control_pub.publish(ctrl)
        
    def slow_move_to_q(self, target_q, duration=3.0):
        """用平滑的三次方曲線將手臂緩慢移動到起點，避免暴衝"""
        if not self.has_feedback: return
        start_q = list(self.measured_q)
        steps = int(duration * 50)
        dt = 0.02
        
        self.arm_motors()
        
        for i in range(steps):
            ratio = i / steps
            ease = ratio * ratio * (3.0 - 2.0 * ratio)  # Cubic ease-in-out
            
            msg = McuJointSetpoint()
            msg.enable_mask = ENABLE_MASK_ALL_JOINTS
            msg.position_rad = [
                start_q[0] + ease * (target_q[0] - start_q[0]),
                start_q[1] + ease * (target_q[1] - start_q[1]),
                start_q[2] + ease * (target_q[2] - start_q[2])
            ]
            msg.velocity_rad_s = [0.0, 0.0, 0.0]
            msg.acceleration_rad_s2 = [0.0, 0.0, 0.0]
            self.mcu_setpoint_pub.publish(msg)
            time.sleep(dt)
        time.sleep(0.5)

    def execute_stroke_and_record(self, trajectory_50hz):
        """嚴格以 50Hz 執行 PVA 並錄製真實回傳值"""
        start_pt = trajectory_50hz[0]
        
        print("\n[執行] 1. 緩慢移動至筆劃起點...")
        self.slow_move_to_q([start_pt['q1'], start_pt['q2'], start_pt['q3']], duration=2.5)
        
        print(f"[執行] 2. 開始 50Hz 真實筆劃寫作 (共 {len(trajectory_50hz)} 點)...")
        self.recording = []
        
        for pt in trajectory_50hz:
            loop_start = time.monotonic()
            
            # 1. 發送 50Hz PVA 指令
            msg = McuJointSetpoint()
            msg.enable_mask = ENABLE_MASK_ALL_JOINTS
            msg.position_rad = [pt['q1'], pt['q2'], pt['q3']]
            msg.velocity_rad_s = [pt['v_q1'], pt['v_q2'], pt['v_q3']]
            msg.acceleration_rad_s2 = [pt['a_q1'], pt['a_q2'], pt['a_q3']]
            self.mcu_setpoint_pub.publish(msg)
            
            # 2. 記錄這一個週期的「目標」與「真實表現」
            self.recording.append({
                't': pt['t'],
                'target_q': [pt['q1'], pt['q2'], pt['q3']],
                'meas_q': list(self.measured_q),
                'target_v': [pt['v_q1'], pt['v_q2'], pt['v_q3']],
                'meas_v': list(self.measured_v)
            })
            
            # 3. 補償休眠以嚴格維持 50Hz (0.02s)
            elapsed = time.monotonic() - loop_start
            sleep_time = 0.02 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
        print("[執行] 3. 筆劃執行完畢！生成調適圖表...")
        self.plot_overlay()
        
    def plot_overlay(self):
        if not self.recording:
            return
            
        t = [r['t'] for r in self.recording]
        
        # 只畫最關鍵的 J1 (大臂) 與 J2 (小臂) 來做調適範例
        t_q1 = [math.degrees(r['target_q'][0]) for r in self.recording]
        m_q1 = [math.degrees(r['meas_q'][0]) for r in self.recording]
        t_v1 = [math.degrees(r['target_v'][0]) for r in self.recording]
        m_v1 = [math.degrees(r['meas_v'][0]) for r in self.recording]
        
        t_q2 = [math.degrees(r['target_q'][1]) for r in self.recording]
        m_q2 = [math.degrees(r['meas_q'][1]) for r in self.recording]
        t_v2 = [math.degrees(r['target_v'][1]) for r in self.recording]
        m_v2 = [math.degrees(r['meas_v'][1]) for r in self.recording]

        fig, axs = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
        fig.suptitle("Calligraphy Trajectory Tuning: Target vs Measured", fontsize=16)

        # J1 Position
        axs[0, 0].plot(t, t_q1, 'r--', label='Target J1', linewidth=2)
        axs[0, 0].plot(t, m_q1, 'b-', label='Measured J1', linewidth=1.5)
        axs[0, 0].set_title("J1 Position (deg)")
        axs[0, 0].grid(True)
        axs[0, 0].legend()

        # J1 Velocity (調適的關鍵！)
        axs[1, 0].plot(t, t_v1, 'r--', label='Target V1', linewidth=2)
        axs[1, 0].plot(t, m_v1, 'b-', label='Measured V1', linewidth=1.5)
        axs[1, 0].set_title("J1 Velocity (deg/s) - Tune Kv and Kp here!")
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].grid(True)
        axs[1, 0].legend()

        # J2 Position
        axs[0, 1].plot(t, t_q2, 'r--', label='Target J2', linewidth=2)
        axs[0, 1].plot(t, m_q2, 'b-', label='Measured J2', linewidth=1.5)
        axs[0, 1].set_title("J2 Position (deg)")
        axs[0, 1].grid(True)
        axs[0, 1].legend()

        # J2 Velocity
        axs[1, 1].plot(t, t_v2, 'r--', label='Target V2', linewidth=2)
        axs[1, 1].plot(t, m_v2, 'b-', label='Measured V2', linewidth=1.5)
        axs[1, 1].set_title("J2 Velocity (deg/s)")
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].grid(True)
        axs[1, 1].legend()

        plt.tight_layout()
        save_path = os.path.join(os.path.dirname(__file__), 'tuning_overlay.png')
        plt.savefig(save_path)
        print(f"\n[圖表] 真實追蹤對比圖已儲存至: {save_path}")
        print("💡 調適指南:")
        print(" - 如果藍線(真實速度)比紅線慢起步，請調高前饋 Kv。")
        print(" - 如果藍線在紅線上瘋狂發抖鋸齒狀，請調低 Kp。")
        print(" - 如果藍線衝過頭(超調)，請微微調高 Kd 或降低 Kp。")
        
        plt.show() # 如果在 Windows 上執行，會直接彈出視窗讓使用者看


def main():
    rclpy.init()
    node = TrajectoryExecutorNode()
    
    # 建立背景執行緒來處理 ROS 回調
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n" + "=" * 76)
    print("  🚀 書法軌跡疊加調適引擎 (Trajectory Executor & Tuner)")
    print("=" * 76)
    
    # 等待連線
    print("等待 MCU Telemetry 回傳...")
    for _ in range(30):
        if node.has_feedback: break
        time.sleep(0.1)
        
    if not node.has_feedback:
        print("❌ 尚未收到 MCU 回傳，請確認 ROS 2 Agent 是否啟動。")
        return

    try:
        # 1. 載入 JSON 與映射 (優先尋找工作區內本地 JSON，跨 Docker/Windows 兼容)
        local_json = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_path.json")
        ext_json = r"C:\Users\jacks\Documents\project\GraduationProject\BBSMG_ws\samples\waypoint_path\robot_path.json"
        test_json = local_json if os.path.exists(local_json) else ext_json
        print(f"\n載入 JSON: {os.path.basename(test_json)} ({test_json})")
        mapper = TrajectoryMapper(z_write_cm=3.5, z_travel_cm=8.0, margin_cm=1.5)
        raw_points = mapper.map_trajectory(test_json)
        
        # 取出第一段 Write 筆劃
        write_segment = []
        for pt in raw_points:
            if pt['type'] == 'write':
                write_segment.append(pt)
            elif len(write_segment) > 0:
                break
                
        # 2. 生成 50Hz 目標 PVA
        print(f"\n進行 PVA 軌跡規劃 (Vmax=4.0 cm/s, Amax=20.0 cm/s^2)...")
        planner = TrajectoryPlanner(dt=0.02)
        pva_50hz = planner.plan_segment(write_segment, vmax=4.0, amax=20.0, kin_solver=mapper.kin)
        
        if not pva_50hz:
            print("規劃失敗或筆劃過短。")
            return
            
        print(f"規劃完成！準備執行 {len(pva_50hz)} 個時間序列點，耗時 {pva_50hz[-1]['t']:.2f} 秒。")
        input("👉 準備好後，按 Enter 鍵啟動手臂並開始繪圖調適...")
        
        # 3. 執行並畫圖
        node.execute_stroke_and_record(pva_50hz)
        
    except KeyboardInterrupt:
        print("\n中斷...")
    finally:
        node.disarm_motors()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

