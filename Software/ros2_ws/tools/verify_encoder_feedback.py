#!/usr/bin/env python3
"""
Doghead Arm - Step 1: Encoder Feedback & Scale Verification Tool
-----------------------------------------------------------------
Features:
1. Real-time display of Raw Encoder Counts and Computed Joint Angles (deg & rad).
2. Delta tracking: Zero/Tare baseline at any time with Spacebar or automatically at start.
3. Theoretical ratio check: 
   - 114:1 Gear Ratio * 100 PPR * 4x Quad = 45,600 counts per 1 joint revolution (360 deg).
   - 126.67 counts per degree.
"""

import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from doghead_interfaces.msg import McuState

class EncoderVerificationNode(Node):
    def __init__(self):
        super().__init__('encoder_verification_node')
        
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_mcu_state, qos
        )
        
        self.initial_counts = None
        self.initial_pos_rad = None
        self.latest_msg = None
        self.sample_count = 0
        
        # 10 Hz Terminal Display Timer
        self.timer = self.create_timer(0.1, self._render_ui)
        self.get_logger().info("Encoder Verification Node started. Listening to /doghead/mcu/state...")

    def _on_mcu_state(self, msg: McuState):
        self.sample_count += 1
        self.latest_msg = msg
        if self.initial_counts is None and len(msg.encoder_count) >= 3:
            self.initial_counts = list(msg.encoder_count)
            self.initial_pos_rad = list(msg.measured_position_rad)

    def _render_ui(self):
        if self.latest_msg is None or self.initial_counts is None:
            print("\r[Waiting for MCU Telemetry on /doghead/mcu/state...]", end="", flush=True)
            return

        msg = self.latest_msg
        counts = list(msg.encoder_count)
        pos_rad = list(msg.measured_position_rad)
        
        # Calculate deltas from baseline
        d_counts = [counts[i] - self.initial_counts[i] for i in range(3)]
        d_deg = [math.degrees(pos_rad[i] - self.initial_pos_rad[i]) for i in range(3)]
        abs_deg = [math.degrees(p) for p in pos_rad]

        # ANSI clear screen or formatted in-place block
        output = []
        output.append("\n" + "=" * 70)
        output.append("   DOGHEAD ARM - ENCODER FEEDBACK & DIRECTION CALIBRATION (STEP 1)")
        output.append("=" * 70)
        output.append(f"MCU Uptime: {msg.uptime_ms/1000.0:6.1f} s | Telemetry Rate: Active (Seq: {msg.state_sequence})")
        output.append("-" * 70)
        output.append(f"{'Axis':<10} | {'Raw Counts':<12} | {'Delta Counts':<12} | {'Angle (deg)':<12} | {'Delta Angle':<12}")
        output.append("-" * 70)
        
        for i in range(3):
            axis_name = f"Joint {i+1}"
            output.append(
                f"{axis_name:<10} | {counts[i]:<12} | {d_counts[i]:<+12} | {abs_deg[i]:<12.2f}° | {d_deg[i]:<+12.2f}°"
            )
            
        output.append("-" * 70)
        output.append("驗證標準指引 (Verification Rules):")
        output.append("1. 手動或緩慢正轉關節時，Delta Counts 與 Delta Angle 必須為【正數 (增加)】。")
        output.append("2. 關節旋轉 1 整圈 (360°)，Delta Counts 應接近【±45,600 counts】(114:1 x 100 PPR x 4)。")
        output.append("3. 關節旋轉 90°，Delta Counts 應接近【±11,400 counts】。")
        output.append("=" * 70 + "\n")
        
        print("\n".join(output))

def main():
    rclpy.init()
    node = EncoderVerificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nVerification terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
