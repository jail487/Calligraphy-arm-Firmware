#!/usr/bin/env python3
"""
Doghead Arm Interactive Motor Velocity Controller
Allows interactive control of Axis 1, 2, 3 or simultaneous 3-axis drive with live telemetry.
"""
import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from doghead_interfaces.msg import McuMotorTestCommand, McuState
from std_srvs.srv import Trigger

class DogheadJogger(Node):
    def __init__(self, axis=0, target_rpm=50.0):
        super().__init__('doghead_jogger')
        self.axis = axis
        self.target_rpm = target_rpm
        
        self.pub = self.create_publisher(
            McuMotorTestCommand, '/doghead/command/motor_test', 10
        )
        best_effort_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_state, best_effort_qos
        )
        self.live_angles = [0.0, 0.0, 0.0]
        self.live_rpms = [0.0, 0.0, 0.0]
        self.seq = 1
        self.timer = self.create_timer(0.05, self._tick)

    def _on_state(self, msg: McuState):
        if len(msg.measured_position_rad) >= 3:
            self.live_angles = [round(math.degrees(p), 1) for p in msg.measured_position_rad]
        if len(msg.measured_velocity_rad_s) >= 3:
            self.live_rpms = [round((v / (2 * math.pi)) * 60.0 * 114.0, 1) for v in msg.measured_velocity_rad_s]

    def _tick(self):
        msg = McuMotorTestCommand()
        msg.protocol_version = 1
        msg.session_id = 0
        msg.sequence = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        msg.valid_for_ms = 500
        msg.axis = int(self.axis)
        msg.target_motor_rpm = float(self.target_rpm)
        msg.enable = True
        self.pub.publish(msg)
        
        print(f"\r[ROS 2 Driving] Axis {self.axis+1} @ {self.target_rpm:5.1f} RPM | Angles: J1={self.live_angles[0]:6.1f}°, J2={self.live_angles[1]:6.1f}°, J3={self.live_angles[2]:6.1f}°", end="", flush=True)

def main():
    axis = 0
    rpm = 50.0
    if len(sys.argv) >= 2:
        axis = int(sys.argv[1])
    if len(sys.argv) >= 3:
        rpm = float(sys.argv[2])

    rclpy.init()
    node = DogheadJogger(axis=axis, target_rpm=rpm)
    
    # Arm driver
    arm_cli = node.create_client(Trigger, '/doghead/driver/arm')
    print(f"Connecting to /doghead/driver/arm...")
    if arm_cli.wait_for_service(timeout_sec=4.0):
        req = Trigger.Request()
        fut = arm_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        if fut.done() and fut.result().success:
            print(f"Driver Armed: {fut.result().message}")
        else:
            print(f"Arm warning: {fut.result().message if fut.done() else 'timeout'}")
    
    print(f"\n>>> Running Axis {axis+1} at {rpm} RPM. Press Ctrl+C to stop. <<<\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping motor...")
        stop_msg = McuMotorTestCommand()
        stop_msg.protocol_version = 1
        stop_msg.valid_for_ms = 100
        stop_msg.axis = axis
        stop_msg.target_motor_rpm = 0.0
        stop_msg.enable = False
        node.pub.publish(stop_msg)
        time.sleep(0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
