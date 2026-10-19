#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from doghead_interfaces.msg import McuJointSetpoint, McuState
from std_srvs.srv import Trigger

class ThreeMotorSpinner(Node):
    def __init__(self, target_motor_rpm=15.0):
        super().__init__('three_motor_spinner')
        self.target_motor_rpm = target_motor_rpm
        
        # 114:1 gear ratio: joint_rad_s = (motor_rpm / 114.0) * (2*pi / 60)
        self.joint_vel_rad_s = (self.target_motor_rpm / 114.0) * (2.0 * math.pi / 60.0)
        
        best_effort_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.setpoint_pub = self.create_publisher(
            McuJointSetpoint, '/doghead/command/joint_setpoint', 10
        )
        self.state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_state, best_effort_qos
        )
        
        self.current_pos = [None, None, None]
        self.initial_pos = [None, None, None]
        self.start_time = None
        self.seq = 0
        
        self.timer = self.create_timer(0.05, self._tick) # 20 Hz
        self.get_logger().info(f"Three-Motor Spinner initialized. Target Motor Speed: {target_motor_rpm} RPM (~{math.degrees(self.joint_vel_rad_s):.2f} deg/s)")

    def _on_state(self, msg: McuState):
        self.current_pos = list(msg.measured_position_rad)
        if self.initial_pos[0] is None and len(self.current_pos) >= 3:
            self.initial_pos = list(self.current_pos)
            self.start_time = time.monotonic()
            self.get_logger().info(f"Locked baseline joint positions: {[round(math.degrees(p), 2) for p in self.initial_pos]} deg")

    def _tick(self):
        if self.initial_pos[0] is None or self.start_time is None:
            return

        elapsed = time.monotonic() - self.start_time
        
        # Continuous position ramp for 3 axes:
        # Joint 1, 2, 3 rotate synchronously
        p1 = self.initial_pos[0] + self.joint_vel_rad_s * elapsed
        p2 = self.initial_pos[1] + self.joint_vel_rad_s * elapsed
        p3 = self.initial_pos[2] + self.joint_vel_rad_s * elapsed
        
        msg = McuJointSetpoint()
        msg.protocol_version = 1
        msg.session_id = 0
        msg.sequence = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        msg.valid_for_ms = 500
        msg.enable_mask = 7 # All 3 axes enabled (0b111)
        msg.position_rad = [p1, p2, p3]
        msg.velocity_rad_s = [self.joint_vel_rad_s, self.joint_vel_rad_s, self.joint_vel_rad_s]
        msg.acceleration_rad_s2 = [0.0, 0.0, 0.0]
        
        self.setpoint_pub.publish(msg)
        
        deg_pos = [round(math.degrees(p), 1) for p in self.current_pos] if self.current_pos[0] is not None else [0, 0, 0]
        print(f"\r[T={elapsed:5.1f}s] Sending 3-Axis Drive (15 RPM) | Current Angles: J1={deg_pos[0]}°, J2={deg_pos[1]}°, J3={deg_pos[2]}°", end="", flush=True)

def main():
    rclpy.init()
    node = ThreeMotorSpinner(target_motor_rpm=15.0)
    
    # 1. Automatically Arm the driver
    arm_cli = node.create_client(Trigger, '/doghead/driver/arm')
    print("Waiting for /doghead/driver/arm service...")
    if arm_cli.wait_for_service(timeout_sec=3.0):
        req = Trigger.Request()
        future = arm_cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if future.done() and future.result().success:
            print(f"Driver Armed: {future.result().message}")
        else:
            print(f"Arm warning: {future.result().message if future.done() else 'timeout'}")
    else:
        print("Driver arm service not available (bypassing)")

    print("\n>>> Rotating all 3 motors simultaneously. Press Ctrl+C to stop. <<<\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping motors...")
        # Send zero setpoint
        msg = McuJointSetpoint()
        msg.protocol_version = 1
        msg.valid_for_ms = 100
        msg.enable_mask = 0
        node.setpoint_pub.publish(msg)
        time.sleep(0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
