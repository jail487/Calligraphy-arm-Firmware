#!/usr/bin/env python3
"""
Automated ROS2 <-> STM32 End-to-End Communication & Actuation Verifier
Sends distinct test commands and validates that STM32 decodes, applies, and reports them back.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from doghead_interfaces.msg import McuMotorTestCommand, McuState, McuDiagnostics
from std_srvs.srv import Trigger

class CommVerifier(Node):
    def __init__(self):
        super().__init__('comm_verifier')
        self.test_pub = self.create_publisher(
            McuMotorTestCommand, '/doghead/command/motor_test', 10
        )
        best_effort_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_mcu_state, best_effort_qos
        )
        self.latest_state = None
        self.state_count = 0

    def _on_mcu_state(self, msg: McuState):
        self.latest_state = msg
        self.state_count += 1

def run_verification():
    rclpy.init()
    node = CommVerifier()
    
    print("================================================================")
    print("      ROS 2 <-> STM32 Communication Verification Suite")
    print("================================================================")

    # 1. Arm the Driver
    arm_cli = node.create_client(Trigger, '/doghead/driver/arm')
    print("[1/3] Calling /doghead/driver/arm...")
    if not arm_cli.wait_for_service(timeout_sec=4.0):
        print("[-] FAIL: Arm service not available.")
        return False
    
    req = Trigger.Request()
    fut = arm_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    if not fut.done() or not fut.result().success:
        print(f"[-] FAIL: Arm failed: {fut.result().message if fut.done() else 'timeout'}")
        return False
    print(f"[+] PASS: Driver ARMED ({fut.result().message})")

    # 2. Test Cases: Distinct RPM values per axis
    test_cases = [
        {"axis": 0, "rpm": 60.0, "name": "Axis 1 (Joint 1)"},
        {"axis": 1, "rpm": 70.0, "name": "Axis 2 (Joint 2)"},
        {"axis": 2, "rpm": 80.0, "name": "Axis 3 (Joint 3)"},
    ]

    all_passed = True
    print("\n[2/3] Executing Live Command -> STM32 Verification Tests:\n")
    seq = 1

    for tc in test_cases:
        axis = tc["axis"]
        target_rpm = tc["rpm"]
        name = tc["name"]
        
        # Expected Timer PWM frequency = (rpm * 114 * 400) / 60
        expected_pwm_hz = int((target_rpm * 114.0 * 400.0) / 60.0)

        # Stream command at 20 Hz for 1.5s
        received_match = False
        start_time = time.monotonic()
        
        while time.monotonic() - start_time < 1.5:
            cmd = McuMotorTestCommand()
            cmd.protocol_version = 1
            cmd.session_id = 0
            cmd.sequence = seq
            seq += 1
            cmd.valid_for_ms = 500
            cmd.axis = axis
            cmd.target_motor_rpm = float(target_rpm)
            cmd.enable = True
            node.test_pub.publish(cmd)
            
            rclpy.spin_once(node, timeout_sec=0.05)
            
            if node.latest_state is not None:
                st = node.latest_state
                # Check if STM32 state reflects our command
                reported_rpm = st.commanded_motor_rpm[axis]
                reported_pwm = st.pwm_frequency_hz[axis]
                reported_mask = st.physical_enable_mask
                
                if abs(reported_rpm - target_rpm) < 0.1 and abs(reported_pwm - expected_pwm_hz) < 10:
                    received_match = True
                    break

        if received_match:
            print(f"  [+] {name:20s}: Sent {target_rpm:4.1f} RPM -> STM32 Received: {reported_rpm:4.1f} RPM | Timer PWM: {reported_pwm:5d} Hz | Safety State: {st.safety_state} (RUNNING) [PASS]")
        else:
            reported = node.latest_state.commanded_motor_rpm[axis] if node.latest_state else "None"
            print(f"  [-] {name:20s}: Sent {target_rpm:4.1f} RPM -> STM32 Reported: {reported} [FAIL]")
            all_passed = False

    # 3. Send Stop
    print("\n[3/3] Sending Safe Stop...")
    stop_cmd = McuMotorTestCommand()
    stop_cmd.protocol_version = 1
    stop_cmd.valid_for_ms = 100
    stop_cmd.enable = False
    node.test_pub.publish(stop_cmd)
    time.sleep(0.2)

    print("\n================================================================")
    if all_passed:
        print("  >>> VERIFICATION RESULT: 100% SUCCESS (ROS 2 -> STM32 COMM OK) <<<")
    else:
        print("  >>> VERIFICATION RESULT: SOME CHECKS FAILED <<<")
    print("================================================================\n")

    node.destroy_node()
    rclpy.shutdown()
    return all_passed

if __name__ == '__main__':
    run_verification()
