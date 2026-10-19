import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from doghead_interfaces.msg import McuState, McuDiagnostics
from doghead_hil.tracking_metrics import TrackingMetricsAnalyzer, is_telemetry_silent


class TelemetryValidatorNode(Node):
    """
    ROS 2 Telemetry Validator Node for doghead HIL testing platform.
    Monitors 50 Hz McuState telemetry for rate accuracy, timing jitter,
    NaN/Inf occurrences, and tracking error metrics.
    """

    def __init__(self, time_fn=None):
        super().__init__('telemetry_validator_node')

        self.declare_parameter('expected_rate_hz', 50.0)
        self.declare_parameter('max_jitter_ms', 10.0)
        self.declare_parameter('rate_tolerance_hz', 2.5)
        self.declare_parameter('telemetry_timeout_ms', 500)
        self.declare_parameter('actuator_hardware_output_enabled', False)

        self.expected_rate_hz = self.get_parameter('expected_rate_hz').value
        self.max_jitter_ms = self.get_parameter('max_jitter_ms').value
        self.rate_tolerance_hz = self.get_parameter('rate_tolerance_hz').value
        self.telemetry_timeout_s = float(self.get_parameter('telemetry_timeout_ms').value) / 1000.0
        self.actuator_hardware_output_enabled = bool(
            self.get_parameter('actuator_hardware_output_enabled').value)
        if (not math.isfinite(float(self.expected_rate_hz)) or self.expected_rate_hz <= 0.0 or
                not math.isfinite(float(self.max_jitter_ms)) or self.max_jitter_ms < 0.0 or
                not math.isfinite(self.telemetry_timeout_s) or self.telemetry_timeout_s <= 0.0):
            raise ValueError('telemetry timing limits must be finite and positive')

        self.time_fn = time_fn if time_fn is not None else time.monotonic

        self.analyzer = TrackingMetricsAnalyzer()
        self.last_rx_time = None
        self.sample_count = 0
        self.last_timing_gate = None
        self.telemetry_silence_violation = False

        # QoS Profile for 50 Hz high-rate McuState telemetry
        best_effort_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(McuState, '/doghead/mcu/state', self._on_mcu_state, best_effort_qos)
        self.create_subscription(McuDiagnostics, '/doghead/mcu/diagnostics', self._on_mcu_diagnostics, 10)

        self.report_pub = self.create_publisher(String, '/doghead/hil/telemetry_report', 10)

        # Publish report summary every 5 seconds
        self.create_timer(5.0, self._publish_report)

        self.get_logger().info("Telemetry Validator Node initialized.")

    def _on_mcu_state(self, msg: McuState):
        now = self.time_fn()

        if not hasattr(msg, 'physical_enable_mask'):
            raise AttributeError("McuState message definition missing required physical_enable_mask field")
        if not hasattr(msg, 'requested_enable_mask'):
            raise AttributeError("McuState message definition missing required requested_enable_mask field")

        req_mask = int(msg.requested_enable_mask)
        phys_mask = int(msg.physical_enable_mask)
        phys_gate = int(phys_mask != 0)
        self.analyzer.record_sample(
            timestamp_s=now,
            target_pos=list(msg.target_position_rad),
            measured_pos=list(msg.measured_position_rad),
            target_vel=list(msg.target_velocity_rad_s),
            measured_vel=list(msg.measured_velocity_rad_s),
            command_age_ms=msg.command_age_ms,
            state_sequence=msg.state_sequence,
            uptime_ms=msg.uptime_ms,
            boot_id=msg.boot_id,
            applied_pos=list(msg.applied_position_rad),
            applied_vel=list(msg.applied_velocity_rad_s),
            position_error=list(msg.position_error_rad),
            commanded_rpm=list(msg.commanded_motor_rpm),
            pwm_frequency_hz=list(msg.pwm_frequency_hz),
            enable_mask=msg.enable_mask,
            physical_enable_mask=phys_mask,
            safety_state=msg.safety_state,
            fault_code=msg.fault_code,
            physical_gate_state=phys_gate,
            requested_enable_mask=req_mask,
        )

        self.sample_count += 1
        self.last_rx_time = now

    def _on_mcu_diagnostics(self, msg: McuDiagnostics):
        if msg.command_timeout_total > 0:
            self.get_logger().warn(f"MCU Diagnostics Warning: total timeouts={msg.command_timeout_total}")
        if msg.stale_sequence_total > 0:
            self.get_logger().warn(f"MCU Diagnostics Warning: stale sequences={msg.stale_sequence_total}")

    def _publish_report(self):
        now = self.time_fn()
        self.telemetry_silence_violation = is_telemetry_silent(
            self.last_rx_time, now, self.telemetry_timeout_s)
        report_str = self.analyzer.generate_markdown_report()
        self.last_timing_gate = self.analyzer.evaluate_timing(
            expected_rate_hz=self.expected_rate_hz,
            max_jitter_ms=self.max_jitter_ms,
            rate_tolerance_hz=self.rate_tolerance_hz,
            telemetry_timeout_s=self.telemetry_timeout_s,
            last_rx_time=self.last_rx_time,
            now_s=now,
            actuator_hardware_output_enabled=self.actuator_hardware_output_enabled,
        )
        gate = self.last_timing_gate
        report_str += (
            "\n### Telemetry Acceptance Gate\n"
            f"- Rate: {'PASS' if gate['rate_pass'] else 'FAIL'} "
            f"({gate['measured_rate_hz']:.3f} Hz, expected {gate['expected_rate_hz']:.3f} ± "
            f"{gate['rate_tolerance_hz']:.3f} Hz)\n"
            f"- Jitter: {'PASS' if gate['jitter_pass'] else 'FAIL'} "
            f"(max deviation {self.analyzer.calculate_telemetry_timing()['max_period_deviation_ms']:.3f} ms, "
            f"limit {gate['max_jitter_ms']:.3f} ms)\n"
            f"- Dropped frames: {'FAIL' if gate['dropped_frame_violation'] else 'PASS'} "
            f"({gate['dropped_frames']})\n"
            f"- Duplicate frames: {'FAIL' if gate['duplicate_frame_violation'] else 'PASS'} "
            f"({gate['duplicate_frames']})\n"
            f"- Out-of-order frames: {'FAIL' if gate['out_of_order_violation'] else 'PASS'} "
            f"({gate['out_of_order_frames']})\n"
            f"- Clock rollback: {'FAIL' if gate['clock_rollback_violation'] else 'PASS'} "
            f"({gate['clock_rollback_count']})\n"
            f"- Thresholds valid: {'FAIL' if gate['invalid_thresholds_violation'] else 'PASS'}\n"
            f"- Sample count valid: {'FAIL' if gate['insufficient_samples_violation'] else 'PASS'} "
            f"({gate['sample_count']})\n"
            f"- Telemetry silence: {'FAIL' if gate['telemetry_silence_violation'] else 'PASS'} "
            f"(timeout {self.telemetry_timeout_s * 1000.0:.1f} ms)\n"
            f"- Output-off safety: {'FAIL' if gate['output_off_violation'] else 'PASS'}\n"
            f"- Non-finite telemetry: {'FAIL' if gate['non_finite_telemetry_violation'] else 'PASS'} "
            f"({gate['non_finite_count']})\n"
            f"- Overall: {'PASS' if gate['pass'] else 'FAIL'}"
        )
        msg = String()
        msg.data = report_str
        self.report_pub.publish(msg)
        self.get_logger().info(f"Published Telemetry Validator Report ({self.sample_count} samples)")


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
