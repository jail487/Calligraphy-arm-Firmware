import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from doghead_interfaces.msg import DriverSessionState, McuJointSetpoint
from doghead_driver.safety_validator import DEFAULT_LIMITS, validate_limits


class HilScenarioRunnerNode(Node):
    """
    HIL Scenario Runner Node for safety testing of ROS 2 driver and firmware contracts.
    Executes telemetry validation, timeout scenarios, stale sequence tests, and NaN injection.
    Defaults strictly to NO actuator enablement (enable_mask = 0, disarmed).
    """

    def __init__(self):
        super().__init__('hil_scenario_runner_node')

        self.declare_parameter('actuator_enabled', False)  # HARD SAFETY DEFAULT: OFF
        self.declare_parameter('scenario_name', 'telemetry_only')
        for limit_name, default_value in DEFAULT_LIMITS.items():
            self.declare_parameter(f'limits.{limit_name}', default_value)

        self.actuator_enabled = self.get_parameter('actuator_enabled').value
        self.scenario_name = self.get_parameter('scenario_name').value
        resolved_limits = {
            limit_name: self.get_parameter(f'limits.{limit_name}').value
            for limit_name in DEFAULT_LIMITS
        }
        self.limits = validate_limits(resolved_limits)
        self.safe_baseline_position = [
            (self.limits['pos_min_rad'][index] + self.limits['pos_max_rad'][index]) / 2.0
            for index in range(3)
        ]

        self.setpoint_pub = self.create_publisher(McuJointSetpoint, '/doghead/command/joint_setpoint', 10)
        self.status_pub = self.create_publisher(String, '/doghead/hil/scenario_status', 10)
        self.active_session_id = 0
        self.driver_armed = False
        self.create_subscription(DriverSessionState, '/doghead/driver/session_state', self._on_session_state, 10)

        # Scenarios timer
        self.create_timer(1.0, self._run_scenario_tick)

        self.step_counter = 0
        self.get_logger().info(f"HIL Scenario Runner initialized. Scenario: '{self.scenario_name}', Actuator Enabled: {self.actuator_enabled}")

    def _on_session_state(self, msg: DriverSessionState):
        self.driver_armed = bool(msg.armed and msg.session_id != 0)
        self.active_session_id = msg.session_id if self.driver_armed else 0

    def _run_scenario_tick(self):
        """Execute scenario tick."""
        self.step_counter += 1
        status_msg = String()

        if self.scenario_name == 'telemetry_only':
            status_msg.data = f"Scenario: telemetry_only | Tick: {self.step_counter} | Actuators: DISABLED"
            self.get_logger().info(status_msg.data)

        elif self.scenario_name in ('command_timeout', 'stale_command', 'nan_injection') and not self.driver_armed:
            status_msg.data = f"Scenario: {self.scenario_name} | waiting for armed DriverSessionState"

        elif self.scenario_name == 'command_timeout':
            # Send setpoints for 3 ticks, then pause to trigger timeout
            if self.step_counter <= 3:
                msg = self._create_base_setpoint(sequence=self.step_counter)
                self.setpoint_pub.publish(msg)
                status_msg.data = f"Scenario: command_timeout | Active stream step {self.step_counter}"
            else:
                status_msg.data = f"Scenario: command_timeout | Paused to verify TTL expiration (step {self.step_counter})"

        elif self.scenario_name == 'stale_command':
            # Send seq 10, then send seq 5 (out-of-order)
            seq = 10 if self.step_counter % 2 == 1 else 5
            msg = self._create_base_setpoint(sequence=seq)
            self.setpoint_pub.publish(msg)
            status_msg.data = f"Scenario: stale_command | Sent sequence {seq}"

        elif self.scenario_name == 'nan_injection':
            # Inject NaN on step 3
            msg = self._create_base_setpoint(sequence=self.step_counter)
            if self.step_counter == 3:
                msg.position_rad = [0.0, float('nan'), 0.0]
                status_msg.data = "Scenario: nan_injection | Injected NaN into joint 2 position"
            else:
                status_msg.data = f"Scenario: nan_injection | Step {self.step_counter}"
            self.setpoint_pub.publish(msg)

        else:
            status_msg.data = f"Scenario: unknown '{self.scenario_name}'"

        self.status_pub.publish(status_msg)

    def _create_base_setpoint(self, sequence: int) -> McuJointSetpoint:
        """Create baseline setpoint message. Enforces actuator_enabled flag."""
        sp = McuJointSetpoint()
        sp.protocol_version = 1
        sp.session_id = self.active_session_id
        sp.sequence = sequence
        sp.valid_for_ms = 250
        # Actuators explicitly off unless parameter explicitly true AND armed
        sp.enable_mask = 0x07 if (self.actuator_enabled and self.driver_armed) else 0x00
        # Keep timeout/stale/NaN scenarios inside the configured command
        # envelope so they exercise the transport and MCU path.
        sp.position_rad = list(self.safe_baseline_position)
        sp.velocity_rad_s = [0.0, 0.0, 0.0]
        sp.acceleration_rad_s2 = [0.0, 0.0, 0.0]
        return sp


def main(args=None):
    rclpy.init(args=args)
    node = HilScenarioRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
