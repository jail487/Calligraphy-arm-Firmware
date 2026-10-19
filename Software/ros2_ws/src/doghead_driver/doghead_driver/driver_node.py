import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from doghead_interfaces.msg import (
    McuState,
    McuDiagnostics,
    McuJointSetpoint,
    McuMotorTestCommand,
    DriverSessionState,
    McuSessionControl,
)
from doghead_driver.safety_validator import (
    DriverSafetyState,
    check_two_key_actuator_gate,
    is_finite_sequence,
    DEFAULT_JOINT_NAMES,
    DEFAULT_LIMITS,
    EXPECTED_PROTOCOL_VERSION,
    validate_limits
)


class DogheadDriverNode(Node):
    """
    ROS 2 driver node for safe telemetry-first Joint-PVA/EF HIL orchestration.
    Converts McuState to /joint_states and enforces safe arming, disarming,
    reconnect handling, sequence validation, TTL check, and finite/envelope rules.

    Two-Key Actuator Gate:
    Physical output enablement requires:
    1. Driver is ARMED (via arm service)
    2. shadow_mode parameter is FALSE
    3. actuator_enabled parameter is TRUE
    """

    def __init__(self):
        super().__init__('doghead_driver_node')

        # Parameters
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('protocol_version', EXPECTED_PROTOCOL_VERSION)
        self.declare_parameter('max_ttl_ms', 1000)
        self.declare_parameter('default_ttl_ms', 250)
        self.declare_parameter('shadow_mode', False)      # Default: False (Direct mode)
        self.declare_parameter('actuator_enabled', True)  # Default: True (Actuator output enabled)
        self.declare_parameter('telemetry_timeout_ms', 500)
        for limit_name, default_value in DEFAULT_LIMITS.items():
            self.declare_parameter(f'limits.{limit_name}', default_value)

        self.joint_names = self.get_parameter('joint_names').value
        self.protocol_version = self.get_parameter('protocol_version').value
        self.max_ttl_ms = self.get_parameter('max_ttl_ms').value
        self.default_ttl_ms = self.get_parameter('default_ttl_ms').value
        self.shadow_mode = self.get_parameter('shadow_mode').value
        self.actuator_enabled = self.get_parameter('actuator_enabled').value
        self.telemetry_timeout_s = float(self.get_parameter('telemetry_timeout_ms').value) / 1000.0

        resolved_limits = {
            limit_name: self.get_parameter(f'limits.{limit_name}').value
            for limit_name in DEFAULT_LIMITS
        }
        self.limits = validate_limits(resolved_limits)

        # Initialize safety state machine from the resolved bringup YAML.
        self.safety_state = DriverSafetyState(self.limits, max_ttl_ms=self.max_ttl_ms)

        # QoS Profiles
        best_effort_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', best_effort_qos)
        self.mcu_setpoint_pub = self.create_publisher(McuJointSetpoint, '/doghead/mcu/joint_setpoint', 10)
        self.mcu_motor_test_pub = self.create_publisher(McuMotorTestCommand, '/doghead/mcu/motor_test_command', 10)
        self.status_pub = self.create_publisher(String, '/doghead/driver/status', 10)
        self.session_state_pub = self.create_publisher(DriverSessionState, '/doghead/driver/session_state', 10)
        self.mcu_session_control_pub = self.create_publisher(
            McuSessionControl, '/doghead/mcu/session_control', 10)

        # Subscriptions (50 Hz high-rate telemetry uses BEST_EFFORT QoS for micro-ROS parity)
        self.mcu_state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_mcu_state, best_effort_qos)
        self.mcu_diagnostics_sub = self.create_subscription(
            McuDiagnostics, '/doghead/mcu/diagnostics', self._on_mcu_diagnostics, 10)
        self.joint_setpoint_sub = self.create_subscription(
            McuJointSetpoint, '/doghead/command/joint_setpoint', self._on_command_joint_setpoint, 10)
        self.motor_test_sub = self.create_subscription(
            McuMotorTestCommand, '/doghead/command/motor_test', self._on_command_motor_test, 10)

        # Services
        self.arm_srv = self.create_service(
            Trigger, '/doghead/driver/arm', self._srv_arm)
        self.disarm_srv = self.create_service(
            Trigger, '/doghead/driver/disarm', self._srv_disarm)
        self.stop_srv = self.create_service(
            Trigger, '/doghead/driver/stop', self._srv_stop)

        # Timer for periodic status publishing (1 Hz)
        self.status_timer = self.create_timer(1.0, self._publish_status)
        # State telemetry is 50 Hz; a 500 ms silence budget gives margin for
        # transport jitter while still failing closed automatically.
        self.telemetry_timeout_timer = self.create_timer(
            max(0.05, self.telemetry_timeout_s / 4.0),
            self._check_telemetry_timeout
        )

        self.mcu_out_sequence = 1
        self.get_logger().info("Doghead Driver Node initialized. Default state: DISARMED.")

    def _on_mcu_state(self, msg: McuState):
        """Process incoming McuState telemetry from MCU."""
        prev_armed = self.safety_state.armed

        # 1. Evaluate safety state & reconnect logic
        is_ok, status_msg = self.safety_state.on_mcu_state(
            protocol_version=msg.protocol_version,
            boot_id=msg.boot_id,
            agent_connected=msg.agent_connected,
            # McuState does not carry the reconnect counter; preserve the
            # diagnostics baseline instead of resetting it to zero at 50 Hz.
            reconnect_total=self.safety_state.last_reconnect_total,
            fault_code=msg.fault_code,
            now_s=self.get_clock().now().nanoseconds / 1e9,
            state_sequence=msg.state_sequence,
            uptime_ms=msg.uptime_ms,
        )

        if not is_ok:
            self.get_logger().warn(f"MCU State Safety Check Warning: {status_msg}")
            if prev_armed and not self.safety_state.armed:
                self._publish_status()  # Immediate notification of disarm/reconnect

        # 2. Never pass NaN/Inf telemetry into sensor_msgs.  Besides being
        # invalid ROS data, non-finite feedback must not keep an armed driver
        # alive during HIL fault containment.
        telemetry_vectors = (
            msg.measured_position_rad,
            msg.measured_velocity_rad_s,
            msg.commanded_motor_rpm,
        )
        if not all(is_finite_sequence(vector) for vector in telemetry_vectors):
            self.safety_state.disarm("MCU telemetry contains non-finite values")
            self.get_logger().error("Dropped non-finite MCU telemetry and disarmed driver")
            if prev_armed:
                self._publish_status()
            return

        # 3. Publish /joint_states from measured telemetry
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        # ROS generated float32 arrays expose NumPy scalars; JointState's
        # Python binding deliberately accepts only native float values.
        js.position = [float(value) for value in msg.measured_position_rad]
        js.velocity = [float(value) for value in msg.measured_velocity_rad_s]
        js.effort = [float(value) for value in msg.commanded_motor_rpm]
        self.joint_state_pub.publish(js)

    def _on_mcu_diagnostics(self, msg: McuDiagnostics):
        """Process MCU diagnostics telemetry."""
        if msg.agent_reconnect_total != self.safety_state.last_reconnect_total:
            self.get_logger().warn(
                f"Agent reconnect counter changed: {self.safety_state.last_reconnect_total} -> "
                f"{msg.agent_reconnect_total}")
            self.safety_state.disarm(
                f"Agent reconnect counter changed to {msg.agent_reconnect_total}")
            # Consume the new baseline, including a counter reset after MCU
            # reboot.  Ignoring decreases would allow an old session to look
            # healthy after the MCU restarted.
            self.safety_state.last_reconnect_total = msg.agent_reconnect_total
            self._publish_status()

    def _check_telemetry_timeout(self):
        ok, reason = self.safety_state.check_telemetry_timeout(
            now_s=self.get_clock().now().nanoseconds / 1e9,
            timeout_s=self.telemetry_timeout_s,
        )
        if not ok:
            self.get_logger().error(reason)
            self._publish_status()

    def _on_command_joint_setpoint(self, msg: McuJointSetpoint):
        """Validate and forward incoming joint setpoint commands."""
        valid, reason = self.safety_state.validate_joint_setpoint(
            protocol_version=msg.protocol_version,
            session_id=msg.session_id,
            sequence=msg.sequence,
            valid_for_ms=msg.valid_for_ms,
            enable_mask=msg.enable_mask,
            position_rad=list(msg.position_rad),
            velocity_rad_s=list(msg.velocity_rad_s),
            acceleration_rad_s2=list(msg.acceleration_rad_s2)
        )

        if not valid:
            self.get_logger().error(f"Rejected Joint Setpoint: {reason}")
            return

        # Prepare outgoing setpoint to MCU
        out_msg = McuJointSetpoint()
        out_msg.protocol_version = msg.protocol_version
        out_msg.session_id = self.safety_state.session_id
        out_msg.sequence = msg.sequence
        out_msg.valid_for_ms = msg.valid_for_ms

        # Two-Key Hard Gate check for physical actuator output
        can_enable = check_two_key_actuator_gate(
            armed=self.safety_state.armed,
            shadow_mode=self.shadow_mode,
            actuator_enabled=self.actuator_enabled
        )

        if can_enable:
            out_msg.enable_mask = msg.enable_mask & 0x07  # 3 axes mask limit
        else:
            out_msg.enable_mask = 0  # Physical actuators disarmed

        out_msg.position_rad = msg.position_rad
        out_msg.velocity_rad_s = msg.velocity_rad_s
        out_msg.acceleration_rad_s2 = msg.acceleration_rad_s2

        self.mcu_setpoint_pub.publish(out_msg)

    def _on_command_motor_test(self, msg: McuMotorTestCommand):
        """Validate and forward motor test bringup commands."""
        valid, reason = self.safety_state.validate_motor_test_command(
            protocol_version=msg.protocol_version,
            session_id=msg.session_id,
            sequence=msg.sequence,
            valid_for_ms=msg.valid_for_ms,
            axis=msg.axis,
            target_motor_rpm=msg.target_motor_rpm,
            requested_accel=msg.requested_accel_rpm_s,
            requested_jerk=msg.requested_jerk_rpm_s2,
            enable=msg.enable
        )

        if not valid:
            self.get_logger().error(f"Rejected Motor Test Command: {reason}")
            return

        out_msg = McuMotorTestCommand()
        out_msg.protocol_version = msg.protocol_version
        out_msg.session_id = self.safety_state.session_id
        out_msg.sequence = self.mcu_out_sequence
        self.mcu_out_sequence = (self.mcu_out_sequence + 1) & 0xFFFFFFFF
        out_msg.valid_for_ms = msg.valid_for_ms
        out_msg.axis = msg.axis
        out_msg.target_motor_rpm = msg.target_motor_rpm
        out_msg.requested_accel_rpm_s = msg.requested_accel_rpm_s
        out_msg.requested_jerk_rpm_s2 = msg.requested_jerk_rpm_s2

        can_enable = check_two_key_actuator_gate(
            armed=self.safety_state.armed,
            shadow_mode=self.shadow_mode,
            actuator_enabled=self.actuator_enabled
        )

        if can_enable:
            out_msg.enable = msg.enable
        else:
            out_msg.enable = False

        self.mcu_motor_test_pub.publish(out_msg)

    def _srv_arm(self, request, response):
        """Service callback to ARM the driver."""
        success, session_id, msg = self.safety_state.arm()
        response.success = success
        response.message = msg
        if success:
            self.get_logger().info(f"Driver ARMED. Active Session ID: {session_id}")
            ctrl_msg = McuSessionControl()
            ctrl_msg.protocol_version = self.protocol_version
            ctrl_msg.session_id = session_id
            ctrl_msg.armed = True
            for _ in range(3):
                self.mcu_session_control_pub.publish(ctrl_msg)
        else:
            self.get_logger().warn(f"Arming failed: {msg}")
        self._publish_status()
        return response

    def _srv_disarm(self, request, response):
        """Service callback to DISARM the driver."""
        self.safety_state.disarm("User disarm request")
        ctrl_msg = McuSessionControl()
        ctrl_msg.protocol_version = self.protocol_version
        ctrl_msg.session_id = 0
        ctrl_msg.armed = False
        self.mcu_session_control_pub.publish(ctrl_msg)
        response.success = True
        response.message = "Driver DISARMED"
        self.get_logger().info("Driver DISARMED.")
        self._publish_status()
        return response

    def _srv_stop(self, request, response):
        """Service callback to STOP & DISARM immediately."""
        self.safety_state.stop("User STOP request")
        ctrl_msg = McuSessionControl()
        ctrl_msg.protocol_version = self.protocol_version
        ctrl_msg.session_id = 0
        ctrl_msg.armed = False
        self.mcu_session_control_pub.publish(ctrl_msg)
        response.success = True
        response.message = "STOP action executed. Driver DISARMED."
        self.get_logger().warn("STOP action executed.")
        self._publish_status()
        return response

    def _publish_status(self):
        """Publish driver safety status and versioned session state periodically or on change."""
        # 1. Publish String status
        status_str = (
            f"Armed: {self.safety_state.armed} | "
            f"Session: {self.safety_state.session_id} | "
            f"AgentConnected: {self.safety_state.agent_connected} | "
            f"ShadowMode: {self.shadow_mode} | "
            f"ActuatorEnabled: {self.actuator_enabled} | "
            f"LastReason: '{self.safety_state.disarm_reason}'"
        )
        msg_str = String()
        msg_str.data = status_str
        self.status_pub.publish(msg_str)

        # 2. Publish DriverSessionState message for motion node session handoff
        sess_msg = DriverSessionState()
        sess_msg.header.stamp = self.get_clock().now().to_msg()
        sess_msg.protocol_version = self.protocol_version
        sess_msg.session_id = self.safety_state.session_id
        sess_msg.armed = self.safety_state.armed
        sess_msg.agent_connected = self.safety_state.agent_connected
        sess_msg.shadow_mode = self.shadow_mode
        sess_msg.actuator_enabled = self.actuator_enabled
        sess_msg.disarm_reason = self.safety_state.disarm_reason or "OK"
        self.session_state_pub.publish(sess_msg)

        control_msg = McuSessionControl()
        control_msg.protocol_version = self.protocol_version
        control_msg.session_id = self.safety_state.session_id
        control_msg.armed = self.safety_state.armed
        self.mcu_session_control_pub.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DogheadDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safety_state.stop("Node shutdown")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
