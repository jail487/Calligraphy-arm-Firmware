import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from doghead_interfaces.msg import EndEffectorReference, McuJointSetpoint, DriverSessionState, McuState
from doghead_motion.kinematics import DogheadKinematics, DEFAULT_JOINT_LIMITS
from doghead_motion.trajectory_generator import JointPvaTrajectoryGenerator
from doghead_driver.safety_validator import is_sequence_newer


class DogheadMotionNode(Node):
    """
    ROS 2 Motion Adapter Node for doghead arm.
    Validates SI frame coordinates, executes 3-axis IK, checks reachability/limits,
    subscribes to versioned driver session state for explicit arm-session handoff,
    and maps accepted references strictly to McuJointSetpoint topics.
    Outputs remain shadow-safe (enable_mask=0) unless an active driver session exists.

    McuState freshness validation:
    - Rejects non-finite measured_position_rad or measured_velocity_rad_s.
    - Rejects messages with agent_connected=False.
    - Detects boot_id changes and invalidates cached measured state on reboot.
    - Rejects replayed/stale messages whose state_sequence or uptime_ms did not
      advance since the last accepted sample (using 32-bit wrap-safe comparison).
    - Invalidates measured state on driver session disarm so that a new session
      must confirm fresh state before trajectories are accepted.
    """

    def __init__(self):
        super().__init__('doghead_motion_node')

        # Parameters
        self.declare_parameter('link_lengths', [0.20, 0.18, 0.10])
        self.declare_parameter('protocol_version', 1)
        self.declare_parameter('valid_for_ms', 250)
        self.declare_parameter('trajectory_rate_hz', 100.0)
        self.declare_parameter('measured_state_timeout_ms', 500)
        for limit_name, default_value in DEFAULT_JOINT_LIMITS.items():
            self.declare_parameter(f'limits.{limit_name}', default_value)

        link_lengths = self.get_parameter('link_lengths').value
        self.protocol_version = self.get_parameter('protocol_version').value
        self.valid_for_ms = self.get_parameter('valid_for_ms').value
        self.trajectory_rate_hz = float(self.get_parameter('trajectory_rate_hz').value)
        self.measured_state_timeout_s = float(self.get_parameter('measured_state_timeout_ms').value) / 1000.0
        if not math.isfinite(self.trajectory_rate_hz) or self.trajectory_rate_hz <= 0.0:
            raise ValueError('trajectory_rate_hz must be positive and finite')
        if not math.isfinite(self.measured_state_timeout_s) or self.measured_state_timeout_s <= 0.0:
            raise ValueError('measured_state_timeout_ms must be positive and finite')

        joint_limits = {
            limit_name: float(self.get_parameter(f'limits.{limit_name}').value)
            for limit_name in DEFAULT_JOINT_LIMITS
        }
        if not all(math.isfinite(value) for value in joint_limits.values()):
            raise ValueError('all joint limits must be finite')
        if any(joint_limits[f'q{idx}_min'] > joint_limits[f'q{idx}_max'] for idx in (1, 2, 3)):
            raise ValueError('joint limits must be ordered min <= max')

        self.kinematics = DogheadKinematics(link_lengths=link_lengths, limits=joint_limits)
        self.traj_gen = JointPvaTrajectoryGenerator()

        import sys
        import os
        sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))
        try:
            from robot_constants import HOME_STANDBY_THETA1_DEG, HOME_STANDBY_THETA2_DEG, HOME_STANDBY_THETA3_DEG
        except ImportError:
            HOME_STANDBY_THETA1_DEG = 111.0
            HOME_STANDBY_THETA2_DEG = 180.0
            HOME_STANDBY_THETA3_DEG = 0.0
            
        # Start from the confirmed home pose from SSoT.
        # This is a safe fallback, never an optimistic endpoint cache.
        self.current_joint_pos = [
            math.radians(HOME_STANDBY_THETA1_DEG), 
            math.radians(HOME_STANDBY_THETA2_DEG), 
            math.radians(HOME_STANDBY_THETA3_DEG)
        ]
        self.measured_state_valid = False
        self.last_measured_state_time_s = None

        # McuState freshness tracking fields.
        # These are used to detect replays, reboots, and stale messages.
        # Sentinel values indicate "no sample accepted yet in this session".
        self._last_accepted_boot_id = None      # None = no sample seen yet
        self._last_accepted_sequence = None     # None = no sample seen yet
        self._last_accepted_uptime_ms = None    # None = no sample seen yet

        self._trajectory_active = False
        self._trajectory_start = list(self.current_joint_pos)
        self._trajectory_target = list(self.current_joint_pos)
        self._trajectory_elapsed_s = 0.0
        self._trajectory_duration_s = 0.0
        self._trajectory_start_time_s = None
        # _last_generated_position is retained for telemetry/debug only.
        # It is NOT used as a preemption start point; measured/applied state is.
        self._last_generated_position = list(self.current_joint_pos)
        self._last_generated_velocity = [0.0, 0.0, 0.0]
        self._last_generated_acceleration = [0.0, 0.0, 0.0]
        self.sequence_counter = 1

        # Session Handoff State (Default: Disarmed / Session 0)
        self.active_session_id = 0
        self.driver_armed = False

        # Publisher to driver setpoint input topic
        self.setpoint_pub = self.create_publisher(McuJointSetpoint, '/doghead/command/joint_setpoint', 10)
        self.status_pub = self.create_publisher(String, '/doghead/motion/status', 10)

        # Subscriptions
        self.create_subscription(EndEffectorReference, '/doghead/motion/ef_reference', self._on_ef_reference, 10)
        self.create_subscription(DriverSessionState, '/doghead/driver/session_state', self._on_driver_session_state, 10)
        best_effort_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(McuState, '/doghead/mcu/state', self._on_mcu_state, best_effort_qos)
        self.create_timer(1.0 / self.trajectory_rate_hz, self._publish_trajectory_sample)

        self.get_logger().info("Doghead Motion Node initialized. Waiting for explicit driver session handoff...")

    def _stop_and_disarm(self, reason: str = "Telemetry invalidated"):
        """
        Stop active trajectory, clear driver session handoff and telemetry validity,
        and publish a bounded zero-enable stop setpoint command without endpoint teleportation.
        """
        was_active_or_armed = self._trajectory_active or self.driver_armed
        self._trajectory_active = False
        self.driver_armed = False
        self.active_session_id = 0
        self.measured_state_valid = False
        self._last_accepted_boot_id = None
        self._last_accepted_sequence = None
        self._last_accepted_uptime_ms = None

        if was_active_or_armed:
            setpoint = McuJointSetpoint()
            setpoint.protocol_version = self.protocol_version
            setpoint.session_id = 0
            setpoint.sequence = self.sequence_counter
            self.sequence_counter = 1 if self.sequence_counter >= 0xFFFFFFFF else self.sequence_counter + 1
            setpoint.valid_for_ms = self.valid_for_ms
            setpoint.enable_mask = 0x00
            setpoint.position_rad = [float(p) for p in self._last_generated_position]
            setpoint.velocity_rad_s = [0.0, 0.0, 0.0]
            setpoint.acceleration_rad_s2 = [0.0, 0.0, 0.0]
            self.setpoint_pub.publish(setpoint)

    def _on_mcu_state(self, msg: McuState):
        """
        Validate and consume fresh, session-safe measured MCU state.

        Validation rejects a message if ANY of the following apply:
        1. protocol_version does not match node protocol_version.
        2. boot_id is 0 or unknown.
        3. measured_position_rad or measured_velocity_rad_s are non-finite.
        4. agent_connected is False (MCU has lost connection to the control agent).
        5. boot_id changed since the last accepted sample (MCU rebooted mid-session).
        6. state_sequence did not advance (replayed or stale packet, wrap-safe).
        7. uptime_ms did not advance (duplicate or time-reversed packet, wrap-safe).

        On a detected reboot (boot_id change), protocol mismatch, invalid boot_id=0,
        or invalid telemetry while active/armed, the active trajectory is stopped and
        driver session state is cleared.
        """
        # --- Guard 1: protocol_version match ---
        incoming_protocol = int(msg.protocol_version)
        if incoming_protocol != self.protocol_version:
            self.get_logger().warn(
                f"Protocol version mismatch in MCU state: got {incoming_protocol}, expected {self.protocol_version}. "
                "Stopping trajectory and disarming session."
            )
            self._stop_and_disarm(f"Protocol version mismatch ({incoming_protocol} != {self.protocol_version})")
            return

        # --- Guard 2: boot_id must be non-zero / known ---
        incoming_boot_id = int(msg.boot_id)
        if incoming_boot_id == 0:
            self.get_logger().warn("MCU telemetry reports boot_id=0 (unknown/invalid). Stopping trajectory and disarming session.")
            self._stop_and_disarm("Invalid MCU boot_id=0")
            return

        # --- Guard 3: finite position and velocity ---
        measured_pos = list(msg.measured_position_rad)
        measured_vel = list(msg.measured_velocity_rad_s)
        if (len(measured_pos) != 3 or len(measured_vel) != 3
                or not all(math.isfinite(float(v)) for v in measured_pos)
                or not all(math.isfinite(float(v)) for v in measured_vel)):
            self._stop_and_disarm("MCU telemetry non-finite")
            return

        # --- Guard 4: agent must be connected ---
        if not msg.agent_connected:
            self._stop_and_disarm("MCU reports agent disconnected")
            return

        # --- Guard 5: boot_id change detection (MCU rebooted) ---
        if self._last_accepted_boot_id is not None and incoming_boot_id != self._last_accepted_boot_id:
            self.get_logger().warn(
                f"MCU reboot detected (boot_id {self._last_accepted_boot_id} -> {incoming_boot_id}). "
                "Invalidating measured state and stopping active trajectory."
            )
            self._stop_and_disarm(f"MCU reboot detected (boot_id changed to {incoming_boot_id})")
            return

        # --- Guard 6: sequence must advance (wrap-safe check to reject replays) ---
        incoming_seq = int(msg.state_sequence) & 0xFFFFFFFF
        if self._last_accepted_sequence is not None and not is_sequence_newer(incoming_seq, self._last_accepted_sequence):
            # Stale or replayed packet — do not update freshness.
            return

        # --- Guard 7: uptime_ms must advance (wrap-safe check to reject time-reversed / duplicate) ---
        incoming_uptime = int(msg.uptime_ms) & 0xFFFFFFFF
        if self._last_accepted_uptime_ms is not None and not is_sequence_newer(incoming_uptime, self._last_accepted_uptime_ms):
            return

        # --- All guards passed: accept this sample ---
        self._last_accepted_boot_id = incoming_boot_id
        self._last_accepted_sequence = incoming_seq
        self._last_accepted_uptime_ms = incoming_uptime

        # Store both measured and applied positions.
        # applied_position_rad is what the MCU last commanded its interpolator to;
        # it is used as the preemption start when a trajectory is active.
        self.current_joint_pos = [float(v) for v in measured_pos]

        applied = list(msg.applied_position_rad)
        if (len(applied) == 3 and all(math.isfinite(float(v)) for v in applied)):
            self._last_applied_pos = [float(v) for v in applied]
        else:
            # If the MCU has not yet applied anything, fall back to measured.
            self._last_applied_pos = list(self.current_joint_pos)

        self.measured_state_valid = True
        self.last_measured_state_time_s = self.get_clock().now().nanoseconds / 1e9

    def _on_driver_session_state(self, msg: DriverSessionState):
        """Receive versioned driver session handoff and handle disarm/reconnect invalidations."""
        if msg.armed and msg.protocol_version == self.protocol_version and msg.session_id > 0:
            if not self.driver_armed or self.active_session_id != msg.session_id:
                self.get_logger().info(f"Driver session handoff received. Active Session ID: {msg.session_id}")
            self.active_session_id = msg.session_id
            self.driver_armed = True
        else:
            if self.driver_armed or self._trajectory_active:
                self.get_logger().warn(f"Driver session invalidated (Reason: '{msg.disarm_reason}'). Motion reset to DISARMED.")
            self._stop_and_disarm(f"Driver session disarmed: {msg.disarm_reason}")

    def _on_ef_reference(self, msg: EndEffectorReference):
        """Handle incoming End-Effector reference command."""
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        # 1. Perform IK and reachability check
        ok, joint_angles, status_msg = self.kinematics.inverse_kinematics(x, y, z)
        if not ok:
            self.get_logger().warn(f"Rejected EF Reference ({x:.3f}, {y:.3f}, {z:.3f}): {status_msg}")
            status_out = String()
            status_out.data = f"REJECTED: {status_msg}"
            self.status_pub.publish(status_out)
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        measured_age_s = (
            float('inf') if self.last_measured_state_time_s is None
            else now_s - self.last_measured_state_time_s
        )
        if (not self.measured_state_valid or not math.isfinite(measured_age_s) or
                measured_age_s < 0.0 or measured_age_s > self.measured_state_timeout_s):
            status_out = String()
            status_out.data = "REJECTED: waiting for fresh measured MCU state"
            self.status_pub.publish(status_out)
            self.get_logger().warn(status_out.data)
            return

        # 2. Start a persistent 100 Hz trajectory stream.
        #
        #    Preemption start point is always a VALIDATED measured/applied state,
        #    never an optimistic last-generated sample.  Using generated samples
        #    would treat uncommitted motor output as confirmed executed state,
        #    violating the constraint that only driver/firmware safety gates are
        #    authoritative on actual motor position.
        #
        #    - Idle arm: start from current measured position.
        #    - Active trajectory being preempted: start from last applied position
        #      (what the MCU's interpolator last received as a setpoint command),
        #      which is a confirmed hardware-acknowledged state rather than a
        #      speculative generated sample.
        if self._trajectory_active and hasattr(self, '_last_applied_pos'):
            preempt_start = list(self._last_applied_pos)
        else:
            preempt_start = list(self.current_joint_pos)

        self._trajectory_start = preempt_start
        self._trajectory_target = list(joint_angles)
        self._trajectory_elapsed_s = 0.0
        self._trajectory_duration_s = self.traj_gen.compute_motion_time(
            self._trajectory_start, self._trajectory_target)
        self._trajectory_start_time_s = now_s
        self._trajectory_active = True

        self._publish_trajectory_sample()

        # 3. Session handoff & shadow safety check
        if not self.driver_armed or self.active_session_id == 0:
            status_note = " (SHADOW-SAFE: Unarmed/No Driver Session)"
        else:
            status_note = ""

        status_out = String()
        status_out.data = f"ACCEPTED: queued trajectory to ({joint_angles[0]:.3f}, {joint_angles[1]:.3f}, {joint_angles[2]:.3f}) rad{status_note}"
        self.status_pub.publish(status_out)

    def _publish_trajectory_sample(self):
        if not self._trajectory_active:
            return

        # Recheck telemetry validity and freshness at each sample.
        now_s = self.get_clock().now().nanoseconds / 1e9
        measured_age_s = (
            float('inf') if self.last_measured_state_time_s is None
            else now_s - self.last_measured_state_time_s
        )

        is_telemetry_fresh = (
            self.measured_state_valid and
            math.isfinite(measured_age_s) and
            0.0 <= measured_age_s <= self.measured_state_timeout_s
        )

        if not is_telemetry_fresh:
            self.get_logger().warn(
                f"Trajectory sample blocked: telemetry fresh={is_telemetry_fresh} "
                f"(age={measured_age_s:.3f}s). Aborting trajectory."
            )
            self._stop_and_disarm("Telemetry stale or invalid during trajectory publication")
            return

        if self._trajectory_start_time_s is not None:
            self._trajectory_elapsed_s = max(0.0, min(
                self._trajectory_duration_s,
                now_s - self._trajectory_start_time_s,
            ))

        pos_rad, vel_rad_s, accel_rad_s2 = self.traj_gen.sample_trajectory_at_time(
            start_pos=self._trajectory_start,
            target_pos=self._trajectory_target,
            elapsed_s=self._trajectory_elapsed_s,
        )

        # 100 Hz command sampling; shadow mode (unarmed driver session) emits
        # session_id=0 and enable_mask=0x00 so physical output is disabled.
        if not self.driver_armed or self.active_session_id == 0:
            session_id = 0
            enable_mask = 0x00
        else:
            session_id = self.active_session_id
            enable_mask = 0x07

        setpoint = McuJointSetpoint()
        setpoint.protocol_version = self.protocol_version
        setpoint.session_id = session_id
        setpoint.sequence = self.sequence_counter
        self.sequence_counter = 1 if self.sequence_counter >= 0xFFFFFFFF else self.sequence_counter + 1
        setpoint.valid_for_ms = self.valid_for_ms
        setpoint.enable_mask = enable_mask

        setpoint.position_rad = [float(p) for p in pos_rad]
        setpoint.velocity_rad_s = [float(v) for v in vel_rad_s]
        setpoint.acceleration_rad_s2 = [float(a) for a in accel_rad_s2]

        self.setpoint_pub.publish(setpoint)

        self._last_generated_position = list(pos_rad)
        self._last_generated_velocity = list(vel_rad_s)
        self._last_generated_acceleration = list(accel_rad_s2)
        if self._trajectory_elapsed_s >= self._trajectory_duration_s:
            self._trajectory_active = False


def main(args=None):
    rclpy.init(args=args)
    node = DogheadMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
