"""
Tests for motion node session handoff, McuState freshness validation,
preemption start provenance, and shadow-safe setpoint output.

All tests that exercise DogheadMotionNode are skipped when rclpy / a ROS 2
runtime is unavailable (non_goals: do not claim ROS 2 runtime verification).
"""

import math
import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy / ROS 2 runtime not available")

from doghead_motion.motion_node import DogheadMotionNode
from doghead_interfaces.msg import DriverSessionState, EndEffectorReference, McuState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_node():
    """Create a fresh DogheadMotionNode; caller must call destroy_node()."""
    return DogheadMotionNode()


def _arm_session(node, session_id=42):
    """Send a valid armed DriverSessionState to the node."""
    msg = DriverSessionState()
    msg.protocol_version = 1
    msg.session_id = session_id
    msg.armed = True
    msg.agent_connected = True
    msg.shadow_mode = False
    msg.actuator_enabled = True
    msg.disarm_reason = "OK"
    node._on_driver_session_state(msg)


def _disarm_session(node, reason="MCU reconnect detected"):
    """Send a disarmed DriverSessionState to the node."""
    msg = DriverSessionState()
    msg.protocol_version = 1
    msg.session_id = 0
    msg.armed = False
    msg.disarm_reason = reason
    node._on_driver_session_state(msg)


def _make_fresh_mcu_state(boot_id=1, sequence=10, uptime_ms=5000,
                           agent_connected=True,
                           pos=None, vel=None, applied=None,
                           protocol_version=1):
    """Build a valid McuState message."""
    msg = McuState()
    msg.protocol_version = protocol_version
    msg.boot_id = boot_id
    msg.state_sequence = sequence
    msg.uptime_ms = uptime_ms
    msg.agent_connected = agent_connected
    msg.measured_position_rad = pos if pos is not None else [2.0, 1.0, 0.0]
    msg.measured_velocity_rad_s = vel if vel is not None else [0.0, 0.0, 0.0]
    msg.applied_position_rad = applied if applied is not None else [2.0, 1.0, 0.0]
    return msg


_REACHABLE_EF = {"x": -0.09191636, "y": 0.19711528, "z": 0.20455676}


def _make_ef_reference(x=None, y=None, z=None):
    ef = EndEffectorReference()
    ef.position.x = x if x is not None else _REACHABLE_EF["x"]
    ef.position.y = y if y is not None else _REACHABLE_EF["y"]
    ef.position.z = z if z is not None else _REACHABLE_EF["z"]
    return ef


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def rclpy_session():
    """Initialize and shutdown rclpy around each test."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture()
def node():
    n = _make_node()
    yield n
    n.destroy_node()


# ---------------------------------------------------------------------------
# Original tests (preserved)
# ---------------------------------------------------------------------------

def test_motion_session_handoff_and_invalidation(node):
    """Armed session activates, then disarm resets to shadow-safe defaults."""
    # 1. Initial State: Unarmed, active_session_id = 0
    assert node.driver_armed is False
    assert node.active_session_id == 0

    # 2. Receive valid driver session handoff (Armed, session_id = 42)
    _arm_session(node, session_id=42)
    assert node.driver_armed is True
    assert node.active_session_id == 42

    # 3. Provide valid MCU state, then send EF reference
    node._on_mcu_state(_make_fresh_mcu_state())

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)

    node._on_ef_reference(_make_ef_reference())

    assert len(published) == 1
    assert published[0].session_id == 42
    assert published[0].enable_mask == 0x07

    # 4. Disarm
    _disarm_session(node)
    assert node.driver_armed is False
    assert node.active_session_id == 0

    # 5. After disarm, measured_state_valid must be False (freshness invalidated)
    assert node.measured_state_valid is False

    # 6. Re-inject fresh MCU state so the EF reference is not rejected for staleness
    node._on_mcu_state(_make_fresh_mcu_state(sequence=20, uptime_ms=6000))

    published.clear()
    node._on_ef_reference(_make_ef_reference())

    assert len(published) == 1
    assert published[0].session_id == 0
    assert published[0].enable_mask == 0x00


def test_motion_mcu_state_subscription_matches_firmware_best_effort_qos(node):
    """McuState subscription must use BEST_EFFORT QoS to match firmware publisher."""
    from rclpy.qos import QoSReliabilityPolicy
    state_sub = [sub for sub in node.subscriptions if sub.topic_name == '/doghead/mcu/state'][0]
    assert state_sub.qos_profile.reliability == QoSReliabilityPolicy.BEST_EFFORT


# ---------------------------------------------------------------------------
# McuState freshness validation — new tests
# ---------------------------------------------------------------------------

def test_mcu_state_non_finite_position_rejected(node):
    """A message with a NaN in measured_position_rad must be rejected."""
    msg = _make_fresh_mcu_state(pos=[float('nan'), 0.0, 0.0])
    node._on_mcu_state(msg)
    assert node.measured_state_valid is False


def test_mcu_state_non_finite_velocity_rejected(node):
    """A message with Inf in measured_velocity_rad_s must be rejected."""
    msg = _make_fresh_mcu_state(vel=[0.0, float('inf'), 0.0])
    node._on_mcu_state(msg)
    assert node.measured_state_valid is False


def test_mcu_state_agent_disconnected_rejected(node):
    """A message with agent_connected=False must be rejected and invalidate state."""
    # First accept a valid sample so we have a baseline.
    node._on_mcu_state(_make_fresh_mcu_state(sequence=1, uptime_ms=1000))
    assert node.measured_state_valid is True

    # Then receive a sample with agent_connected=False.
    node._on_mcu_state(_make_fresh_mcu_state(sequence=2, uptime_ms=2000, agent_connected=False))
    assert node.measured_state_valid is False


def test_mcu_state_replayed_sequence_rejected(node):
    """A repeated or lower state_sequence must be silently rejected (no freshness update)."""
    node._on_mcu_state(_make_fresh_mcu_state(sequence=10, uptime_ms=5000))
    assert node.measured_state_valid is True
    t1 = node.last_measured_state_time_s

    # Same sequence — replay
    node._on_mcu_state(_make_fresh_mcu_state(sequence=10, uptime_ms=6000))
    # freshness timestamp must not have changed
    assert node.last_measured_state_time_s == t1

    # Lower sequence — replay
    node._on_mcu_state(_make_fresh_mcu_state(sequence=5, uptime_ms=7000))
    assert node.last_measured_state_time_s == t1


def test_mcu_state_non_advancing_uptime_rejected(node):
    """A message whose uptime_ms did not advance must be rejected."""
    node._on_mcu_state(_make_fresh_mcu_state(sequence=10, uptime_ms=5000))
    t1 = node.last_measured_state_time_s

    # sequence advanced but uptime identical
    node._on_mcu_state(_make_fresh_mcu_state(sequence=11, uptime_ms=5000))
    assert node.last_measured_state_time_s == t1


def test_mcu_state_boot_id_change_invalidates_and_allows_new_session(node):
    """A boot_id change means the MCU rebooted: current measured state is invalidated.
    The first packet of the new boot session must then be accepted as a fresh baseline."""
    # Accept a sample from boot 1.
    node._on_mcu_state(_make_fresh_mcu_state(boot_id=1, sequence=100, uptime_ms=50000))
    assert node.measured_state_valid is True

    # Receive a packet from boot 2 (reboot).
    node._on_mcu_state(_make_fresh_mcu_state(boot_id=2, sequence=200, uptime_ms=100))
    # The reboot packet itself is rejected to prevent a stale/glitch from being accepted.
    assert node.measured_state_valid is False

    # The NEXT packet from boot 2 must be accepted as the new baseline.
    node._on_mcu_state(_make_fresh_mcu_state(boot_id=2, sequence=201, uptime_ms=200))
    assert node.measured_state_valid is True
    assert node._last_accepted_boot_id == 2


def test_mcu_state_session_disarm_resets_freshness_sentinels(node):
    """After a driver disarm, freshness sentinels are cleared so the next boot
    sample can re-establish the baseline without being flagged as a replay."""
    _arm_session(node, session_id=1)
    node._on_mcu_state(_make_fresh_mcu_state(boot_id=1, sequence=50, uptime_ms=25000))
    assert node._last_accepted_sequence == 50

    _disarm_session(node)
    assert node._last_accepted_boot_id is None
    assert node._last_accepted_sequence is None
    assert node._last_accepted_uptime_ms is None
    assert node.measured_state_valid is False


# ---------------------------------------------------------------------------
# Preemption start provenance tests
# ---------------------------------------------------------------------------

def test_preemption_starts_from_applied_not_generated(node):
    """When a trajectory is active, preempting it must start from the MCU's
    last applied position — not from _last_generated_position."""
    _arm_session(node, session_id=7)

    measured_pos = [2.0, 1.0, 0.0]
    applied_pos = [1.9, 0.95, 0.05]  # MCU applied slightly different
    node._on_mcu_state(_make_fresh_mcu_state(
        pos=measured_pos, applied=applied_pos, sequence=1, uptime_ms=1000))

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)

    # First EF reference — starts a trajectory
    node._on_ef_reference(_make_ef_reference())
    assert node._trajectory_active is True

    # Corrupt _last_generated_position to a clearly wrong value so we can
    # detect if it is erroneously used as the preemption start.
    node._last_generated_position = [99.0, 99.0, 99.0]

    # Simulate incoming updated applied state mid-trajectory
    new_applied = [1.5, 0.8, 0.1]
    node._on_mcu_state(_make_fresh_mcu_state(
        pos=[1.5, 0.8, 0.1], applied=new_applied, sequence=2, uptime_ms=2000))

    # Second EF reference — preempts the active trajectory
    node._on_ef_reference(_make_ef_reference(x=-0.10, y=0.18, z=0.22))

    # The preemption start must be close to the applied position, not 99.x
    for val in node._trajectory_start:
        assert val < 10.0, (
            f"Preemption start {node._trajectory_start} appears to be from "
            "_last_generated_position (99.x), not from measured/applied state."
        )


def test_preemption_idle_starts_from_measured(node):
    """When no trajectory is active, the start point is the measured position."""
    _arm_session(node, session_id=3)

    measured_pos = [1.0, 0.5, 0.2]
    node._on_mcu_state(_make_fresh_mcu_state(pos=measured_pos, sequence=1, uptime_ms=500))

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)

    node._on_ef_reference(_make_ef_reference())

    # Trajectory start must match the measured position within ROS float32 conversion tolerance.
    for i in range(3):
        assert abs(node._trajectory_start[i] - measured_pos[i]) < 1e-6, (
            f"Expected trajectory start[{i}]={measured_pos[i]}, got {node._trajectory_start[i]}"
        )


# ---------------------------------------------------------------------------
# Shadow-safe setpoint output
# ---------------------------------------------------------------------------

def test_shadow_safe_enable_mask_when_unarmed(node):
    """Without an active driver session, enable_mask must be 0x00 (shadow-safe)."""
    # Inject fresh state (no armed session)
    node._on_mcu_state(_make_fresh_mcu_state(sequence=1, uptime_ms=1000))

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)
    node._on_ef_reference(_make_ef_reference())

    assert len(published) == 1
    assert published[0].enable_mask == 0x00
    assert published[0].session_id == 0


def test_shadow_safe_enable_mask_when_armed(node):
    """With an active armed driver session, enable_mask must be 0x07."""
    _arm_session(node, session_id=99)
    node._on_mcu_state(_make_fresh_mcu_state(sequence=1, uptime_ms=1000))

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)
    node._on_ef_reference(_make_ef_reference())

    assert len(published) == 1
    assert published[0].enable_mask == 0x07
    assert published[0].session_id == 99


# ---------------------------------------------------------------------------
# T02 Fail-closed & Wrap-safe rollover tests
# ---------------------------------------------------------------------------

def test_active_trajectory_aborted_on_telemetry_invalidation(node):
    """If telemetry becomes invalid/disconnected while trajectory is active, node stops trajectory immediately."""
    _arm_session(node, session_id=12)
    node._on_mcu_state(_make_fresh_mcu_state(sequence=1, uptime_ms=1000))

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)
    node._on_ef_reference(_make_ef_reference())

    assert node._trajectory_active is True
    published.clear()

    # Telemetry becomes invalid (agent disconnected)
    node._on_mcu_state(_make_fresh_mcu_state(sequence=2, uptime_ms=2000, agent_connected=False))

    assert node._trajectory_active is False
    assert node.driver_armed is False
    assert node.active_session_id == 0
    assert node.measured_state_valid is False
    assert len(published) == 1
    assert published[0].enable_mask == 0x00
    assert published[0].session_id == 0


def test_timer_path_rechecks_freshness_without_disarm_msg(node):
    """The timer path rechecks freshness at each sample; stale telemetry stops trajectory without driver disarm."""
    _arm_session(node, session_id=55)
    node._on_mcu_state(_make_fresh_mcu_state(sequence=1, uptime_ms=1000))

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)
    node._on_ef_reference(_make_ef_reference())

    assert node._trajectory_active is True
    published.clear()

    # Simulate clock advancing beyond timeout (500 ms default timeout)
    node.last_measured_state_time_s = (node.get_clock().now().nanoseconds / 1e9) - 1.0

    # Timer tick runs
    node._publish_trajectory_sample()

    assert node._trajectory_active is False
    assert node.driver_armed is False
    assert node.active_session_id == 0
    assert len(published) == 1
    assert published[0].enable_mask == 0x00
    assert published[0].session_id == 0

    # Subsequent timer ticks do not publish (bounded)
    published.clear()
    node._publish_trajectory_sample()
    assert len(published) == 0


def test_stop_path_publishes_bounded_stop_without_endpoint_teleportation(node):
    """The stop path publishes a zero-enable setpoint without setting current_joint_pos to endpoint."""
    _arm_session(node, session_id=88)
    initial_pos = [2.0, 1.0, 0.0]
    node._on_mcu_state(_make_fresh_mcu_state(pos=initial_pos, sequence=1, uptime_ms=1000))

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)
    node._on_ef_reference(_make_ef_reference())

    assert node._trajectory_active is True

    # Stop and disarm
    node._stop_and_disarm("Manual test stop")

    assert node._trajectory_active is False
    assert node.current_joint_pos == initial_pos
    assert node.current_joint_pos != node._trajectory_target

    assert len(published) > 0
    stop_msg = published[-1]
    assert stop_msg.enable_mask == 0x00
    assert stop_msg.session_id == 0


def test_mcu_state_sequence_and_uptime_wrap_around_accepted(node):
    """Sequence and uptime comparisons use 32-bit wrap-safe semantics across 0xFFFFFFFF boundary."""
    # First sample near uint32 max
    node._on_mcu_state(_make_fresh_mcu_state(sequence=0xFFFFFFFF, uptime_ms=0xFFFFFFF0))
    assert node.measured_state_valid is True
    t1 = node.last_measured_state_time_s

    # Next sample wraps to 10 / 100
    node._on_mcu_state(_make_fresh_mcu_state(sequence=10, uptime_ms=100))
    assert node.measured_state_valid is True
    assert node.last_measured_state_time_s > t1
    assert node._last_accepted_sequence == 10
    assert node._last_accepted_uptime_ms == 100


def test_command_sequence_rollover_is_monotonic_under_driver_validator(node):
    """Command sequence rollover from 0xFFFFFFFF to 1 is wrap-safe for driver validator."""
    from doghead_driver.safety_validator import DriverSafetyState

    driver_state = DriverSafetyState()
    driver_state.on_mcu_state(protocol_version=1, boot_id=1, agent_connected=True)
    ok, session_id, _ = driver_state.arm()
    assert ok

    _arm_session(node, session_id=session_id)
    node._on_mcu_state(_make_fresh_mcu_state(sequence=1, uptime_ms=1000))

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)

    # Set sequence counter near rollover
    node.sequence_counter = 0xFFFFFFFF
    node._on_ef_reference(_make_ef_reference())

    assert len(published) == 1
    msg1 = published[0]
    assert msg1.sequence == 0xFFFFFFFF

    # Validate first command at driver
    valid1, _ = driver_state.validate_joint_setpoint(
        protocol_version=msg1.protocol_version,
        session_id=msg1.session_id,
        sequence=msg1.sequence,
        valid_for_ms=msg1.valid_for_ms,
        enable_mask=msg1.enable_mask,
        position_rad=list(msg1.position_rad),
        velocity_rad_s=list(msg1.velocity_rad_s),
        acceleration_rad_s2=list(msg1.acceleration_rad_s2),
    )
    assert valid1 is True

    # Publish next sample (wraps to 1)
    node._publish_trajectory_sample()
    assert len(published) == 2
    msg2 = published[1]
    assert msg2.sequence == 1

    # Validate second command at driver (wraps to 1)
    valid2, reason2 = driver_state.validate_joint_setpoint(
        protocol_version=msg2.protocol_version,
        session_id=msg2.session_id,
        sequence=msg2.sequence,
        valid_for_ms=msg2.valid_for_ms,
        enable_mask=msg2.enable_mask,
        position_rad=list(msg2.position_rad),
        velocity_rad_s=list(msg2.velocity_rad_s),
        acceleration_rad_s2=list(msg2.acceleration_rad_s2),
    )
    assert valid2 is True, f"Driver rejected wrapped sequence 1: {reason2}"


def test_unarmed_shadow_mode_trajectory_interpolation_progresses(node):
    """Unarmed node with fresh telemetry continues shadow PVA stream with enable_mask=0 and session_id=0."""
    from doghead_driver.safety_validator import check_two_key_actuator_gate

    assert node.driver_armed is False
    assert node.active_session_id == 0

    # Provide fresh state
    node._on_mcu_state(_make_fresh_mcu_state(sequence=1, uptime_ms=1000))
    assert node.measured_state_valid is True

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)

    # Queue trajectory while unarmed (shadow mode)
    node._on_ef_reference(_make_ef_reference())
    assert node._trajectory_active is True
    assert len(published) == 1
    assert published[0].enable_mask == 0x00
    assert published[0].session_id == 0

    # Execute multiple timer ticks while keeping telemetry fresh and advancing simulated elapsed time
    for step in range(2, 6):
        node._on_mcu_state(_make_fresh_mcu_state(sequence=step, uptime_ms=1000 + step * 10))
        node._trajectory_start_time_s -= 0.05
        node._publish_trajectory_sample()
        assert node._trajectory_active is True, f"Trajectory prematurely aborted at step {step}"

    assert len(published) == 5

    # Confirm position samples progress along the S-curve
    p0 = published[0].position_rad[0]
    p_last = published[-1].position_rad[0]
    assert p_last != p0, f"Expected position progress along trajectory, got start={p0}, last={p_last}"

    # Confirm every command is strictly shadow-safe
    for sample in published:
        assert sample.enable_mask == 0x00
        assert sample.session_id == 0
        actuator_output_allowed = check_two_key_actuator_gate(
            armed=node.driver_armed, shadow_mode=True, actuator_enabled=False
        )
        assert actuator_output_allowed is False


def test_mcu_state_protocol_version_mismatch_aborts_active_trajectory(node):
    """Protocol version mismatch in telemetry must stop active trajectory and disarm session."""
    _arm_session(node, session_id=77)
    node._on_mcu_state(_make_fresh_mcu_state(sequence=1, uptime_ms=1000, protocol_version=1))

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)
    node._on_ef_reference(_make_ef_reference())

    assert node._trajectory_active is True
    published.clear()

    # Receive McuState with protocol version mismatch (e.g. 99)
    node._on_mcu_state(_make_fresh_mcu_state(sequence=2, uptime_ms=2000, protocol_version=99))

    assert node._trajectory_active is False
    assert node.driver_armed is False
    assert node.active_session_id == 0
    assert node.measured_state_valid is False
    assert len(published) == 1
    assert published[0].enable_mask == 0x00
    assert published[0].session_id == 0


def test_mcu_state_zero_boot_id_aborts_active_trajectory(node):
    """boot_id=0 telemetry must stop active trajectory and clear session."""
    _arm_session(node, session_id=88)
    node._on_mcu_state(_make_fresh_mcu_state(boot_id=1, sequence=1, uptime_ms=1000))

    published = []
    node.setpoint_pub.publish = lambda msg: published.append(msg)
    node._on_ef_reference(_make_ef_reference())

    assert node._trajectory_active is True
    published.clear()

    # Receive McuState with boot_id=0
    node._on_mcu_state(_make_fresh_mcu_state(boot_id=0, sequence=2, uptime_ms=2000))

    assert node._trajectory_active is False
    assert node.driver_armed is False
    assert node.active_session_id == 0
    assert node.measured_state_valid is False
    assert len(published) == 1
    assert published[0].enable_mask == 0x00
    assert published[0].session_id == 0


def test_driver_session_state_protocol_version_mismatch_disarms(node):
    """DriverSessionState with protocol version mismatch must disarm motion node."""
    _arm_session(node, session_id=50)
    assert node.driver_armed is True

    # DriverSessionState message with protocol_version mismatch
    invalid_session_msg = DriverSessionState()
    invalid_session_msg.protocol_version = 99  # Expected: 1
    invalid_session_msg.session_id = 50
    invalid_session_msg.armed = True
    invalid_session_msg.disarm_reason = "OK"

    node._on_driver_session_state(invalid_session_msg)

    assert node.driver_armed is False
    assert node.active_session_id == 0
    assert node.measured_state_valid is False



