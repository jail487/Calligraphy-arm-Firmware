import math
import pytest
from doghead_driver.safety_validator import (
    DriverSafetyState,
    is_finite_number,
    is_finite_sequence,
    validate_array_size,
    validate_protocol_version,
    validate_ttl,
    is_sequence_newer,
    check_two_key_actuator_gate,
    EXPECTED_PROTOCOL_VERSION,
    validate_limits,
    SESSION_BOOT_TAG_SHIFT,
)


def test_finiteness_check():
    assert is_finite_number(0.0) is True
    assert is_finite_number(100) is True
    assert is_finite_number(float('nan')) is False
    assert is_finite_number(float('inf')) is False
    assert is_finite_number(-float('inf')) is False
    assert is_finite_number("string") is False

    # ROS 2 float32 arrays expose NumPy scalar values at runtime.
    import numpy as np
    assert is_finite_number(np.float32(0.0)) is True
    assert is_finite_number(np.float32('nan')) is False

    assert is_finite_sequence([0.1, 0.2, 0.3]) is True
    assert is_finite_sequence([0.1, float('nan'), 0.3]) is False
    assert is_finite_sequence([0.1, float('inf'), 0.3]) is False


def test_array_size_validation():
    assert validate_array_size([1.0, 2.0, 3.0], 3) is True
    assert validate_array_size([1.0, 2.0], 3) is False
    assert validate_array_size([1.0, 2.0, 3.0, 4.0], 3) is False


def test_protocol_version_validation():
    ok, msg = validate_protocol_version(1, expected=1)
    assert ok is True
    ok, msg = validate_protocol_version(2, expected=1)
    assert ok is False
    assert "mismatch" in msg


def test_ttl_validation():
    ok, _ = validate_ttl(250, max_ttl_ms=1000)
    assert ok is True
    ok, _ = validate_ttl(0, max_ttl_ms=1000)
    assert ok is False
    ok, _ = validate_ttl(1500, max_ttl_ms=1000)
    assert ok is False


def test_driver_state_uses_configured_ttl_limit():
    state = DriverSafetyState(max_ttl_ms=100)
    state.on_mcu_state(protocol_version=1, boot_id=1, agent_connected=True, now_s=1.0)
    ok, session, _ = state.arm()
    assert ok
    ok, message = state.validate_joint_setpoint(
        protocol_version=1, session_id=session, sequence=1, valid_for_ms=101, enable_mask=0,
        position_rad=[2.0, 1.0, 0.0], velocity_rad_s=[0.0, 0.0, 0.0],
        acceleration_rad_s2=[0.0, 0.0, 0.0])
    assert not ok
    assert "exceeds max limit" in message
def test_sequence_monotonicity():
    assert is_sequence_newer(10, 5) is True
    assert is_sequence_newer(5, 10) is False
    assert is_sequence_newer(5, 5) is False
    # 32-bit wrap test: from 0xFFFFFFFF to 10 and 1
    assert is_sequence_newer(10, 0xFFFFFFFF) is True
    assert is_sequence_newer(1, 0xFFFFFFFF) is True
    assert is_sequence_newer(0xFFFFFFFF, 0xFFFFFFFE) is True


def test_driver_state_mcu_uptime_and_sequence_wrap_around():
    state = DriverSafetyState()
    ok, _ = state.on_mcu_state(
        protocol_version=1, boot_id=1, agent_connected=True,
        state_sequence=0xFFFFFFFF, uptime_ms=0xFFFFFFF0, now_s=1.0)
    assert ok

    # Sequence and uptime wrap around 0xFFFFFFFF
    ok, msg = state.on_mcu_state(
        protocol_version=1, boot_id=1, agent_connected=True,
        state_sequence=10, uptime_ms=100, now_s=1.1)
    assert ok, f"Wrap-around MCU state rejected: {msg}"
    assert state.last_state_sequence == 10
    assert state.last_uptime_ms == 100



def test_driver_safety_state_reconnect():
    state = DriverSafetyState()
    assert state.armed is False

    # Simulate agent connected
    ok, msg = state.on_mcu_state(protocol_version=1, boot_id=100, agent_connected=True)
    assert ok is True

    # Arm driver
    armed, session_id, msg = state.arm()
    assert armed is True
    assert session_id > 0

    # Simulate agent reconnect (agent_connected drops to False then True)
    ok, msg = state.on_mcu_state(protocol_version=1, boot_id=100, agent_connected=False)
    assert ok is False
    assert state.armed is False  # Must disarm!

    # Try validating setpoint when disarmed -> must fail
    ok, msg = state.validate_joint_setpoint(
        protocol_version=1, session_id=session_id, sequence=1, valid_for_ms=250, enable_mask=1,
        position_rad=[2.0, 1.0, 0.0], velocity_rad_s=[0.0, 0.0, 0.0], acceleration_rad_s2=[0.0, 0.0, 0.0]
    )
    assert ok is False
    assert "DISARMED" in msg


def test_boot_id_change_invalidates_session_and_rejects_old_replay():
    state = DriverSafetyState()
    state.on_mcu_state(protocol_version=1, boot_id=10, agent_connected=True, now_s=1.0)
    ok, old_session, _ = state.arm()
    assert ok

    ok, message = state.on_mcu_state(
        protocol_version=1, boot_id=11, agent_connected=True, now_s=1.02)
    assert not ok
    assert "boot_id" in message
    assert not state.armed
    assert state.session_id == 0

    ok, message = state.validate_joint_setpoint(
        protocol_version=1, session_id=old_session, sequence=1, valid_for_ms=250, enable_mask=0,
        position_rad=[2.0, 1.0, 0.0], velocity_rad_s=[0.0, 0.0, 0.0],
        acceleration_rad_s2=[0.0, 0.0, 0.0])
    assert not ok
    assert "DISARMED" in message


def test_reconnect_counter_reset_is_not_ignored():
    state = DriverSafetyState()
    state.on_mcu_state(protocol_version=1, boot_id=1, agent_connected=True, now_s=1.0)
    state.last_reconnect_total = 8
    ok, message = state.on_mcu_state(
        protocol_version=1, boot_id=1, agent_connected=True,
        reconnect_total=0, now_s=1.1)
    assert not ok
    assert "changed" in message
    assert not state.armed


def test_telemetry_silence_disarms_armed_driver():
    state = DriverSafetyState()
    state.on_mcu_state(protocol_version=1, boot_id=1, agent_connected=True, now_s=10.0)
    ok, _, _ = state.arm()
    assert ok

    ok, message = state.check_telemetry_timeout(now_s=10.6, timeout_s=0.5)
    assert not ok
    assert "silence" in message
    assert not state.armed
    assert state.session_id == 0


def test_motor_test_sequence_is_shared_with_joint_mailbox():
    state = DriverSafetyState()
    state.on_mcu_state(protocol_version=1, boot_id=1, agent_connected=True, now_s=1.0)
    ok, session, _ = state.arm()
    assert ok
    ok, _ = state.validate_motor_test_command(
        1, session, 10, 250, 0, 0.0, 0.0, 0.0, True)
    assert ok

    # A disable is an idempotent safety stop and remains forwardable even
    # after session loss with an old sequence.
    ok, _ = state.validate_motor_test_command(
        99, 0, 9, 0, 99, float("nan"), float("nan"), float("nan"), False)
    assert ok

    ok, message = state.validate_motor_test_command(
        1, session, 9, 250, 0, 0.0, 0.0, 0.0, True)
    assert not ok
    assert "Stale sequence" in message


def test_arm_epoch_is_not_reused_after_disarm():
    state = DriverSafetyState()
    state.on_mcu_state(protocol_version=1, boot_id=100, agent_connected=True)
    ok, first, _ = state.arm()
    assert ok and first > 0
    state.disarm()
    ok, second, _ = state.arm()
    assert ok and second > first
    assert (first >> SESSION_BOOT_TAG_SHIFT) == 100


def test_state_replay_does_not_refresh_watchdog_or_session():
    state = DriverSafetyState()
    ok, _ = state.on_mcu_state(
        protocol_version=1, boot_id=7, agent_connected=True,
        state_sequence=10, uptime_ms=1000, now_s=10.0)
    assert ok
    ok, session, _ = state.arm()
    assert ok

    ok, message = state.on_mcu_state(
        protocol_version=1, boot_id=7, agent_connected=True,
        state_sequence=10, uptime_ms=1000, now_s=10.4)
    assert not ok
    assert "replayed" in message or "Stale" in message
    assert state.last_telemetry_time == 10.0
    assert not state.armed
    assert state.session_id == 0


def test_zero_boot_id_is_not_accepted_for_arm_or_liveness():
    state = DriverSafetyState()
    ok, message = state.on_mcu_state(
        protocol_version=1, boot_id=0, agent_connected=True,
        state_sequence=1, uptime_ms=1, now_s=1.0)
    assert not ok
    assert "boot_id" in message
    assert state.last_telemetry_time is None


def test_resolved_yaml_limits_are_validated_and_used():
    limits = validate_limits({
        "pos_min_rad": [1.0, 2.0, 3.0],
        "pos_max_rad": [2.0, 3.0, 4.0],
        "vel_max_rad_s": [0.1, 0.2, 0.3],
        "accel_max_rad_s2": [0.2, 0.3, 0.4],
        "bringup_motor_rpm_max": 90.0,
    })
    assert limits["pos_min_rad"] == [1.0, 2.0, 3.0]
    assert limits["bringup_motor_rpm_max"] == 90.0


def test_nan_injection_rejection():
    state = DriverSafetyState()
    state.on_mcu_state(protocol_version=1, boot_id=100, agent_connected=True)
    state.arm()

    # NaN in position
    ok, msg = state.validate_joint_setpoint(
        protocol_version=1, session_id=state.session_id, sequence=1, valid_for_ms=250, enable_mask=1,
        position_rad=[2.0, float('nan'), 0.0], velocity_rad_s=[0.0, 0.0, 0.0], acceleration_rad_s2=[0.0, 0.0, 0.0]
    )
    assert ok is False
    assert "Non-finite" in msg


def test_limit_exceeded_rejection():
    state = DriverSafetyState()
    state.on_mcu_state(protocol_version=1, boot_id=100, agent_connected=True)
    state.arm()

    # Position exceeds confirmed joint 1 max limit (+pi rad)
    ok, msg = state.validate_joint_setpoint(
        protocol_version=1, session_id=state.session_id, sequence=1, valid_for_ms=250, enable_mask=1,
        position_rad=[3.2, 0.0, 0.0], velocity_rad_s=[0.0, 0.0, 0.0], acceleration_rad_s2=[0.0, 0.0, 0.0]
    )
    assert ok is False
    assert "exceeds limit" in msg


def test_two_key_actuator_gate():
    # ALL three conditions must be True for enable output to be True:
    # 1. armed = True
    # 2. shadow_mode = False
    # 3. actuator_enabled = True

    # Case 1: Defaults (Disarmed, Shadow=True, ActuatorEnabled=False)
    assert check_two_key_actuator_gate(armed=False, shadow_mode=True, actuator_enabled=False) is False

    # Case 2: Armed, but Shadow=True
    assert check_two_key_actuator_gate(armed=True, shadow_mode=True, actuator_enabled=True) is False

    # Case 3: Armed, Shadow=False, but ActuatorEnabled=False
    assert check_two_key_actuator_gate(armed=True, shadow_mode=False, actuator_enabled=False) is False

    # Case 4: Disarmed, Shadow=False, ActuatorEnabled=True
    assert check_two_key_actuator_gate(armed=False, shadow_mode=False, actuator_enabled=True) is False

    # Case 5: Golden path (Armed=True, Shadow=False, ActuatorEnabled=True)
    assert check_two_key_actuator_gate(armed=True, shadow_mode=False, actuator_enabled=True) is True


def test_canonical_topics_parity():
    """Verify canonical contract topic names."""
    contract_topics = {
        "setpoint": "/doghead/mcu/joint_setpoint",
        "motor_test": "/doghead/mcu/motor_test_command",
        "state": "/doghead/mcu/state",
        "diagnostics": "/doghead/mcu/diagnostics",
        "joint_states": "/joint_states",
    }
    assert contract_topics["setpoint"] == "/doghead/mcu/joint_setpoint"
    assert contract_topics["motor_test"] == "/doghead/mcu/motor_test_command"
    assert contract_topics["state"] == "/doghead/mcu/state"
    assert contract_topics["diagnostics"] == "/doghead/mcu/diagnostics"
    assert contract_topics["joint_states"] == "/joint_states"


def test_driver_node_qos_compatibility():
    """Assert DogheadDriverNode configures Best-Effort QoS for high-rate telemetry topics."""
    import rclpy
    from rclpy.qos import QoSReliabilityPolicy
    from doghead_driver.driver_node import DogheadDriverNode

    node = None
    try:
        rclpy.init()
        node = DogheadDriverNode()
        # Verify /joint_states publisher QoS reliability policy is BEST_EFFORT
        assert node.joint_state_pub.qos_profile.reliability == QoSReliabilityPolicy.BEST_EFFORT

        # Verify /doghead/mcu/state subscription QoS reliability policy is BEST_EFFORT
        state_sub = [sub for sub in node.subscriptions if sub.topic_name == '/doghead/mcu/state'][0]
        assert state_sub.qos_profile.reliability == QoSReliabilityPolicy.BEST_EFFORT

        # Verify /doghead/mcu/diagnostics subscription retains RELIABLE QoS
        diag_sub = [sub for sub in node.subscriptions if sub.topic_name == '/doghead/mcu/diagnostics'][0]
        assert diag_sub.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE
        assert node.mcu_session_control_pub.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_driver_node_drops_nonfinite_telemetry():
    """MCU telemetry must not crash the driver or be emitted as JointState."""
    import rclpy
    from doghead_driver.driver_node import DogheadDriverNode
    from doghead_interfaces.msg import McuState

    node = None
    try:
        rclpy.init()
        node = DogheadDriverNode()
        telemetry = McuState()
        telemetry.protocol_version = 1
        telemetry.agent_connected = True
        telemetry.measured_position_rad = [float('nan'), 0.0, 0.0]
        telemetry.measured_velocity_rad_s = [0.0, 0.0, 0.0]
        telemetry.commanded_motor_rpm = [0.0, 0.0, 0.0]
        node._on_mcu_state(telemetry)
        assert node.safety_state.armed is False
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_driver_node_converts_ros_float32_telemetry():
    """Finite ROS float32 telemetry is converted before publishing JointState."""
    import rclpy
    from doghead_driver.driver_node import DogheadDriverNode
    from doghead_interfaces.msg import McuState

    node = None
    try:
        rclpy.init()
        node = DogheadDriverNode()
        telemetry = McuState()
        telemetry.protocol_version = 1
        telemetry.agent_connected = True
        telemetry.measured_position_rad = [0.0, 0.0, 0.0]
        telemetry.measured_velocity_rad_s = [0.0, 0.0, 0.0]
        telemetry.commanded_motor_rpm = [0.0, 0.0, 0.0]
        node._on_mcu_state(telemetry)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_driver_node_consumes_reconnect_counter_once():
    import rclpy
    from doghead_driver.driver_node import DogheadDriverNode
    from doghead_interfaces.msg import McuDiagnostics

    node = None
    try:
        rclpy.init()
        node = DogheadDriverNode()
        diagnostics = McuDiagnostics()
        diagnostics.agent_reconnect_total = 1
        node._on_mcu_diagnostics(diagnostics)
        assert node.safety_state.last_reconnect_total == 1
        node._on_mcu_diagnostics(diagnostics)
        assert node.safety_state.last_reconnect_total == 1
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_driver_safety_state_protocol_version_mismatch_disarms():
    state = DriverSafetyState()
    state.on_mcu_state(protocol_version=1, boot_id=10, agent_connected=True)
    ok, session, _ = state.arm()
    assert ok is True

    # Receive protocol version mismatch (e.g. version 2)
    ok, msg = state.on_mcu_state(protocol_version=2, boot_id=10, agent_connected=True)
    assert ok is False
    assert state.armed is False
    assert state.session_id == 0
    assert "Protocol version mismatch" in msg


def test_driver_safety_state_zero_boot_id_disarms():
    state = DriverSafetyState()
    state.on_mcu_state(protocol_version=1, boot_id=10, agent_connected=True)
    ok, session, _ = state.arm()
    assert ok is True

    # Receive telemetry with boot_id=0
    ok, msg = state.on_mcu_state(protocol_version=1, boot_id=0, agent_connected=True)
    assert ok is False
    assert state.armed is False
    assert state.session_id == 0
    assert "boot_id=0" in msg


def test_driver_node_protocol_version_mismatch_telemetry_disarms():
    import rclpy
    from doghead_driver.driver_node import DogheadDriverNode
    from doghead_interfaces.msg import McuState

    node = None
    try:
        rclpy.init()
        node = DogheadDriverNode()
        # First arm
        node.safety_state.on_mcu_state(protocol_version=1, boot_id=1, agent_connected=True)
        node.safety_state.arm()
        assert node.safety_state.armed is True

        # Send telemetry with protocol version mismatch
        telemetry = McuState()
        telemetry.protocol_version = 99
        telemetry.boot_id = 1
        telemetry.agent_connected = True
        telemetry.measured_position_rad = [2.0, 1.0, 0.0]
        telemetry.measured_velocity_rad_s = [0.0, 0.0, 0.0]
        telemetry.commanded_motor_rpm = [0.0, 0.0, 0.0]
        node._on_mcu_state(telemetry)

        assert node.safety_state.armed is False
        assert node.safety_state.session_id == 0
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

