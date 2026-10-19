import pytest
from doghead_hil.scenario_runner import HilScenarioRunnerNode


def test_scenario_runner_default_safety():
    # Verify default parameter for actuator_enabled is False
    # Without starting ROS node spinning, we can test scenario helper method logic
    node = None
    try:
        import rclpy
        rclpy.init()
        node = HilScenarioRunnerNode()
        assert node.actuator_enabled is False

        sp = node._create_base_setpoint(sequence=1)
        assert sp.enable_mask == 0x00  # Physically gated OFF by default
        assert sp.session_id == 0
        expected = [
            (1.3962634 + 3.1415927) / 2.0,
            3.1415927 / 2.0,
            0.0,
        ]
        assert all(abs(float(actual) - wanted) < 1e-6 for actual, wanted in zip(sp.position_rad, expected))
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_scenario_runner_uses_live_driver_session():
    import rclpy
    from doghead_interfaces.msg import DriverSessionState

    node = None
    try:
        rclpy.init()
        node = HilScenarioRunnerNode()
        armed = DriverSessionState()
        armed.armed = True
        armed.session_id = 77
        node._on_session_state(armed)
        assert node._create_base_setpoint(sequence=1).session_id == 77

        disarmed = DriverSessionState()
        disarmed.armed = False
        disarmed.session_id = 77
        node._on_session_state(disarmed)
        assert node._create_base_setpoint(sequence=2).session_id == 0
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_telemetry_validator_qos_compatibility():
    """Assert TelemetryValidatorNode configures Best-Effort QoS for McuState subscription."""
    import rclpy
    from rclpy.qos import QoSReliabilityPolicy
    from doghead_hil.telemetry_validator import TelemetryValidatorNode

    node = None
    try:
        rclpy.init()
        node = TelemetryValidatorNode()
        state_sub = [sub for sub in node.subscriptions if sub.topic_name == '/doghead/mcu/state'][0]
        assert state_sub.qos_profile.reliability == QoSReliabilityPolicy.BEST_EFFORT

        diag_sub = [sub for sub in node.subscriptions if sub.topic_name == '/doghead/mcu/diagnostics'][0]
        assert diag_sub.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
