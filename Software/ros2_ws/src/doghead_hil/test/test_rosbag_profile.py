from pathlib import Path


def test_rosbag_profile_matches_declared_runtime_topics():
    profile = Path(__file__).parents[2] / "doghead_bringup" / "config" / "rosbag2_profile.yaml"
    topics = {
        line.strip()[2:]
        for line in profile.read_text(encoding="utf-8").splitlines()
        if line.strip().startswith("- /")
    }
    expected = {
        "/joint_states",
        "/doghead/mcu/state", "/doghead/mcu/diagnostics",
        "/doghead/mcu/session_control", "/doghead/mcu/joint_setpoint",
        "/doghead/mcu/motor_test_command",
        "/doghead/command/joint_setpoint", "/doghead/command/motor_test",
        "/doghead/motion/ef_reference", "/doghead/motion/status",
        "/doghead/driver/status", "/doghead/driver/session_state",
        "/doghead/hil/telemetry_report", "/doghead/hil/scenario_status",
        "/doghead/heartbeat",
    }
    assert topics == expected
