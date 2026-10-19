import pytest
from doghead_hil.tracking_metrics import TrackingMetricsAnalyzer, is_telemetry_silent


def test_telemetry_silence_gate_is_fail_closed():
    assert is_telemetry_silent(None, 1.0, 0.5)
    assert is_telemetry_silent(1.0, 1.6, 0.5)
    assert not is_telemetry_silent(1.0, 1.4, 0.5)
    assert is_telemetry_silent(10.0, 9.0, 0.5)
    assert is_telemetry_silent(1.0, 1.2, -0.5)
    assert is_telemetry_silent(1.0, 1.2, float("nan"))


def test_tracking_metrics_zero_error():
    analyzer = TrackingMetricsAnalyzer()

    # Record identical target and measured values
    for i in range(10):
        analyzer.record_sample(
            timestamp_s=i * 0.02,
            target_pos=[0.1, 0.2, 0.3],
            measured_pos=[0.1, 0.2, 0.3],
            target_vel=[0.0, 0.0, 0.0],
            measured_vel=[0.0, 0.0, 0.0],
            command_age_ms=10
        )

    joint_rmse, overall_rmse = analyzer.calculate_position_rmse()
    assert overall_rmse == 0.0
    assert joint_rmse == [0.0, 0.0, 0.0]

    max_err = analyzer.calculate_max_position_error()
    assert max_err == [0.0, 0.0, 0.0]

    mean_period, jitter_std, max_period = analyzer.calculate_telemetry_jitter()
    assert abs(mean_period - 20.0) < 1e-3  # 0.02s = 20ms
    assert jitter_std == 0.0


def test_tracking_metrics_known_error():
    analyzer = TrackingMetricsAnalyzer()

    analyzer.record_sample(0.0, [0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0, 0, 0], [0, 0, 0], 5)
    analyzer.record_sample(0.02, [0.0, 0.0, 0.0], [-0.1, 0.0, 0.0], [0, 0, 0], [0, 0, 0], 10)

    joint_rmse, overall_rmse = analyzer.calculate_position_rmse()
    assert abs(joint_rmse[0] - 0.1) < 1e-4
    assert joint_rmse[1] == 0.0
    assert joint_rmse[2] == 0.0

    rep = analyzer.generate_summary_dict()
    assert rep["sample_count"] == 2
    assert rep["command_age_mean_ms"] == 7.5


def test_tracking_metrics_sequence_and_wraparound():
    analyzer = TrackingMetricsAnalyzer()

    # 1. Consecutive sequences & uptime
    analyzer.record_sample(0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=100, uptime_ms=1000)
    analyzer.record_sample(0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=101, uptime_ms=1020)
    analyzer.record_sample(0.04, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=102, uptime_ms=1040)

    rep = analyzer.generate_summary_dict()
    assert rep["state_sequence_total_received"] == 3
    assert rep["state_sequence_dropped"] == 0
    assert rep["state_sequence_duplicates"] == 0
    assert rep["state_sequence_out_of_order"] == 0
    assert abs(rep["mcu_period_mean_ms"] - 20.0) < 1e-3
    assert abs(rep["mcu_rate_hz"] - 50.0) < 1e-1

    # 2. Sequence gap (dropped frames)
    # Jump from 102 to 106 (gap of 4 sequence steps -> 3 dropped frames: 103, 104, 105)
    analyzer.record_sample(0.12, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=106, uptime_ms=1120)

    rep = analyzer.generate_summary_dict()
    assert rep["state_sequence_total_received"] == 4
    assert rep["state_sequence_dropped"] == 3

    # 3. Duplicate sequence
    analyzer.record_sample(0.14, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=106, uptime_ms=1120)

    rep = analyzer.generate_summary_dict()
    assert rep["state_sequence_duplicates"] == 1

    # 4. Out-of-order sequence
    analyzer.record_sample(0.16, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=104, uptime_ms=1080)

    rep = analyzer.generate_summary_dict()
    assert rep["state_sequence_out_of_order"] == 1

    # 5. uint32 wraparound
    analyzer2 = TrackingMetricsAnalyzer()
    analyzer2.record_sample(0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=4294967295, uptime_ms=4294967280)
    analyzer2.record_sample(0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=0, uptime_ms=4294967300)
    analyzer2.record_sample(0.04, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=1, uptime_ms=24)

    rep2 = analyzer2.generate_summary_dict()
    assert rep2["state_sequence_total_received"] == 3
    assert rep2["state_sequence_dropped"] == 0
    assert rep2["state_sequence_duplicates"] == 0
    assert rep2["state_sequence_out_of_order"] == 0
    assert abs(rep2["mcu_period_mean_ms"] - 20.0) < 1e-3


def test_tracking_metrics_out_of_order_and_boot_reset():
    # 1. Same-boot out-of-order frame followed by normal frame
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(0.00, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=100, uptime_ms=1000, boot_id=1)
    analyzer.record_sample(0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=101, uptime_ms=1020, boot_id=1)
    # Out-of-order frame within same boot
    analyzer.record_sample(0.04, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=99, uptime_ms=980, boot_id=1)
    # Next normal frame (102 after 101)
    analyzer.record_sample(0.06, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=102, uptime_ms=1040, boot_id=1)

    rep = analyzer.generate_summary_dict()
    assert rep["state_sequence_total_received"] == 4
    assert rep["state_sequence_dropped"] == 0
    assert rep["state_sequence_out_of_order"] == 1

    # 2. MCU reboot / boot_id reset
    analyzer2 = TrackingMetricsAnalyzer()
    analyzer2.record_sample(0.00, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=100, uptime_ms=1000, boot_id=1)
    # MCU rebooted: boot_id changes to 2, sequence resets to 1
    analyzer2.record_sample(0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=1, uptime_ms=20, boot_id=2)
    # Next normal frame
    analyzer2.record_sample(0.04, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, state_sequence=2, uptime_ms=40, boot_id=2)

    rep2 = analyzer2.generate_summary_dict()
    assert rep2["state_sequence_total_received"] == 3
    assert rep2["state_sequence_dropped"] == 0
    assert rep2["state_sequence_out_of_order"] == 1


def test_telemetry_rate_violation_fails_expected_rate_gate():
    analyzer = TrackingMetricsAnalyzer()
    for i in range(3):
        analyzer.record_sample(
            i * 0.1, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
    gate = analyzer.evaluate_timing(expected_rate_hz=50.0, max_jitter_ms=10.0)
    assert gate["rate_pass"] is False
    assert gate["pass"] is False


def test_telemetry_jitter_violation_uses_max_period_deviation():
    analyzer = TrackingMetricsAnalyzer()
    timestamps = [0.0, 0.02, 0.04, 0.10]
    for timestamp in timestamps:
        analyzer.record_sample(
            timestamp, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
    timing = analyzer.calculate_telemetry_timing()
    assert timing["jitter_std_ms"] != timing["max_period_ms"]
    assert timing["max_period_deviation_ms"] > 10.0
    gate = analyzer.evaluate_timing(expected_rate_hz=30.0, max_jitter_ms=10.0)
    assert gate["jitter_pass"] is False
    assert gate["pass"] is False


def test_dropped_frame_violation_fails_gate_and_csv_export(tmp_path):
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(
        0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
        state_sequence=10, uptime_ms=100, boot_id=1)
    analyzer.record_sample(
        0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
        state_sequence=12, uptime_ms=120, boot_id=1)
    gate = analyzer.evaluate_timing(expected_rate_hz=50.0, max_jitter_ms=10.0)
    assert gate["dropped_frame_violation"] is True
    assert gate["dropped_frames"] == 1
    assert gate["pass"] is False

    csv_path = tmp_path / "tracking.csv"
    analyzer.export_csv(str(csv_path))
    csv_text = csv_path.read_text(encoding="utf-8")
    assert csv_text.splitlines()[0].startswith("timestamp_s,")
    assert "target_pos_0" in csv_text.splitlines()[0]


def test_applied_state_and_output_provenance_are_exported(tmp_path):
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(
        0.0, [1, 2, 3], [0, 0, 0], [0, 0, 0], [0, 0, 0],
        applied_pos=[0.1, 0.2, 0.3], applied_vel=[0.4, 0.5, 0.6],
        position_error=[0.1, 0.2, 0.3], commanded_rpm=[1, 2, 3],
        pwm_frequency_hz=[10, 20, 30], enable_mask=7,
        physical_enable_mask=5,
        safety_state=3, fault_code=0, physical_gate_state=1)
    path = tmp_path / "provenance.csv"
    analyzer.export_csv(str(path))
    header = path.read_text(encoding="utf-8").splitlines()[0]
    row = path.read_text(encoding="utf-8").splitlines()[1]
    assert "applied_pos_0" in header
    assert "physical_enable_mask" in header
    assert "pwm_frequency_hz_2" in header
    assert row.endswith(",7,5,3,0,1")
    rep = analyzer.generate_summary_dict()
    assert rep["logical_enable_mask_last"] == 7
    assert rep["physical_enable_mask_last"] == 5
    assert rep["physical_gate_state_last"] == 1


def test_telemetry_validator_monotonic_injection():
    rclpy = pytest.importorskip("rclpy")
    from doghead_hil.telemetry_validator import TelemetryValidatorNode
    from doghead_interfaces.msg import McuState

    mock_times = [100.0, 100.02, 100.04]
    time_idx = 0

    def mock_time():
        nonlocal time_idx
        t = mock_times[time_idx]
        time_idx = min(time_idx + 1, len(mock_times) - 1)
        return t

    node = None
    try:
        rclpy.init()
        node = TelemetryValidatorNode(time_fn=mock_time)
        msg1 = McuState()
        msg1.boot_id = 1
        msg1.state_sequence = 1
        msg1.uptime_ms = 1000

        msg2 = McuState()
        msg2.boot_id = 1
        msg2.state_sequence = 2
        msg2.uptime_ms = 1020

        node._on_mcu_state(msg1)
        node._on_mcu_state(msg2)

        rep = node.analyzer.generate_summary_dict()
        assert rep["sample_count"] == 2
        assert abs(rep["telemetry_period_mean_ms"] - 20.0) < 1e-3
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_telemetry_validator_physical_enable_mask_provenance():
    rclpy = pytest.importorskip("rclpy")
    from doghead_hil.telemetry_validator import TelemetryValidatorNode
    from doghead_interfaces.msg import McuState

    node = None
    try:
        rclpy.init()
        node = TelemetryValidatorNode(time_fn=lambda: 100.0)
        assert node.actuator_hardware_output_enabled is False

        msg = McuState()
        msg.boot_id = 1
        msg.state_sequence = 1
        msg.uptime_ms = 1000
        msg.enable_mask = 7
        assert hasattr(msg, "physical_enable_mask"), "McuState binding missing physical_enable_mask field"
        msg.physical_enable_mask = 5

        node._on_mcu_state(msg)

        rep = node.analyzer.generate_summary_dict()
        assert rep["logical_enable_mask_last"] == 7
        assert rep["physical_enable_mask_last"] == 5
        assert rep["physical_gate_state_last"] == 1
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_timing_gate_fails_on_duplicate_frames():
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], state_sequence=1, uptime_ms=100, boot_id=1)
    analyzer.record_sample(0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], state_sequence=1, uptime_ms=120, boot_id=1)
    gate = analyzer.evaluate_timing(expected_rate_hz=50.0, max_jitter_ms=10.0)
    assert gate["duplicate_frame_violation"] is True
    assert gate["duplicate_frames"] == 1
    assert gate["pass"] is False


def test_timing_gate_fails_on_out_of_order_frames():
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], state_sequence=10, uptime_ms=100, boot_id=1)
    analyzer.record_sample(0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], state_sequence=9, uptime_ms=120, boot_id=1)
    gate = analyzer.evaluate_timing(expected_rate_hz=50.0, max_jitter_ms=10.0)
    assert gate["out_of_order_violation"] is True
    assert gate["out_of_order_frames"] == 1
    assert gate["pass"] is False


def test_timing_gate_fails_on_clock_rollback():
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(1.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
    analyzer.record_sample(0.9, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
    gate = analyzer.evaluate_timing(expected_rate_hz=50.0, max_jitter_ms=10.0)
    assert gate["clock_rollback_violation"] is True
    assert gate["pass"] is False


def test_timing_gate_fails_on_invalid_thresholds():
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
    analyzer.record_sample(0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
    gate = analyzer.evaluate_timing(expected_rate_hz=-50.0, max_jitter_ms=10.0)
    assert gate["invalid_thresholds_violation"] is True
    assert gate["pass"] is False

    gate_nan = analyzer.evaluate_timing(expected_rate_hz=50.0, max_jitter_ms=float("nan"))
    assert gate_nan["invalid_thresholds_violation"] is True
    assert gate_nan["pass"] is False


def test_timing_gate_fails_on_insufficient_samples():
    analyzer = TrackingMetricsAnalyzer()
    gate0 = analyzer.evaluate_timing(expected_rate_hz=50.0, max_jitter_ms=10.0)
    assert gate0["insufficient_samples_violation"] is True
    assert gate0["telemetry_silence_violation"] is True
    assert gate0["pass"] is False

    analyzer.record_sample(0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
    gate1 = analyzer.evaluate_timing(expected_rate_hz=50.0, max_jitter_ms=10.0)
    assert gate1["insufficient_samples_violation"] is True
    assert gate1["pass"] is False


def test_timing_gate_fails_on_silence():
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
    analyzer.record_sample(0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])
    gate = analyzer.evaluate_timing(
        expected_rate_hz=50.0, max_jitter_ms=10.0,
        telemetry_timeout_s=0.5, last_rx_time=0.02, now_s=1.0
    )
    assert gate["telemetry_silence_violation"] is True
    assert gate["pass"] is False


def test_tracking_report_exit_codes(tmp_path):
    from doghead_hil.tracking_report import main as report_main

    valid_csv = tmp_path / "valid.csv"
    a_valid = TrackingMetricsAnalyzer()
    for i in range(10):
        a_valid.record_sample(
            i * 0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
            state_sequence=i + 1, uptime_ms=i * 20, boot_id=1
        )
    a_valid.export_csv(str(valid_csv))
    assert report_main([str(valid_csv)]) == 0

    dup_csv = tmp_path / "dup.csv"
    a_dup = TrackingMetricsAnalyzer()
    a_dup.record_sample(0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], state_sequence=1, uptime_ms=0, boot_id=1)
    a_dup.record_sample(0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], state_sequence=1, uptime_ms=20, boot_id=1)
    a_dup.export_csv(str(dup_csv))
    assert report_main([str(dup_csv)]) == 1

    ooo_csv = tmp_path / "ooo.csv"
    a_ooo = TrackingMetricsAnalyzer()
    a_ooo.record_sample(0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], state_sequence=5, uptime_ms=0, boot_id=1)
    a_ooo.record_sample(0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], state_sequence=4, uptime_ms=20, boot_id=1)
    a_ooo.export_csv(str(ooo_csv))
    assert report_main([str(ooo_csv)]) == 1

    assert report_main([str(tmp_path / "nonexistent.csv")]) == 1


def test_telemetry_validator_rejects_missing_physical_provenance_field():
    """Verify TelemetryValidatorNode rejects a McuState msg missing physical_enable_mask without fallback."""
    rclpy = pytest.importorskip("rclpy")
    from doghead_hil.telemetry_validator import TelemetryValidatorNode

    class LegacyMcuStateMissingPhysicalMask:
        def __init__(self):
            self.boot_id = 1
            self.state_sequence = 1
            self.uptime_ms = 100
            self.control_mode = 0
            self.safety_state = 0
            self.fault_code = 0
            self.enable_mask = 7
            self.agent_connected = True
            self.target_position_rad = [0.0, 0.0, 0.0]
            self.target_velocity_rad_s = [0.0, 0.0, 0.0]
            self.target_acceleration_rad_s2 = [0.0, 0.0, 0.0]
            self.applied_position_rad = [0.0, 0.0, 0.0]
            self.applied_velocity_rad_s = [0.0, 0.0, 0.0]
            self.measured_position_rad = [0.0, 0.0, 0.0]
            self.measured_velocity_rad_s = [0.0, 0.0, 0.0]
            self.position_error_rad = [0.0, 0.0, 0.0]
            self.commanded_motor_rpm = [0.0, 0.0, 0.0]
            self.encoder_count = [0, 0, 0]
            self.pwm_frequency_hz = [0, 0, 0]
            self.command_age_ms = 10
            self.last_accepted_command_sequence = 1
            self.rejected_command_total = 0

    node = None
    try:
        rclpy.init()
        node = TelemetryValidatorNode(time_fn=lambda: 100.0)
        invalid_msg = LegacyMcuStateMissingPhysicalMask()
        with pytest.raises(AttributeError, match="missing required physical_enable_mask"):
            node._on_mcu_state(invalid_msg)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_telemetry_validator_fails_output_off_acceptance_when_physical_enable_nonzero():
    """Verify timing gate fails output-off acceptance when physical_enable_mask is non-zero in hardware output OFF mode."""
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(
        0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
        state_sequence=1, uptime_ms=100, boot_id=1,
        enable_mask=7, physical_enable_mask=1, requested_enable_mask=7
    )
    analyzer.record_sample(
        0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
        state_sequence=2, uptime_ms=120, boot_id=1,
        enable_mask=7, physical_enable_mask=1, requested_enable_mask=7
    )
    gate = analyzer.evaluate_timing(
        expected_rate_hz=50.0, max_jitter_ms=10.0,
        actuator_hardware_output_enabled=False
    )
    assert gate["output_off_violation"] is True
    assert gate["pass"] is False


def test_full_enable_provenance_contract_recorded_and_exported(tmp_path):
    """Verify requested_enable_mask, applied enable_mask, and physical_enable_mask are fully preserved."""
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(
        0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
        requested_enable_mask=7, enable_mask=3, physical_enable_mask=0
    )
    rep = analyzer.generate_summary_dict()
    assert rep["requested_enable_mask_last"] == 7
    assert rep["logical_enable_mask_last"] == 3
    assert rep["physical_enable_mask_last"] == 0

    csv_file = tmp_path / "provenance_test.csv"
    analyzer.export_csv(str(csv_file))
    lines = csv_file.read_text(encoding="utf-8").splitlines()
    assert "requested_enable_mask" in lines[0]
    assert lines[1].endswith(",7,3,0,0,0,0")


def test_telemetry_validator_rejects_missing_requested_provenance_field():
    """Verify TelemetryValidatorNode rejects a McuState msg missing requested_enable_mask without fallback."""
    rclpy = pytest.importorskip("rclpy")
    from doghead_hil.telemetry_validator import TelemetryValidatorNode

    class LegacyMcuStateMissingRequestedMask:
        def __init__(self):
            self.boot_id = 1
            self.state_sequence = 1
            self.uptime_ms = 100
            self.control_mode = 0
            self.safety_state = 0
            self.fault_code = 0
            self.enable_mask = 7
            self.physical_enable_mask = 0
            self.agent_connected = True
            self.target_position_rad = [0.0, 0.0, 0.0]
            self.target_velocity_rad_s = [0.0, 0.0, 0.0]
            self.target_acceleration_rad_s2 = [0.0, 0.0, 0.0]
            self.applied_position_rad = [0.0, 0.0, 0.0]
            self.applied_velocity_rad_s = [0.0, 0.0, 0.0]
            self.measured_position_rad = [0.0, 0.0, 0.0]
            self.measured_velocity_rad_s = [0.0, 0.0, 0.0]
            self.position_error_rad = [0.0, 0.0, 0.0]
            self.commanded_motor_rpm = [0.0, 0.0, 0.0]
            self.encoder_count = [0, 0, 0]
            self.pwm_frequency_hz = [0, 0, 0]
            self.command_age_ms = 10
            self.last_accepted_command_sequence = 1
            self.rejected_command_total = 0

    node = None
    try:
        rclpy.init()
        node = TelemetryValidatorNode(time_fn=lambda: 100.0)
        invalid_msg = LegacyMcuStateMissingRequestedMask()
        with pytest.raises(AttributeError, match="missing required requested_enable_mask"):
            node._on_mcu_state(invalid_msg)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_non_finite_telemetry_fails_gate():
    """Verify non-finite telemetry (NaN/Inf) triggers non_finite_telemetry_violation and fails gate."""
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(
        0.0, [float("nan"), 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
        state_sequence=1, uptime_ms=100, boot_id=1
    )
    analyzer.record_sample(
        0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
        state_sequence=2, uptime_ms=120, boot_id=1
    )
    gate = analyzer.evaluate_timing(expected_rate_hz=50.0, max_jitter_ms=10.0)
    assert gate["non_finite_telemetry_violation"] is True
    assert gate["non_finite_count"] == 1
    assert gate["pass"] is False


def test_output_off_acceptance_with_non_zero_physical_gate_state():
    """Verify timing gate fails output-off acceptance when physical_gate_state is non-zero."""
    analyzer = TrackingMetricsAnalyzer()
    analyzer.record_sample(
        0.0, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
        state_sequence=1, uptime_ms=100, boot_id=1,
        enable_mask=7, physical_enable_mask=0, physical_gate_state=1
    )
    analyzer.record_sample(
        0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
        state_sequence=2, uptime_ms=120, boot_id=1,
        enable_mask=7, physical_enable_mask=0, physical_gate_state=1
    )
    gate = analyzer.evaluate_timing(
        expected_rate_hz=50.0, max_jitter_ms=10.0,
        actuator_hardware_output_enabled=False
    )
    assert gate["output_off_violation"] is True
    assert gate["pass"] is False


def test_passing_all_zero_output_off_sample():
    """Verify passing timing gate with all-zero output-off physical mask/gate sample."""
    analyzer = TrackingMetricsAnalyzer()
    for i in range(10):
        analyzer.record_sample(
            i * 0.02, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
            state_sequence=i + 1, uptime_ms=i * 20, boot_id=1,
            requested_enable_mask=7, enable_mask=7, physical_enable_mask=0, physical_gate_state=0
        )
    gate = analyzer.evaluate_timing(
        expected_rate_hz=50.0, max_jitter_ms=10.0,
        actuator_hardware_output_enabled=False
    )
    assert gate["output_off_violation"] is False
    assert gate["pass"] is True


def test_tracking_report_rejects_missing_provenance_csv(tmp_path):
    """Verify tracking_report exits non-zero when CSV is missing required physical/requested enable mask headers."""
    from doghead_hil.tracking_report import main as report_main

    bad_csv = tmp_path / "bad_headers.csv"
    bad_csv.write_text("timestamp_s,target_pos_0,target_pos_1,target_pos_2,measured_pos_0,measured_pos_1,measured_pos_2\n0.0,0,0,0,0,0,0\n", encoding="utf-8")
    assert report_main([str(bad_csv)]) == 1


