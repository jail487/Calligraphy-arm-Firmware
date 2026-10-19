import argparse
import csv

from doghead_hil.tracking_metrics import TrackingMetricsAnalyzer


def main(args=None):
    parser = argparse.ArgumentParser(description="Generate a deterministic Doghead HIL tracking report")
    parser.add_argument("csv_path", help="CSV containing timestamp_s and target/measured PVA columns")
    parser.add_argument("--expected-rate-hz", type=float, default=50.0)
    parser.add_argument("--max-jitter-ms", type=float, default=10.0)
    parser.add_argument("--rate-tolerance-hz", type=float, default=None)
    parser.add_argument("--telemetry-timeout-ms", type=float, default=500.0)
    parsed = parser.parse_args(args)

    analyzer = TrackingMetricsAnalyzer()
    try:
        with open(parsed.csv_path, newline="", encoding="utf-8") as source:
            reader = csv.DictReader(source)
            fields = set(reader.fieldnames or [])
            if "physical_enable_mask" not in fields or "requested_enable_mask" not in fields:
                print(f"Error reading telemetry CSV '{parsed.csv_path}': missing required physical or requested enable mask field")
                return 1
            for row in reader:
                sequence = row.get("state_sequence", "")
                uptime = row.get("uptime_ms", "")
                boot_id = row.get("boot_id", "")
                analyzer.record_sample(
                    float(row["timestamp_s"]),
                    [float(row[f"target_pos_{i}"]) for i in range(3)],
                    [float(row[f"measured_pos_{i}"]) for i in range(3)],
                    [float(row[f"target_vel_{i}"]) for i in range(3)],
                    [float(row[f"measured_vel_{i}"]) for i in range(3)],
                    int(row.get("command_age_ms", 0) or 0),
                    state_sequence=int(sequence) if sequence else None,
                    uptime_ms=int(uptime) if uptime else None,
                    boot_id=int(boot_id) if boot_id else None,
                    applied_pos=[float(row.get(f"applied_pos_{i}", 0.0) or 0.0) for i in range(3)],
                    applied_vel=[float(row.get(f"applied_vel_{i}", 0.0) or 0.0) for i in range(3)],
                    position_error=[float(row.get(f"position_error_{i}", 0.0) or 0.0) for i in range(3)],
                    commanded_rpm=[float(row.get(f"commanded_rpm_{i}", 0.0) or 0.0) for i in range(3)],
                    pwm_frequency_hz=[int(row.get(f"pwm_frequency_hz_{i}", 0) or 0) for i in range(3)],
                    requested_enable_mask=int(row.get("requested_enable_mask", 0) or 0),
                    enable_mask=int(row.get("enable_mask", 0) or 0),
                    physical_enable_mask=int(row.get("physical_enable_mask", 0) or 0),
                    safety_state=int(row.get("safety_state", 0) or 0),
                    fault_code=int(row.get("fault_code", 0) or 0),
                    physical_gate_state=int(row.get("physical_gate_state", 0) or 0),
                )
    except Exception as err:
        print(f"Error reading telemetry CSV '{parsed.csv_path}': {err}")
        return 1

    report = analyzer.generate_summary_dict()
    telemetry_timeout_s = (
        parsed.telemetry_timeout_ms / 1000.0
        if parsed.telemetry_timeout_ms is not None
        else 0.5
    )
    report["timing_gate"] = analyzer.evaluate_timing(
        expected_rate_hz=parsed.expected_rate_hz,
        max_jitter_ms=parsed.max_jitter_ms,
        rate_tolerance_hz=parsed.rate_tolerance_hz,
        telemetry_timeout_s=telemetry_timeout_s,
    )
    print(analyzer.generate_markdown_report())
    gate = report["timing_gate"]
    print(
        f"Timing gate: {'PASS' if gate['pass'] else 'FAIL'}"
        f" (rate={'PASS' if gate['rate_pass'] else 'FAIL'},"
        f" jitter={'PASS' if gate['jitter_pass'] else 'FAIL'},"
        f" dropped={'FAIL' if gate['dropped_frame_violation'] else 'PASS'},"
        f" duplicate={'FAIL' if gate['duplicate_frame_violation'] else 'PASS'},"
        f" out_of_order={'FAIL' if gate['out_of_order_violation'] else 'PASS'},"
        f" clock_rollback={'FAIL' if gate['clock_rollback_violation'] else 'PASS'},"
        f" silence={'FAIL' if gate['telemetry_silence_violation'] else 'PASS'},"
        f" output_off={'FAIL' if gate['output_off_violation'] else 'PASS'},"
        f" non_finite={'FAIL' if gate['non_finite_telemetry_violation'] else 'PASS'},"
        f" insufficient_samples={'FAIL' if gate['insufficient_samples_violation'] else 'PASS'})"
    )
    return 0 if gate["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
