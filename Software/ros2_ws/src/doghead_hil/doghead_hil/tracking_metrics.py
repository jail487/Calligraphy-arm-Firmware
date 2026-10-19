import csv
import math
from typing import List, Dict, Any, Tuple


def is_telemetry_silent(last_rx_time: float, now_s: float, timeout_s: float) -> bool:
    """Return the fail-closed silence decision used by the live validator."""
    if last_rx_time is None or now_s is None or timeout_s is None:
        return True
    try:
        age = float(now_s) - float(last_rx_time)
        tout = float(timeout_s)
    except (ValueError, TypeError):
        return True
    return (not math.isfinite(age) or not math.isfinite(tout) or
            tout <= 0.0 or age < 0.0 or age > tout)


class TrackingMetricsAnalyzer:
    """
    Analyzes telemetry snapshots to calculate control tracking performance metrics:
    - Position RMSE & Max Absolute Error
    - Velocity RMSE
    - Telemetry inter-arrival rate & jitter
    - Command age statistics
    Treats floating-point round-off below FLOAT_EPSILON (1e-12) consistently as zero.
    """

    FLOAT_EPSILON = 1e-12

    def __init__(self, num_joints: int = 3, max_samples: int = 1000):
        self.num_joints = num_joints
        self.max_samples = max_samples
        self.timestamps: List[float] = []
        self.target_pos_history: List[List[float]] = []
        self.measured_pos_history: List[List[float]] = []
        self.target_vel_history: List[List[float]] = []
        self.measured_vel_history: List[List[float]] = []
        self.command_age_history: List[int] = []
        self.mcu_dt_history: List[float] = []
        self.state_sequence_history: List[int] = []
        self.uptime_history: List[int] = []
        self.boot_id_history: List[int] = []
        self.applied_pos_history: List[List[float]] = []
        self.applied_vel_history: List[List[float]] = []
        self.position_error_history: List[List[float]] = []
        self.commanded_rpm_history: List[List[float]] = []
        self.pwm_frequency_history: List[List[int]] = []
        self.requested_enable_mask_history: List[int] = []
        self.enable_mask_history: List[int] = []
        self.physical_enable_mask_history: List[int] = []
        self.safety_state_history: List[int] = []
        self.fault_code_history: List[int] = []
        self.physical_gate_state_history: List[int] = []
        self.non_finite_count: int = 0

        # Sequence & Uptime tracking attributes
        self.last_seq: int = None
        self.last_uptime: int = None
        self.last_boot_id: int = None
        self.sequence_received_count: int = 0
        self.dropped_frames: int = 0
        self.duplicate_frames: int = 0
        self.out_of_order_frames: int = 0
        self.clock_rollback_count: int = 0

    def record_sample(self, timestamp_s: float, target_pos: List[float], measured_pos: List[float],
                      target_vel: List[float], measured_vel: List[float], command_age_ms: int = 0,
                      state_sequence: int = None, uptime_ms: int = None, boot_id: int = None,
                      applied_pos: List[float] = None, applied_vel: List[float] = None,
                      position_error: List[float] = None, commanded_rpm: List[float] = None,
                      pwm_frequency_hz: List[int] = None, enable_mask: int = 0,
                      physical_enable_mask: int = 0,
                      safety_state: int = 0, fault_code: int = 0,
                      physical_gate_state: int = 0,
                      requested_enable_mask: int = 0):
        """Record a single telemetry frame."""
        def _check_finite(val) -> bool:
            if val is None:
                return True
            if isinstance(val, (list, tuple)):
                return all(_check_finite(x) for x in val)
            try:
                f = float(val)
                return math.isfinite(f)
            except (ValueError, TypeError):
                return False

        sample_inputs = [
            timestamp_s, target_pos, measured_pos, target_vel, measured_vel,
            command_age_ms, applied_pos, applied_vel, position_error,
            commanded_rpm, pwm_frequency_hz, requested_enable_mask,
            enable_mask, physical_enable_mask, safety_state, fault_code,
            physical_gate_state
        ]
        if not all(_check_finite(inp) for inp in sample_inputs):
            self.non_finite_count += 1

        try:
            ts = float(timestamp_s)
        except (ValueError, TypeError):
            ts = float('nan')

        if not math.isfinite(ts) or ts < 0.0:
            self.clock_rollback_count += 1
        elif self.timestamps and ts < self.timestamps[-1] - self.FLOAT_EPSILON:
            self.clock_rollback_count += 1

        self.timestamps.append(timestamp_s)
        self.target_pos_history.append(list(target_pos))
        self.measured_pos_history.append(list(measured_pos))
        self.target_vel_history.append(list(target_vel))
        self.measured_vel_history.append(list(measured_vel))
        self.command_age_history.append(command_age_ms)
        self.state_sequence_history.append(state_sequence)
        self.uptime_history.append(uptime_ms)
        self.boot_id_history.append(boot_id)
        self.applied_pos_history.append(list(applied_pos) if applied_pos is not None else [0.0] * self.num_joints)
        self.applied_vel_history.append(list(applied_vel) if applied_vel is not None else [0.0] * self.num_joints)
        self.position_error_history.append(list(position_error) if position_error is not None else [0.0] * self.num_joints)
        self.commanded_rpm_history.append(list(commanded_rpm) if commanded_rpm is not None else [0.0] * self.num_joints)
        self.pwm_frequency_history.append(list(pwm_frequency_hz) if pwm_frequency_hz is not None else [0] * self.num_joints)
        self.requested_enable_mask_history.append(int(requested_enable_mask))
        self.enable_mask_history.append(int(enable_mask))
        self.physical_enable_mask_history.append(int(physical_enable_mask))
        self.safety_state_history.append(int(safety_state))
        self.fault_code_history.append(int(fault_code))
        self.physical_gate_state_history.append(int(physical_gate_state))

        if state_sequence is not None and uptime_ms is not None:
            seq = int(state_sequence) & 0xFFFFFFFF
            uptime = int(uptime_ms) & 0xFFFFFFFF
            b_id = int(boot_id) & 0xFFFFFFFF if boot_id is not None else None

            if self.last_seq is None:
                self.last_seq = seq
                self.last_uptime = uptime
                self.last_boot_id = b_id
                self.sequence_received_count = 1
            elif b_id is not None and self.last_boot_id is not None and b_id != self.last_boot_id:
                # MCU boot reset detected: re-baseline without false loss
                self.out_of_order_frames += 1
                self.sequence_received_count += 1
                self.last_seq = seq
                self.last_uptime = uptime
                self.last_boot_id = b_id
            else:
                seq_diff = (seq - self.last_seq) & 0xFFFFFFFF
                if seq_diff == 0:
                    self.duplicate_frames += 1
                elif seq_diff < 0x80000000:
                    dropped = seq_diff - 1
                    self.dropped_frames += dropped
                    self.sequence_received_count += 1
                    uptime_diff = (uptime - self.last_uptime) & 0xFFFFFFFF
                    if 0 < uptime_diff < 0x80000000:
                        mcu_dt = float(uptime_diff) / float(seq_diff)
                        self.mcu_dt_history.append(mcu_dt)
                    elif uptime_diff >= 0x80000000:
                        self.clock_rollback_count += 1
                    self.last_seq = seq
                    self.last_uptime = uptime
                    if b_id is not None:
                        self.last_boot_id = b_id
                else:
                    # Out-of-order / backward frame within same boot: count it but preserve forward baseline
                    self.out_of_order_frames += 1
                    self.sequence_received_count += 1

        if self.max_samples is not None and len(self.timestamps) > self.max_samples:
            self.timestamps.pop(0)
            self.target_pos_history.pop(0)
            self.measured_pos_history.pop(0)
            self.target_vel_history.pop(0)
            self.measured_vel_history.pop(0)
            self.command_age_history.pop(0)
            self.state_sequence_history.pop(0)
            self.uptime_history.pop(0)
            self.boot_id_history.pop(0)
            self.applied_pos_history.pop(0)
            self.applied_vel_history.pop(0)
            self.position_error_history.pop(0)
            self.commanded_rpm_history.pop(0)
            self.pwm_frequency_history.pop(0)
            self.requested_enable_mask_history.pop(0)
            self.enable_mask_history.pop(0)
            self.physical_enable_mask_history.pop(0)
            self.safety_state_history.pop(0)
            self.fault_code_history.pop(0)
            self.physical_gate_state_history.pop(0)
            if len(self.mcu_dt_history) > self.max_samples:
                self.mcu_dt_history.pop(0)

    def calculate_position_rmse(self) -> Tuple[List[float], float]:
        """Returns per-joint RMSE (rad) and overall RMSE (rad)."""
        N = len(self.timestamps)
        if N == 0:
            return [0.0] * self.num_joints, 0.0

        sq_sum = [0.0] * self.num_joints
        for i in range(N):
            for j in range(self.num_joints):
                err = self.target_pos_history[i][j] - self.measured_pos_history[i][j]
                if abs(err) < self.FLOAT_EPSILON:
                    err = 0.0
                sq_sum[j] += err * err

        joint_rmse = [math.sqrt(sq / N) for sq in sq_sum]
        overall_rmse = math.sqrt(sum(sq_sum) / (N * self.num_joints))
        return joint_rmse, overall_rmse

    def calculate_max_position_error(self) -> List[float]:
        """Returns per-joint maximum absolute error (rad)."""
        N = len(self.timestamps)
        if N == 0:
            return [0.0] * self.num_joints

        max_err = [0.0] * self.num_joints
        for i in range(N):
            for j in range(self.num_joints):
                err = abs(self.target_pos_history[i][j] - self.measured_pos_history[i][j])
                if err < self.FLOAT_EPSILON:
                    err = 0.0
                if err > max_err[j]:
                    max_err[j] = err

        return max_err

    def calculate_telemetry_jitter(self) -> Tuple[float, float, float]:
        """
        Legacy compatibility wrapper.  The third value is max period, not
        jitter standard deviation or max deviation.  New callers should use
        calculate_telemetry_timing() with explicitly named fields.
        """
        N = len(self.timestamps)
        if N < 2:
            return 0.0, 0.0, 0.0

        dt_ms = [(self.timestamps[i] - self.timestamps[i - 1]) * 1000.0 for i in range(1, N)]
        mean_dt = sum(dt_ms) / len(dt_ms)
        variance = sum((dt - mean_dt) ** 2 for dt in dt_ms) / len(dt_ms)
        std_dev = math.sqrt(variance)
        if std_dev < self.FLOAT_EPSILON:
            std_dev = 0.0
        max_dt = max(dt_ms)

        return mean_dt, std_dev, max_dt

    def calculate_telemetry_timing(self) -> Dict[str, float]:
        """Return separately named period, jitter SD, max period and deviation."""
        if len(self.timestamps) < 2:
            return {
                "mean_period_ms": 0.0,
                "jitter_std_ms": 0.0,
                "max_period_ms": 0.0,
                "max_period_deviation_ms": 0.0,
            }

        periods_ms = [
            (self.timestamps[i] - self.timestamps[i - 1]) * 1000.0
            for i in range(1, len(self.timestamps))
        ]
        mean_period = sum(periods_ms) / len(periods_ms)
        return {
            "mean_period_ms": mean_period,
            "jitter_std_ms": math.sqrt(
                sum((period - mean_period) ** 2 for period in periods_ms) / len(periods_ms)
            ),
            "max_period_ms": max(periods_ms),
            "max_period_deviation_ms": max(abs(period - mean_period) for period in periods_ms),
        }

    def evaluate_timing(self, expected_rate_hz: float, max_jitter_ms: float,
                        rate_tolerance_hz: float = None,
                        telemetry_timeout_s: float = None,
                        last_rx_time: float = None,
                        now_s: float = None,
                        min_samples: int = 2,
                        actuator_hardware_output_enabled: bool = False) -> Dict[str, Any]:
        """Apply rate, jitter, dropped, duplicate, out-of-order, silence, rollback, threshold, and output-off acceptance gates."""
        try:
            exp_rate = float(expected_rate_hz) if expected_rate_hz is not None else None
            max_jit = float(max_jitter_ms) if max_jitter_ms is not None else None
            rate_tol = float(rate_tolerance_hz) if rate_tolerance_hz is not None else None
            tout_s = float(telemetry_timeout_s) if telemetry_timeout_s is not None else None
        except (ValueError, TypeError):
            exp_rate = None
            max_jit = None
            rate_tol = None
            tout_s = None

        invalid_thresholds_violation = False
        if exp_rate is None or not math.isfinite(exp_rate) or exp_rate <= 0.0:
            invalid_thresholds_violation = True
        if max_jit is None or not math.isfinite(max_jit) or max_jit < 0.0:
            invalid_thresholds_violation = True
        if rate_tol is not None and (not math.isfinite(rate_tol) or rate_tol < 0.0):
            invalid_thresholds_violation = True
        if tout_s is not None and (not math.isfinite(tout_s) or tout_s <= 0.0):
            invalid_thresholds_violation = True

        if rate_tol is None and exp_rate is not None and exp_rate > 0.0:
            rate_tol = max(0.5, abs(exp_rate) * 0.05)
        elif rate_tol is None:
            rate_tol = 0.5

        sample_count = len(self.timestamps)
        insufficient_samples_violation = (sample_count < min_samples)

        clock_rollback_violation = (self.clock_rollback_count > 0)
        if not clock_rollback_violation and sample_count >= 2:
            for i in range(1, sample_count):
                ts_curr = float(self.timestamps[i]) if self.timestamps[i] is not None else float('nan')
                ts_prev = float(self.timestamps[i - 1]) if self.timestamps[i - 1] is not None else float('nan')
                if not math.isfinite(ts_curr) or not math.isfinite(ts_prev) or ts_curr < ts_prev - self.FLOAT_EPSILON:
                    clock_rollback_violation = True
                    break

        timing = self.calculate_telemetry_timing()
        mean_period = timing["mean_period_ms"]
        measured_rate = 1000.0 / mean_period if mean_period > 0.0 else 0.0
        rate_error = abs(measured_rate - exp_rate) if (exp_rate is not None and exp_rate > 0.0) else float('inf')

        rate_pass = (
            not invalid_thresholds_violation and
            not insufficient_samples_violation and
            not clock_rollback_violation and
            rate_error <= rate_tol
        )
        jitter_pass = (
            not invalid_thresholds_violation and
            not insufficient_samples_violation and
            not clock_rollback_violation and
            timing["max_period_deviation_ms"] <= max_jit
        )

        dropped_frame_violation = (self.dropped_frames > 0)
        duplicate_frame_violation = (self.duplicate_frames > 0)
        out_of_order_violation = (self.out_of_order_frames > 0)

        if last_rx_time is not None and now_s is not None and tout_s is not None:
            telemetry_silence_violation = is_telemetry_silent(last_rx_time, now_s, tout_s)
        elif sample_count == 0:
            telemetry_silence_violation = True
        elif tout_s is not None:
            max_gap_s = max(
                (self.timestamps[i] - self.timestamps[i - 1] for i in range(1, sample_count)),
                default=0.0
            ) if sample_count >= 2 else 0.0
            telemetry_silence_violation = (max_gap_s > tout_s)
        else:
            telemetry_silence_violation = False

        output_off_violation = (
            not actuator_hardware_output_enabled and
            (any(mask != 0 for mask in self.physical_enable_mask_history) or
             any(gate != 0 for gate in self.physical_gate_state_history))
        )

        non_finite_telemetry_violation = (self.non_finite_count > 0)

        overall_pass = bool(
            rate_pass and
            jitter_pass and
            not dropped_frame_violation and
            not duplicate_frame_violation and
            not out_of_order_violation and
            not clock_rollback_violation and
            not invalid_thresholds_violation and
            not insufficient_samples_violation and
            not telemetry_silence_violation and
            not output_off_violation and
            not non_finite_telemetry_violation
        )

        return {
            "pass": overall_pass,
            "expected_rate_hz": float(exp_rate) if exp_rate is not None else 0.0,
            "measured_rate_hz": measured_rate,
            "rate_error_hz": rate_error if math.isfinite(rate_error) else 0.0,
            "rate_tolerance_hz": float(rate_tol),
            "rate_pass": bool(rate_pass),
            "max_jitter_ms": float(max_jit) if max_jit is not None else 0.0,
            "jitter_pass": bool(jitter_pass),
            "dropped_frame_violation": bool(dropped_frame_violation),
            "dropped_frames": int(self.dropped_frames),
            "duplicate_frame_violation": bool(duplicate_frame_violation),
            "duplicate_frames": int(self.duplicate_frames),
            "out_of_order_violation": bool(out_of_order_violation),
            "out_of_order_frames": int(self.out_of_order_frames),
            "clock_rollback_violation": bool(clock_rollback_violation),
            "clock_rollback_count": int(self.clock_rollback_count),
            "invalid_thresholds_violation": bool(invalid_thresholds_violation),
            "insufficient_samples_violation": bool(insufficient_samples_violation),
            "telemetry_silence_violation": bool(telemetry_silence_violation),
            "output_off_violation": bool(output_off_violation),
            "non_finite_telemetry_violation": bool(non_finite_telemetry_violation),
            "non_finite_count": int(self.non_finite_count),
            "sample_count": sample_count,
        }

    def export_csv(self, path: str) -> None:
        """Write the recorded samples in a deterministic, reportable schema."""
        fields = [
            "timestamp_s", "command_age_ms", "state_sequence", "uptime_ms", "boot_id",
            *[f"target_pos_{i}" for i in range(self.num_joints)],
            *[f"measured_pos_{i}" for i in range(self.num_joints)],
            *[f"target_vel_{i}" for i in range(self.num_joints)],
            *[f"measured_vel_{i}" for i in range(self.num_joints)],
            *[f"applied_pos_{i}" for i in range(self.num_joints)],
            *[f"applied_vel_{i}" for i in range(self.num_joints)],
            *[f"position_error_{i}" for i in range(self.num_joints)],
            *[f"commanded_rpm_{i}" for i in range(self.num_joints)],
            *[f"pwm_frequency_hz_{i}" for i in range(self.num_joints)],
            "requested_enable_mask", "enable_mask", "physical_enable_mask", "safety_state", "fault_code", "physical_gate_state",
        ]
        with open(path, "w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=fields)
            writer.writeheader()
            for i, timestamp in enumerate(self.timestamps):
                row = {
                    "timestamp_s": timestamp,
                    "command_age_ms": self.command_age_history[i],
                    "state_sequence": "" if self.state_sequence_history[i] is None else self.state_sequence_history[i],
                    "uptime_ms": "" if self.uptime_history[i] is None else self.uptime_history[i],
                    "boot_id": "" if self.boot_id_history[i] is None else self.boot_id_history[i],
                }
                for j in range(self.num_joints):
                    row[f"target_pos_{j}"] = self.target_pos_history[i][j]
                    row[f"measured_pos_{j}"] = self.measured_pos_history[i][j]
                    row[f"target_vel_{j}"] = self.target_vel_history[i][j]
                    row[f"measured_vel_{j}"] = self.measured_vel_history[i][j]
                    row[f"applied_pos_{j}"] = self.applied_pos_history[i][j]
                    row[f"applied_vel_{j}"] = self.applied_vel_history[i][j]
                    row[f"position_error_{j}"] = self.position_error_history[i][j]
                    row[f"commanded_rpm_{j}"] = self.commanded_rpm_history[i][j]
                    row[f"pwm_frequency_hz_{j}"] = self.pwm_frequency_history[i][j]
                row["requested_enable_mask"] = self.requested_enable_mask_history[i]
                row["enable_mask"] = self.enable_mask_history[i]
                row["physical_enable_mask"] = self.physical_enable_mask_history[i]
                row["safety_state"] = self.safety_state_history[i]
                row["fault_code"] = self.fault_code_history[i]
                row["physical_gate_state"] = self.physical_gate_state_history[i]
                writer.writerow(row)

    def generate_report(self) -> Dict[str, Any]:
        """Alias for generate_summary_dict."""
        return self.generate_summary_dict()

    def generate_summary_dict(self) -> Dict[str, Any]:
        """Generate comprehensive tracking report dictionary."""
        N = len(self.timestamps)
        if N == 0:
            return {"sample_count": 0, "status": "No data recorded"}

        joint_pos_rmse, overall_pos_rmse = self.calculate_position_rmse()
        max_pos_err = self.calculate_max_position_error()
        timing = self.calculate_telemetry_timing()
        mean_dt_ms = timing["mean_period_ms"]
        std_dev_jitter_ms = timing["jitter_std_ms"]
        max_dt_ms = timing["max_period_ms"]

        mcu_period_mean_ms = (
            sum(self.mcu_dt_history) / len(self.mcu_dt_history)
            if len(self.mcu_dt_history) > 0 else 0.0
        )
        mcu_rate_hz = 1000.0 / mcu_period_mean_ms if mcu_period_mean_ms > 0 else 0.0
        host_rate_hz = 1000.0 / mean_dt_ms if mean_dt_ms > 0 else 0.0

        mean_age = sum(self.command_age_history) / N if N > 0 else 0
        max_age = max(self.command_age_history) if N > 0 else 0
        max_position_error = max(
            (max(abs(value) for value in error) for error in self.position_error_history),
            default=0.0,
        )
        max_commanded_rpm = max(
            (max(abs(value) for value in rpm) for rpm in self.commanded_rpm_history),
            default=0.0,
        )

        return {
            "sample_count": N,
            "overall_position_rmse_rad": overall_pos_rmse,
            "joint_position_rmse_rad": joint_pos_rmse,
            "joint_max_pos_error_rad": max_pos_err,
            "telemetry_period_mean_ms": mean_dt_ms,
            "telemetry_jitter_std_ms": std_dev_jitter_ms,
            "telemetry_period_max_ms": max_dt_ms,
            "telemetry_period_max_deviation_ms": timing["max_period_deviation_ms"],
            "host_rate_hz": host_rate_hz,
            "mcu_period_mean_ms": mcu_period_mean_ms,
            "mcu_rate_hz": mcu_rate_hz,
            "state_sequence_total_received": self.sequence_received_count,
            "state_sequence_dropped": self.dropped_frames,
            "state_sequence_duplicates": self.duplicate_frames,
            "state_sequence_out_of_order": self.out_of_order_frames,
            "clock_rollback_count": self.clock_rollback_count,
            "command_age_mean_ms": mean_age,
            "command_age_max_ms": max_age,
            "position_error_max_rad": max_position_error,
            "commanded_motor_rpm_max": max_commanded_rpm,
            "requested_enable_mask_last": self.requested_enable_mask_history[-1] if self.requested_enable_mask_history else 0,
            "logical_enable_mask_last": self.enable_mask_history[-1],
            "physical_enable_mask_last": self.physical_enable_mask_history[-1],
            "physical_gate_state_last": self.physical_gate_state_history[-1],
        }

    def generate_markdown_report(self) -> str:
        """Format metrics into readable Markdown report."""
        rep = self.generate_summary_dict()
        if rep.get("sample_count", 0) == 0:
            return "### HIL Tracking Report\nNo telemetry data recorded."

        lines = [
            "### HIL Tracking Performance Report",
            f"- **Sample Count**: {rep['sample_count']}",
            f"- **Overall Position RMSE**: {rep['overall_position_rmse_rad']:.6f} rad",
            "- **Per-Joint Position RMSE (rad)**:",
            f"  - Joint 1: {rep['joint_position_rmse_rad'][0]:.6f} | Max Error: {rep['joint_max_pos_error_rad'][0]:.6f}",
            f"  - Joint 2: {rep['joint_position_rmse_rad'][1]:.6f} | Max Error: {rep['joint_max_pos_error_rad'][1]:.6f}",
            f"  - Joint 3: {rep['joint_position_rmse_rad'][2]:.6f} | Max Error: {rep['joint_max_pos_error_rad'][2]:.6f}",
            "- **Telemetry Timing & Sequence Metrics**:",
            f"  - Host Inter-arrival: Mean {rep['telemetry_period_mean_ms']:.2f} ms ({rep['host_rate_hz']:.1f} Hz) | Jitter SD: {rep['telemetry_jitter_std_ms']:.3f} ms | Max Period: {rep['telemetry_period_max_ms']:.2f} ms | Max Deviation: {rep['telemetry_period_max_deviation_ms']:.2f} ms" if rep['telemetry_period_mean_ms'] > 0 else "  - Host Inter-arrival: N/A",
            f"  - MCU Source Timing: Mean {rep['mcu_period_mean_ms']:.2f} ms ({rep['mcu_rate_hz']:.1f} Hz)" if rep['mcu_period_mean_ms'] > 0 else "  - MCU Source Timing: N/A",
            f"  - Sequence Continuity: Received {rep['state_sequence_total_received']} | Dropped: {rep['state_sequence_dropped']} | Duplicates: {rep['state_sequence_duplicates']} | Out-of-order/Reset: {rep['state_sequence_out_of_order']}",
            "- **Command Age Metrics**:",
            f"  - Mean Age: {rep['command_age_mean_ms']:.1f} ms | Max Age: {rep['command_age_max_ms']} ms",
            "- **Applied/Output Provenance**:",
            f"  - Max Position Error: {rep['position_error_max_rad']:.6f} rad | Max Commanded RPM: {rep['commanded_motor_rpm_max']:.3f}",
            f"  - Last Requested Enable Mask: {rep['requested_enable_mask_last']} | Last Logical Enable Mask: {rep['logical_enable_mask_last']} | Last Physical Enable Mask: {rep['physical_enable_mask_last']} | Last Physical Gate State: {rep['physical_gate_state_last']}",
        ]
        return "\n".join(lines)
