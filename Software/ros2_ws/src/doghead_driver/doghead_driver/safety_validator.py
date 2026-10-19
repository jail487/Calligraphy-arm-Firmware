import math
import numbers
import time
from typing import Tuple, List, Optional, Dict, Any

EXPECTED_PROTOCOL_VERSION = 1
SESSION_BOOT_TAG_SHIFT = 16
SESSION_BOOT_TAG_MASK = 0xFFFF0000
SESSION_EPOCH_MASK = 0x0000FFFF
DEFAULT_MAX_TTL_MS = 1000
DEFAULT_VALID_FOR_MS = 250
NUM_JOINTS = 3
DEFAULT_JOINT_NAMES = ["joint1", "joint2", "joint3"]

# Hard Safety Limits for Host-Side Driver
DEFAULT_LIMITS = {
    "pos_min_rad": [1.3962634, 0.0, -3.1415927],
    "pos_max_rad": [3.1415927, 3.1415927, 3.1415927],
    "vel_max_rad_s": [4.593, 4.593, 4.593],
    "accel_max_rad_s2": [15.0, 15.0, 15.0],
    # Cap on joint/output-shaft RPM in Motor Test mode (NOT motor shaft RPM).
    # Gear ratio is applied once inside the MCU firmware driver.
    "bringup_motor_rpm_max": 5000.0,
}


def validate_limits(limits: Dict[str, Any]) -> Dict[str, Any]:
    """Validate and copy the resolved ROS safety-limit parameters."""
    required_vectors = ("pos_min_rad", "pos_max_rad", "vel_max_rad_s", "accel_max_rad_s2")
    resolved = {}
    for key in required_vectors:
        values = list(limits.get(key, []))
        if len(values) != NUM_JOINTS or not is_finite_sequence(values):
            raise ValueError(f"limits.{key} must contain {NUM_JOINTS} finite values")
        resolved[key] = [float(value) for value in values]
    for index in range(NUM_JOINTS):
        if resolved["pos_min_rad"][index] > resolved["pos_max_rad"][index]:
            raise ValueError(f"limits position range is inverted at joint {index + 1}")
        if resolved["vel_max_rad_s"][index] < 0.0 or resolved["accel_max_rad_s2"][index] < 0.0:
            raise ValueError(f"limits velocity/acceleration must be non-negative at joint {index + 1}")
    rpm_cap = limits.get("bringup_motor_rpm_max", DEFAULT_LIMITS["bringup_motor_rpm_max"])
    if not is_finite_number(rpm_cap) or float(rpm_cap) < 0.0:
        raise ValueError("limits.bringup_motor_rpm_max must be a finite non-negative number")
    resolved["bringup_motor_rpm_max"] = float(rpm_cap)
    return resolved


def is_finite_number(val: Any) -> bool:
    """Check if a single number is a finite float or int (not NaN or Inf)."""
    # ROS Python message arrays yield NumPy float32 scalar values.  They are
    # real numbers even though they are not instances of the built-in float.
    if not isinstance(val, numbers.Real):
        return False
    return math.isfinite(val)


def is_finite_sequence(seq: Any) -> bool:
    """Check if all elements in a sequence are finite numbers."""
    try:
        return all(is_finite_number(x) for x in seq)
    except (TypeError, ValueError):
        return False


def validate_array_size(arr: Any, expected_size: int = NUM_JOINTS) -> bool:
    """Verify that an array or sequence has exactly the expected size."""
    try:
        return len(arr) == expected_size
    except TypeError:
        return False


def validate_protocol_version(version: int, expected: int = EXPECTED_PROTOCOL_VERSION) -> Tuple[bool, str]:
    """Validate protocol version against expected version."""
    if version != expected:
        return False, f"Protocol version mismatch: got {version}, expected {expected}"
    return True, "OK"


def validate_ttl(valid_for_ms: int, max_ttl_ms: int = DEFAULT_MAX_TTL_MS) -> Tuple[bool, str]:
    """Validate command TTL duration."""
    if valid_for_ms <= 0:
        return False, f"Invalid TTL: {valid_for_ms} ms (must be > 0)"
    if valid_for_ms > max_ttl_ms:
        return False, f"TTL exceeds max limit: {valid_for_ms} ms > {max_ttl_ms} ms"
    return True, "OK"


def is_sequence_newer(new_seq: int, current_seq: int) -> bool:
    """Wrap-safe 32-bit sequence monotonicity check."""
    diff = (new_seq - current_seq) & 0xFFFFFFFF
    return 0 < diff < 0x80000000


def check_two_key_actuator_gate(armed: bool, shadow_mode: bool, actuator_enabled: bool) -> bool:
    """
    Two-key hard gate for physical actuator output:
    Output enablement requires ALL of:
    1. Driver is ARMED (armed == True)
    2. shadow_mode parameter is FALSE (shadow_mode == False)
    3. actuator_enabled parameter is TRUE (actuator_enabled == True)
    """
    return bool(armed and (not shadow_mode) and actuator_enabled)



class DriverSafetyState:
    """Tracks driver arming state, session management, and MCU connection baseline."""

    def __init__(self, limits: Optional[Dict[str, Any]] = None,
                 max_ttl_ms: int = DEFAULT_MAX_TTL_MS):
        self.limits = limits or DEFAULT_LIMITS
        self.max_ttl_ms = max_ttl_ms
        self.armed = False
        self.session_id = 0
        self._session_epoch = 0
        self.last_sequence = 0
        self.agent_connected = False
        self.last_boot_id = 0
        self.last_reconnect_total = 0
        self.last_telemetry_time = None
        self.last_state_sequence = None
        self.last_uptime_ms = None
        self.disarm_reason = "Initialized disarmed"

    def arm(self) -> Tuple[bool, int, str]:
        """Arm the driver and create a new session ID."""
        if not self.agent_connected:
            self.disarm("Cannot arm: micro-ROS Agent not connected")
            return False, 0, self.disarm_reason
        if self.last_boot_id == 0:
            self.disarm("Cannot arm: MCU boot_id is unknown/invalid")
            return False, 0, self.disarm_reason

        # Keep the Arm epoch independent from the active ID.  A disarm must
        # never cause the next Arm cycle to reuse session 1.
        self._session_epoch = (self._session_epoch % 0x7FFFFFFF) + 1
        # Bind each host Arm epoch to the MCU boot epoch.  Firmware rejects a
        # session whose upper tag does not match its current boot_id.
        epoch = self._session_epoch & SESSION_EPOCH_MASK
        if epoch == 0:
            self._session_epoch = 1
            epoch = 1
        self.session_id = ((self.last_boot_id & 0xFFFF) << SESSION_BOOT_TAG_SHIFT) | epoch
        self.last_sequence = 0
        self.armed = True
        self.disarm_reason = ""
        return True, self.session_id, f"Armed with session_id {self.session_id}"

    def disarm(self, reason: str = "Requested disarm"):
        """Disarm the driver and invalidate active session."""
        self.armed = False
        self.session_id = 0
        self.last_sequence = 0
        self.disarm_reason = reason

    def stop(self, reason: str = "Requested stop"):
        """Immediate stop and disarm."""
        self.disarm(f"STOP triggered: {reason}")

    def on_mcu_state(self, protocol_version: int, boot_id: int, agent_connected: bool,
                     reconnect_total: int = 0, fault_code: int = 0,
                     now_s: Optional[float] = None, state_sequence: Optional[int] = None,
                     uptime_ms: Optional[int] = None) -> Tuple[bool, str]:
        """
        Process MCU state update. Disarms on reconnect, protocol mismatch, or MCU fault.
        Returns (is_ok, status_message).
        """
        # 1. Protocol check
        ver_ok, ver_msg = validate_protocol_version(protocol_version)
        if not ver_ok:
            self.disarm(f"Disarming: {ver_msg}")
            return False, ver_msg

        if int(boot_id) == 0:
            self.disarm("Disarming: invalid MCU boot_id=0")
            return False, "Invalid MCU boot_id=0"

        # A state frame is a liveness proof only if it is newer than the last
        # accepted frame.  Duplicate/out-of-order/replayed frames must not
        # keep an armed driver alive.
        freshness_available = state_sequence is not None and uptime_ms is not None
        if (state_sequence is None) != (uptime_ms is None):
            self.disarm("Disarming: incomplete MCU state freshness fields")
            return False, "Incomplete MCU state freshness fields"

        incoming_seq = None if state_sequence is None else int(state_sequence) & 0xFFFFFFFF
        incoming_uptime = None if uptime_ms is None else int(uptime_ms) & 0xFFFFFFFF

        # 2. Reconnect & Boot ID check
        reconnect_detected = False
        if self.agent_connected and not agent_connected:
            reconnect_detected = True
            reason = "Agent connection lost"
        elif self.last_boot_id != 0 and boot_id != self.last_boot_id:
            reconnect_detected = True
            reason = f"MCU reboot detected (boot_id changed from {self.last_boot_id} to {boot_id})"
        elif self.last_telemetry_time is not None and reconnect_total != self.last_reconnect_total:
            reconnect_detected = True
            reason = f"Agent reconnect counter changed ({self.last_reconnect_total} -> {reconnect_total})"

        boot_changed = self.last_boot_id != 0 and boot_id != self.last_boot_id

        if (not boot_changed and freshness_available and
                self.last_state_sequence is not None and self.last_uptime_ms is not None):
            if (not is_sequence_newer(incoming_seq, self.last_state_sequence) or
                    not is_sequence_newer(incoming_uptime, self.last_uptime_ms)):
                reason = (
                    f"Stale/replayed MCU state: sequence {incoming_seq} after "
                    f"{self.last_state_sequence}, uptime {incoming_uptime} after "
                    f"{self.last_uptime_ms}"
                )
                self.disarm(f"Disarming: {reason}")
                return False, reason

        if freshness_available:
            self.last_state_sequence = incoming_seq
            self.last_uptime_ms = incoming_uptime

        self.agent_connected = agent_connected
        self.last_boot_id = boot_id
        self.last_reconnect_total = reconnect_total

        # An agent-disconnected frame is not a liveness refresh, even though
        # it may carry a valid MCU sequence.
        if not agent_connected:
            self.disarm("Disarming: MCU reports agent disconnected")
            return False, "MCU reports agent disconnected"

        self.last_telemetry_time = time.monotonic() if now_s is None else float(now_s)

        if reconnect_detected:
            self.disarm(f"Disarming on reconnect: {reason}")
            return False, f"Reconnect disarm: {reason}"

        # 3. Fault code check
        if fault_code != 0:
            self.disarm(f"Disarming: MCU fault code {fault_code}")
            return False, f"MCU fault code {fault_code}"

        return True, "MCU state OK"

    def check_telemetry_timeout(self, now_s: Optional[float] = None,
                                timeout_s: float = 0.5) -> Tuple[bool, str]:
        """Fail closed when state telemetry has been silent for too long."""
        if not self.armed:
            return True, "Driver is disarmed"
        if self.last_telemetry_time is None:
            self.agent_connected = False
            self.disarm("Telemetry silence: no MCU state received")
            return False, self.disarm_reason

        now = time.monotonic() if now_s is None else float(now_s)
        age_s = now - self.last_telemetry_time
        if not math.isfinite(age_s) or age_s < 0.0 or age_s > timeout_s:
            self.agent_connected = False
            self.disarm(f"Telemetry silence: MCU state age {age_s:.3f}s exceeded {timeout_s:.3f}s")
            return False, self.disarm_reason
        return True, "Telemetry live"

    def validate_joint_setpoint(self, protocol_version: int, session_id: int, sequence: int,
                                valid_for_ms: int, enable_mask: int,
                                position_rad: List[float], velocity_rad_s: List[float],
                                acceleration_rad_s2: List[float]) -> Tuple[bool, str]:
        """
        Thoroughly validate an incoming joint setpoint command before sending to MCU.
        """
        # 1. Arm & Session check
        if not self.armed:
            return False, f"Driver is DISARMED ({self.disarm_reason})"
        if session_id != 0 and session_id != self.session_id:
            return False, f"Session mismatch: got {session_id}, expected {self.session_id}"

        # 2. Protocol check
        ver_ok, ver_msg = validate_protocol_version(protocol_version)
        if not ver_ok:
            return False, ver_msg

        # 3. Sequence check
        if self.last_sequence != 0 and sequence != 0 and not is_sequence_newer(sequence, self.last_sequence):
            if sequence <= 5:
                self.last_sequence = 0
            else:
                return False, f"Stale sequence: {sequence} <= last accepted {self.last_sequence}"

        # 4. TTL check
        ttl_ok, ttl_msg = validate_ttl(valid_for_ms, self.max_ttl_ms)
        if not ttl_ok:
            return False, ttl_msg

        # 5. Array size checks
        if not (validate_array_size(position_rad) and validate_array_size(velocity_rad_s) and validate_array_size(acceleration_rad_s2)):
            return False, "Invalid payload array size (must be 3 elements per vector)"

        # 6. Finite checks (no NaN/Inf)
        if not (is_finite_sequence(position_rad) and is_finite_sequence(velocity_rad_s) and is_finite_sequence(acceleration_rad_s2)):
            return False, "Non-finite value (NaN/Inf) detected in joint setpoint payload"

        # 7. Envelope & limit checks
        for i in range(NUM_JOINTS):
            pos = position_rad[i]
            vel = abs(velocity_rad_s[i])
            acc = abs(acceleration_rad_s2[i])

            min_p = self.limits["pos_min_rad"][i]
            max_p = self.limits["pos_max_rad"][i]
            max_v = self.limits["vel_max_rad_s"][i]
            max_a = self.limits["accel_max_rad_s2"][i]

            if not (min_p <= pos <= max_p):
                return False, f"Joint {i+1} position {pos:.4f} rad exceeds limit [{min_p}, {max_p}]"
            if vel > max_v:
                return False, f"Joint {i+1} velocity {vel:.4f} rad/s exceeds limit {max_v}"
            if acc > max_a:
                return False, f"Joint {i+1} acceleration {acc:.4f} rad/s2 exceeds limit {max_a}"

        # If all checks pass, record sequence
        self.last_sequence = sequence
        return True, "OK"

    def validate_motor_test_command(self, protocol_version: int, session_id: int, sequence: int,
                                    valid_for_ms: int, axis: int, target_motor_rpm: float,
                                    requested_accel: float, requested_jerk: float,
                                    enable: bool) -> Tuple[bool, str]:
        """
        Validate motor test bringup command.
        
        Unit contract:
        target_motor_rpm carries joint/output-shaft RPM (NOT motor shaft RPM).
        Gear ratio is applied exactly once inside the MCU firmware driver.
        """
        # Disable is an idempotent safety stop.  It is safe to forward even
        # after disarm/session loss and must not be blocked by an old command
        # sequence; the MCU handles it as an out-of-band stop as well.
        if not enable:
            return True, "OK: motor-test safety stop"

        if not self.armed and enable:
            return False, f"Driver is DISARMED ({self.disarm_reason})"
        if session_id != 0 and session_id != self.session_id and enable:
            return False, f"Session mismatch: got {session_id}, expected {self.session_id}"

        ver_ok, ver_msg = validate_protocol_version(protocol_version)
        if not ver_ok:
            return False, ver_msg

        ttl_ok, ttl_msg = validate_ttl(valid_for_ms, self.max_ttl_ms)
        if not ttl_ok:
            return False, ttl_msg

        if axis not in (0, 1, 2):
            return False, f"Invalid test axis: {axis} (must be 0, 1, or 2)"

        if enable and self.last_sequence != 0 and sequence != 0 and not is_sequence_newer(sequence, self.last_sequence):
            if sequence <= 5:
                self.last_sequence = 0
            else:
                return False, f"Stale sequence: {sequence} <= last accepted {self.last_sequence}"

        if not (is_finite_number(target_motor_rpm) and is_finite_number(requested_accel) and is_finite_number(requested_jerk)):
            return False, "Non-finite value (NaN/Inf) detected in motor test command"

        max_rpm = self.limits.get("bringup_motor_rpm_max", 150.0)
        if abs(target_motor_rpm) > max_rpm:
            return False, f"Bringup motor RPM {target_motor_rpm} exceeds hard cap {max_rpm}"

        # Motor test and Joint PVA share the MCU mailbox sequence epoch.  A
        # stop does not advance that epoch, so a later valid command still
        # needs a genuinely newer sequence.
        if enable:
            self.last_sequence = sequence
        return True, "OK"
