import math
from typing import List, Tuple, Dict, Any, Optional

DEFAULT_MAX_VELOCITY = [0.174533, 0.174533, 0.174533]  # rad/s; conservative 10 deg/s
DEFAULT_MAX_ACCEL = [0.349066, 0.349066, 0.349066]    # rad/s^2; conservative 20 deg/s^2


class JointPvaTrajectoryGenerator:
    """
    Generates time-parameterized Joint PVA (Position, Velocity, Acceleration) setpoints
    subject to joint velocity and acceleration bounds.
    """

    def __init__(self, max_velocity: Optional[List[float]] = None,
                 max_acceleration: Optional[List[float]] = None):
        self.max_v = max_velocity or DEFAULT_MAX_VELOCITY
        self.max_a = max_acceleration or DEFAULT_MAX_ACCEL

    def compute_motion_time(self, start_pos: List[float], target_pos: List[float]) -> float:
        """Compute minimum travel time required to satisfy velocity & acceleration constraints across all joints."""
        max_time = 0.05  # minimum 50 ms motion duration
        for i in range(3):
            dist = abs(target_pos[i] - start_pos[i])
            v_max = self.max_v[i]
            a_max = self.max_a[i]

            # The quintic sample below has peak |ds/dt|=1.875 and peak
            # |d2s/dt2|≈5.7735.  Include those shape factors so every emitted
            # PVA sample remains inside the declared limits.
            t_v = 1.875 * dist / v_max if v_max > 0 else 0.0
            t_a = math.sqrt(5.773503 * dist / a_max) if a_max > 0 else 0.0

            t_joint = max(t_v, t_a)
            if t_joint > max_time:
                max_time = t_joint

        return max_time

    def sample_trajectory(self, start_pos: List[float], target_pos: List[float],
                          progress: float) -> Tuple[List[float], List[float], List[float]]:
        """
        Sample trajectory at normalized progress s in [0, 1] using smooth S-curve / quintic profile.
        Returns: (position_rad, velocity_rad_s, acceleration_rad_s2)
        """
        s = max(0.0, min(1.0, progress))

        # Quintic polynomial interpolation for smooth s(t): s(t) = 10*s^3 - 15*s^4 + 6*s^5
        pos_s = 10.0 * (s ** 3) - 15.0 * (s ** 4) + 6.0 * (s ** 5)
        vel_ds = 30.0 * (s ** 2) - 60.0 * (s ** 3) + 30.0 * (s ** 4)
        acc_d2s = 60.0 * s - 180.0 * (s ** 2) + 120.0 * (s ** 3)

        pos = [0.0] * 3
        vel = [0.0] * 3
        acc = [0.0] * 3

        total_t = self.compute_motion_time(start_pos, target_pos)

        for i in range(3):
            delta = target_pos[i] - start_pos[i]
            pos[i] = start_pos[i] + delta * pos_s
            vel[i] = (delta / total_t) * vel_ds if total_t > 0 else 0.0
            acc[i] = (delta / (total_t ** 2)) * acc_d2s if total_t > 0 else 0.0

        return pos, vel, acc

    def sample_trajectory_at_time(self, start_pos: List[float], target_pos: List[float],
                                  elapsed_s: float) -> Tuple[List[float], List[float], List[float]]:
        """Sample a trajectory from elapsed time, suitable for a periodic publisher."""
        duration = self.compute_motion_time(start_pos, target_pos)
        if not math.isfinite(elapsed_s):
            elapsed_s = 0.0
        progress = elapsed_s / duration if duration > 0.0 else 1.0
        return self.sample_trajectory(start_pos, target_pos, progress)

    def sample_stream(self, start_pos: List[float], target_pos: List[float],
                      sample_rate_hz: float = 100.0):
        """Yield fixed-rate samples without collapsing the path to one endpoint."""
        if not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive and finite")
        duration = self.compute_motion_time(start_pos, target_pos)
        step_s = 1.0 / sample_rate_hz
        elapsed_s = 0.0
        while elapsed_s < duration:
            yield self.sample_trajectory_at_time(start_pos, target_pos, elapsed_s)
            elapsed_s += step_s
        yield self.sample_trajectory_at_time(start_pos, target_pos, duration)
