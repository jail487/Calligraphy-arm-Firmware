import math
from typing import Tuple, List, Optional, Dict

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))
try:
    from robot_constants import (
        LINK_R1_M, LINK_R2_M, LINK_R3_M, TOOL_OFFSET_FWD_M,
        JOINT1_MIN_DEG, JOINT1_MAX_DEG,
        JOINT2_MIN_DEG, JOINT2_MAX_DEG,
        JOINT3_MIN_DEG, JOINT3_MAX_DEG
    )
except ImportError:
    LINK_R1_M, LINK_R2_M, LINK_R3_M, TOOL_OFFSET_FWD_M = 0.1400, 0.1400, 0.0880, 0.0453
    JOINT1_MIN_DEG, JOINT1_MAX_DEG = 0.0, 111.0
    JOINT2_MIN_DEG, JOINT2_MAX_DEG = 110.0, 180.0
    JOINT3_MIN_DEG, JOINT3_MAX_DEG = -90.0, 90.0

DEFAULT_LINK_LENGTHS = [LINK_R1_M, LINK_R2_M, LINK_R3_M, TOOL_OFFSET_FWD_M]

DEFAULT_JOINT_LIMITS = {
    "q1_min": math.radians(JOINT1_MIN_DEG), "q1_max": math.radians(JOINT1_MAX_DEG),
    "q2_min": math.radians(JOINT2_MIN_DEG), "q2_max": math.radians(JOINT2_MAX_DEG),
    "q3_min": math.radians(JOINT3_MIN_DEG), "q3_max": math.radians(JOINT3_MAX_DEG),
}


class DogheadKinematics:
    """
    Kinematics solver for 2-DOF 3-Parallelogram Parallel Mechanism (PPM) + Base Rotation:
    - R1: Shoulder Link (大臂長度)
    - R2: Forearm Link (小臂長度)
    - R3: End-Effector vertical offset (筆尖垂直下垂偏移，例如 88mm)
    - R_tool_fwd: End-Effector forward horizontal offset (筆尖水平向前延伸偏移，例如 33.5mm)
    - Joint 1 (theta1): Shoulder angle (大臂角度)
    - Joint 2 (theta2): Forearm crank angle (小臂拉桿角度)
    - Joint 3 (theta3): Base rotation angle (基座旋轉角)

    Forward Kinematics:
      r_plane = (R1 * cos(theta1) - R2 * cos(theta2)) + R_tool_fwd
      X = r_plane * cos(theta3)
      Y = r_plane * sin(theta3)
      Z = R1 * sin(theta1) - R2 * sin(theta2) - R3
    """

    def __init__(self, link_lengths: Optional[List[float]] = None,
                 limits: Optional[Dict[str, float]] = None):
        lengths = link_lengths or DEFAULT_LINK_LENGTHS
        self.R1 = lengths[0]
        self.R2 = lengths[1]
        self.R3 = lengths[2] if len(lengths) > 2 else 0.0
        self.R_tool_fwd = lengths[3] if len(lengths) > 3 else 0.0
        
        # 兼容 L1, L2, L3 別名
        self.L1 = self.R1
        self.L2 = self.R2
        self.L3 = self.R3

        self.limits = limits or DEFAULT_JOINT_LIMITS
        self.max_reach = self.R1 + self.R2 + self.R_tool_fwd
        self.min_reach = max(0.0, abs(self.R1 - self.R2) - self.R_tool_fwd)

    def forward_kinematics(self, q1: float, q2: float, q3: float) -> Tuple[bool, Tuple[float, float, float], str]:
        """
        Calculate 3D End-Effector position (x, y, z) in meters from joint angles (rad).
        q1: theta1 (Shoulder), q2: theta2 (Forearm Crank), q3: theta3 (Base).
        """
        if not (math.isfinite(q1) and math.isfinite(q2) and math.isfinite(q3)):
            return False, (0.0, 0.0, 0.0), "Non-finite joint angle input"

        r_plane = (self.R1 * math.cos(q1) - self.R2 * math.cos(q2)) + self.R_tool_fwd
        x = r_plane * math.cos(q3)
        y = r_plane * math.sin(q3)
        z = self.R1 * math.sin(q1) - self.R2 * math.sin(q2) - self.R3

        return True, (x, y, z), "OK"

    def inverse_kinematics(self, x: float, y: float, z: float, elbow_up: Optional[bool] = True) -> Tuple[bool, Tuple[float, float, float], str]:
        """
        Calculate joint angles (q1, q2, q3) in radians for target position (x, y, z) in meters.
        Analytical Closed-form Inverse Kinematics for 3-Parallelogram Mechanism with Tool Offset.
        """
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            return False, (0.0, 0.0, 0.0), "Unreachable: Non-finite target coordinates (NaN/Inf)"

        # 1. Base rotation angle (Joint 3)
        q3 = math.atan2(y, x)

        # 2. Planar distance & effective height (扣除末端向前延伸 tool offset)
        r = math.hypot(x, y)
        r_arm = r - self.R_tool_fwd
        z_eff = z + self.R3
        D_sq = r_arm * r_arm + z_eff * z_eff
        D = math.sqrt(D_sq)

        # 3. Workspace reachability check for the parallelogram link mechanism
        arm_max = self.R1 + self.R2
        arm_min = abs(self.R1 - self.R2)
        if D > arm_max or D < arm_min:
            return False, (0.0, 0.0, 0.0), (
                f"Unreachable: Target link distance {D:.4f} m outside mechanism boundary "
                f"[{arm_min:.4f}, {arm_max:.4f}] m"
            )

        # 4. Solve Shoulder Angle q1 (theta1)
        cos_psi1 = (D_sq + self.R1**2 - self.R2**2) / (2.0 * self.R1 * D)
        cos_psi1 = max(-1.0, min(1.0, cos_psi1))
        psi1 = math.acos(cos_psi1)
        phi = math.atan2(z_eff, r_arm)

        # Elbow-up configuration selects positive branch
        q1 = phi + psi1

        # 5. Solve Forearm Crank Angle q2 (theta2) uniquely from vector closure
        dx2 = self.R1 * math.cos(q1) - r_arm
        dy2 = self.R1 * math.sin(q1) - z_eff
        q2 = math.atan2(dy2, dx2)
        if q2 < 0.0:
            q2 += 2.0 * math.pi

        # 6. Joint soft limits check
        tolerance = 0.035 # ~2.0 deg soft numerical margin
        if not (self.limits["q1_min"] - tolerance <= q1 <= self.limits["q1_max"] + tolerance):
            return False, (0.0, 0.0, 0.0), f"Unreachable: Joint 1 (Shoulder) angle {math.degrees(q1):.1f}° exceeds [{math.degrees(self.limits['q1_min']):.1f}°, {math.degrees(self.limits['q1_max']):.1f}°]"
        if not (self.limits["q2_min"] - tolerance <= q2 <= self.limits["q2_max"] + tolerance):
            return False, (0.0, 0.0, 0.0), f"Unreachable: Joint 2 (Elbow) angle {math.degrees(q2):.1f}° exceeds [{math.degrees(self.limits['q2_min']):.1f}°, {math.degrees(self.limits['q2_max']):.1f}°]"
        if not (self.limits["q3_min"] - tolerance <= q3 <= self.limits["q3_max"] + tolerance):
            return False, (0.0, 0.0, 0.0), f"Unreachable: Joint 3 (Base) angle {math.degrees(q3):.1f}° exceeds [{math.degrees(self.limits['q3_min']):.1f}°, {math.degrees(self.limits['q3_max']):.1f}°]"

        return True, (q1, q2, q3), "OK"
