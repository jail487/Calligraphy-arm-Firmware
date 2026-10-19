import math
import pytest
from doghead_motion.kinematics import DogheadKinematics


def test_forward_and_inverse_kinematics_roundtrip():
    kin = DogheadKinematics(limits={
        "q1_min": 0.0, "q1_max": math.pi,
        "q2_min": 0.0, "q2_max": math.pi,
        "q3_min": -math.pi, "q3_max": math.pi,
    })

    # Test joint angles (Shoulder q1, Forearm Crank q2, Base q3) with R1=R2=0.1139, R3=0.0
    test_joints = [
        (math.radians(60.0), math.radians(150.0), 0.0),
        (math.radians(45.0), math.radians(135.0), math.radians(15.0)),
        (math.radians(90.0), math.radians(170.0), math.radians(-20.0)),
    ]

    for q1_in, q2_in, q3_in in test_joints:
        ok_fk, (x, y, z), msg_fk = kin.forward_kinematics(q1_in, q2_in, q3_in)
        assert ok_fk is True, f"FK failed: {msg_fk}"

        ok_ik, (q1_out, q2_out, q3_out), msg_ik = kin.inverse_kinematics(x, y, z)
        assert ok_ik is True, f"IK failed: {msg_ik}"

        assert math.isclose(q1_in, q1_out, abs_tol=1e-3)
        assert math.isclose(q2_in, q2_out, abs_tol=1e-3)
        assert math.isclose(q3_in, q3_out, abs_tol=1e-3)

    # With R3=0.0, Z = R1*sin(q1) - R2*sin(q2)
    test_positions = [
        (0.15, 0.0, 0.04),
        (0.12, 0.03, 0.01),
        (0.16, -0.02, 0.06),
    ]

    for x_in, y_in, z_in in test_positions:
        ok_ik, (q1, q2, q3), msg_ik = kin.inverse_kinematics(x_in, y_in, z_in)
        assert ok_ik is True, f"IK failed for target ({x_in}, {y_in}, {z_in}): {msg_ik}"

        ok_fk, (x_out, y_out, z_out), msg_fk = kin.forward_kinematics(q1, q2, q3)
        assert ok_fk is True, f"FK failed: {msg_fk}"

        assert math.isclose(x_in, x_out, abs_tol=1e-3)
        assert math.isclose(y_in, y_out, abs_tol=1e-3)
        assert math.isclose(z_in, z_out, abs_tol=1e-3)


def test_unreachable_target():
    kin = DogheadKinematics()
    # Beyond max reach R1 + R2 = 0.2278m
    ok_ik, (q1, q2, q3), msg = kin.inverse_kinematics(0.25, 0.0, 0.0)
    assert ok_ik is False
    assert "Unreachable" in msg


def test_non_finite_inputs():
    kin = DogheadKinematics()
    assert kin.forward_kinematics(float('nan'), 0.0, 0.0)[0] is False
    assert kin.inverse_kinematics(0.15, float('inf'), 0.0)[0] is False
