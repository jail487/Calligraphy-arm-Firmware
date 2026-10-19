import json
import math
import pytest
from doghead_motion.json_trajectory_loader import (
    JsonTrajectoryLoader,
    CoordinateTransformConfig,
    TrajectoryPoint,
)

SAMPLE_USER_JSON = """[
  {
    "type": "travel",
    "x": 454.14,
    "y": 97.76,
    "z": 10.0
  },
  {
    "type": "write",
    "x": 454.14,
    "y": 97.76,
    "z": -1.0
  },
  {
    "type": "write",
    "x": 461.39,
    "y": 102.42,
    "z": -1.62
  }
]"""


def test_parse_user_sample_json():
    loader = JsonTrajectoryLoader(CoordinateTransformConfig(unit="mm"))
    points = loader.load_from_json_string(SAMPLE_USER_JSON)

    assert len(points) == 3

    # Point 0: travel
    p0 = points[0]
    assert p0.point_type == "travel"
    assert p0.is_travel is True
    assert p0.is_writing is False
    assert math.isclose(p0.x, 0.45414, abs_tol=1e-5)
    assert math.isclose(p0.y, 0.09776, abs_tol=1e-5)
    assert math.isclose(p0.z, 0.01000, abs_tol=1e-5)
    # PVA preserved defaults
    assert p0.vx == 0.0
    assert p0.vy == 0.0
    assert p0.vz == 0.0
    assert p0.ax == 0.0
    assert p0.ay == 0.0
    assert p0.az == 0.0

    # Point 1: write
    p1 = points[1]
    assert p1.point_type == "write"
    assert p1.is_writing is True
    assert math.isclose(p1.z, -0.00100, abs_tol=1e-5)

    # Point 2: write
    p2 = points[2]
    assert math.isclose(p2.x, 0.46139, abs_tol=1e-5)
    assert math.isclose(p2.y, 0.10242, abs_tol=1e-5)
    assert math.isclose(p2.z, -0.00162, abs_tol=1e-5)


def test_coordinate_scaling_and_offset():
    cfg = CoordinateTransformConfig(
        unit="mm",
        scale_x=0.5,
        scale_y=0.5,
        scale_z=1.0,
        offset_x=0.05,
        offset_y=-0.02,
        offset_z=0.01,
    )
    loader = JsonTrajectoryLoader(cfg)
    raw = [{"type": "write", "x": 200.0, "y": 100.0, "z": 5.0}]
    pts = loader.load_from_dict_list(raw)

    assert len(pts) == 1
    # x = 200 * 0.001 * 0.5 + 0.05 = 0.10 + 0.05 = 0.15
    assert math.isclose(pts[0].x, 0.15, abs_tol=1e-5)
    # y = 100 * 0.001 * 0.5 - 0.02 = 0.05 - 0.02 = 0.03
    assert math.isclose(pts[0].y, 0.03, abs_tol=1e-5)
    # z = 5 * 0.001 * 1.0 + 0.01 = 0.005 + 0.01 = 0.015
    assert math.isclose(pts[0].z, 0.015, abs_tol=1e-5)


def test_pva_velocity_and_acceleration_preservation():
    loader = JsonTrajectoryLoader(CoordinateTransformConfig(unit="m"))
    raw = [{
        "type": "write",
        "x": 0.15, "y": 0.05, "z": -0.002,
        "vx": 0.02, "vy": 0.01, "vz": 0.0,
        "ax": 0.1, "ay": 0.05, "az": 0.0
    }]
    pts = loader.load_from_dict_list(raw)

    assert len(pts) == 1
    assert math.isclose(pts[0].x, 0.15, abs_tol=1e-5)
    assert math.isclose(pts[0].vx, 0.02, abs_tol=1e-5)
    assert math.isclose(pts[0].vy, 0.01, abs_tol=1e-5)
    assert math.isclose(pts[0].ax, 0.1, abs_tol=1e-5)
    assert math.isclose(pts[0].ay, 0.05, abs_tol=1e-5)


def test_invalid_json_handling():
    loader = JsonTrajectoryLoader()

    # Missing coordinate fields
    with pytest.raises(ValueError, match="Missing required coordinate fields"):
        loader.load_from_dict_list([{"type": "travel", "x": 100.0, "y": 50.0}])

    # Non-finite coordinates (NaN)
    with pytest.raises(ValueError, match="Non-finite position coordinates"):
        loader.load_from_dict_list([{"type": "write", "x": float('nan'), "y": 50.0, "z": 0.0}])

    # Non-finite coordinates (Inf)
    with pytest.raises(ValueError, match="Non-finite position coordinates"):
        loader.load_from_dict_list([{"type": "write", "x": 100.0, "y": float('inf'), "z": 0.0}])

    # Malformed JSON string
    with pytest.raises(ValueError, match="Invalid JSON string format"):
        loader.load_from_json_string("not a json string")

    # Non-existent file
    with pytest.raises(ValueError, match="Failed to open trajectory file"):
        loader.load_from_file("non_existent_file_12345.json")

