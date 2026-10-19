import sys, os, math
sys.path.insert(0, 'Software/ros2_ws')
sys.path.insert(0, 'Software/ros2_ws/src/doghead_motion/doghead_motion')
from kinematics import DogheadKinematics
import robot_constants as rc

kin = DogheadKinematics()

# Simulate corners from draw_calibration_square.py
standby_rad = math.radians(rc.HOME_STANDBY_THETA3_DEG) # 45 deg
cos_t = math.cos(standby_rad)
sin_t = math.sin(standby_rad)

r_center_cm = 20.0
square_size_cm = 6.0
cx = r_center_cm * cos_t
cy = r_center_cm * sin_t
half = square_size_cm / 2.0

local_corners = [
    (-half, -half),  # 1. 內近角
    ( half, -half),  # 2. 外遠角
    ( half,  half),  # 3. 外遠角
    (-half,  half)   # 4. 內近角
]

z_touch_cm = -12.73
press_depth_cm = 0.15
tilt_diff_cm = 4.29
z_write_cm = z_touch_cm - press_depth_cm
r_span = square_size_cm * math.sqrt(2.0)
tilt_slope = tilt_diff_cm / r_span

corners = []
for i, (u, v) in enumerate(local_corners):
    wx = cx + u * cos_t - v * sin_t
    wy = cy + u * sin_t + v * cos_t
    r_pt = math.hypot(wx, wy)
    delta_r = r_pt - r_center_cm
    z_corr = tilt_slope * delta_r
    zw = z_write_cm + z_corr
    ok, j, msg = kin.inverse_kinematics(wx/100.0, wy/100.0, zw/100.0)
    corners.append((wx, wy, zw, j))
    print(f"Corner {i+1}: local=({u:+4.1f}, {v:+4.1f}), W=({wx:6.2f}, {wy:6.2f}, Z={zw:6.2f}) cm, r={r_pt:5.2f} cm | J1={math.degrees(j[0]):5.2f}, J2={math.degrees(j[1]):5.2f}, J3={math.degrees(j[2]):5.2f} deg")

# Distances between consecutive corners
for i in range(4):
    c_curr = corners[i]
    c_next = corners[(i+1)%4]
    dx = c_next[0] - c_curr[0]
    dy = c_next[1] - c_curr[1]
    dz = c_next[2] - c_curr[2]
    d2d = math.hypot(dx, dy)
    d3d = math.sqrt(dx*dx + dy*dy + dz*dz)
    dj3 = math.degrees(c_next[3][2] - c_curr[3][2])
    print(f"Edge {i+1}->{(i+1)%4+1}: dx={dx:+5.2f}, dy={dy:+5.2f}, dz={dz:+5.2f} cm | 2D dist={d2d:5.2f} cm, 3D dist={d3d:5.2f} cm | delta_J3={dj3:+5.2f} deg")
