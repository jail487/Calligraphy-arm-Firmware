import os, sys, math, json
from draw_calligraphy_yong import CalligraphyYongDrawer

import rclpy
rclpy.init()
d = CalligraphyYongDrawer()
# Let's inspect physical_pts in d.build_yong_trajectory
with open('robot_path.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

raw_wps = data.get('waypoints', [])
strokes = []
curr_stroke = []
for wp in raw_wps:
    if wp['type'] == 'travel':
        if curr_stroke:
            strokes.append(curr_stroke)
            curr_stroke = []
    else:
        curr_stroke.append(wp)
if curr_stroke:
    strokes.append(curr_stroke)

standby_rad = math.radians(41.1)
cos_t = math.cos(standby_rad)
sin_t = math.sin(standby_rad)
cx = 20.0 * cos_t
cy = 20.0 * sin_t
r_span = 6.0 * math.sqrt(2.0)
tilt_slope = 4.29 / r_span
travel_h_cm = -10.43 - (-12.73)

def map_coord(px, py, z_json):
    u_norm = (py / 1024.0) - 0.5
    v_norm = 0.5 - (px / 1024.0)
    u = u_norm * 5.35
    v = v_norm * 8.485
    wx = cx + u * cos_t - v * sin_t
    wy = cy + u * sin_t + v * cos_t
    r_pt = math.hypot(wx, wy)
    delta_r = r_pt - 20.0
    z_paper = -12.73 + tilt_slope * delta_r
    press_ratio = max(0.0, min(1.0, (abs(z_json) - 1.0) / 3.0))
    press_depth_cm = (1.8 + press_ratio * (3.6 - 1.8)) / 10.0
    wz = z_paper - press_depth_cm
    return wx, wy, wz, z_paper + travel_h_cm

physical_pts = []
for s_idx, stroke in enumerate(strokes):
    s_start = stroke[0]
    sx, sy, sz_write, sz_air = map_coord(s_start['x'], s_start['y'], s_start['z'])
    physical_pts.append({'type': 'travel', 'x_cm': sx, 'y_cm': sy, 'z_cm': sz_air})
    physical_pts.append({'type': 'write', 'x_cm': sx, 'y_cm': sy, 'z_cm': sz_write})
    for wp in stroke[1:]:
        wx, wy, wz_write, _ = map_coord(wp['x'], wp['y'], wp['z'])
        physical_pts.append({'type': 'write', 'x_cm': wx, 'y_cm': wy, 'z_cm': wz_write})
    s_end = stroke[-1]
    ex, ey, _, ez_air = map_coord(s_end['x'], s_end['y'], s_end['z'])
    physical_pts.append({'type': 'travel', 'x_cm': ex, 'y_cm': ey, 'z_cm': ez_air})

print(f"Total physical_pts: {len(physical_pts)}")
for i, pt in enumerate(physical_pts):
    print(f"[{i:02d}] type={pt['type']:6s} x={pt['x_cm']:.2f} y={pt['y_cm']:.2f} z={pt['z_cm']:.2f}")

# Now see how trajectory_planner segments it:
segments = []
curr_seg = [physical_pts[0]]
for pt in physical_pts[1:]:
    if pt['type'] == curr_seg[-1]['type']:
        curr_seg.append(pt)
    else:
        curr_seg.append(pt)
        segments.append(curr_seg)
        curr_seg = [pt]
if curr_seg:
    segments.append(curr_seg)

print("\n--- SEGMENTS ---")
for i, s in enumerate(segments):
    print(f"Segment {i}: count={len(s)} type={s[0]['type']} -> start=({s[0]['x_cm']:.2f},{s[0]['y_cm']:.2f},{s[0]['z_cm']:.2f}) end=({s[-1]['x_cm']:.2f},{s[-1]['y_cm']:.2f},{s[-1]['z_cm']:.2f})")
