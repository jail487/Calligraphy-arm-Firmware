import os, sys, math
import rclpy
from draw_calligraphy_yong import CalligraphyYongDrawer

rclpy.init()
drawer = CalligraphyYongDrawer()

# Let's inspect raw physical_pts generated in build_yong_trajectory
with open('robot_path.json', 'r', encoding='utf-8') as f:
    import json
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

print(f"Strokes count: {len(strokes)}")
for idx, s in enumerate(strokes):
    print(f"Stroke {idx}: start=({s[0]['x']}, {s[0]['y']}, {s[0]['z']}), end=({s[-1]['x']}, {s[-1]['y']}, {s[-1]['z']}), count={len(s)}")

rclpy.shutdown()
