import os, sys, math
import rclpy
from draw_calligraphy_yong import CalligraphyYongDrawer

rclpy.init()
drawer = CalligraphyYongDrawer()
pts = drawer.build_yong_trajectory('robot_path.json')

# Find transitions from write to travel
transitions = []
for i in range(1, len(pts)):
    if pts[i-1]['seg_type'] == 'write' and pts[i]['seg_type'] == 'travel':
        transitions.append(i)

print(f"Write to travel transitions at indices: {transitions}")
for tr in transitions:
    print(f"\n--- Transition around index {tr} ---")
    for j in range(max(0, tr - 5), min(len(pts), tr + 15)):
        p = pts[j]
        print(f"[{j:04d}] t={p['t']:.2f} x={p['x']:.2f} y={p['y']:.2f} z={p['z']:.2f} seg={p['seg_type']}")

rclpy.shutdown()
