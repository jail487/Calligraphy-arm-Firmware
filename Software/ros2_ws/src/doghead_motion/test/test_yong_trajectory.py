import sys, os, json, math
sys.path.insert(0, 'Software/ros2_ws')
sys.path.insert(0, 'Software/ros2_ws/src/doghead_motion/doghead_motion')
from kinematics import DogheadKinematics
from trajectory_planner import TrajectoryPlanner
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

kin = DogheadKinematics()

with open('Software/ros2_ws/robot_path.json', 'r') as f:
    data = json.load(f)

wps = data['waypoints']

r_center_cm = 20.0
standby_rad = math.radians(45.0)
cos_t = math.cos(standby_rad)
sin_t = math.sin(standby_rad)
cx = r_center_cm * cos_t
cy = r_center_cm * sin_t

size_r = 5.35
size_t = 8.485
z_touch_cm = -12.73
travel_h_cm = 2.30
tilt_diff_cm = 4.29
r_span = 6.0 * math.sqrt(2.0)
tilt_slope = tilt_diff_cm / r_span

physical_pts = []
unreachable = 0

for i, wp in enumerate(wps):
    px = float(wp['x'])
    py = float(wp['y'])
    wp_type = wp.get('type', 'travel')
    z_json = float(wp.get('z', 10.0))
    
    # In robot_path.json: (0,0) is bottom-left, (1024,1024) is top-right
    # u is radial (along the arm, near to far)
    # v is tangential (across the arm, left to right)
    u_norm = (py / 1024.0) - 0.5
    v_norm = (px / 1024.0) - 0.5
    
    u = u_norm * size_r
    v = v_norm * size_t
    
    wx = cx + u * cos_t - v * sin_t
    wy = cy + u * sin_t + v * cos_t
    
    r_pt = math.hypot(wx, wy)
    delta_r = r_pt - r_center_cm
    z_paper = z_touch_cm + tilt_slope * delta_r
    
    if wp_type == 'travel' or z_json > 0.0:
        wz = z_paper + travel_h_cm
    else:
        # z_json in [-1.0, -4.0] -> 1.0mm to 2.5mm press
        press_depth = 0.10 + (abs(z_json) - 1.0) / 3.0 * 0.15
        wz = z_paper - press_depth
        
    ok, j, msg = kin.inverse_kinematics(wx/100.0, wy/100.0, wz/100.0)
    if not ok:
        unreachable += 1
        print(f"Unreachable #{i}: ({wx:.2f}, {wy:.2f}, {wz:.2f}) -> {msg}")
    physical_pts.append({'type': wp_type, 'x_cm': wx, 'y_cm': wy, 'z_cm': wz, 'joints': j if ok else None})

print(f"Total waypoints: {len(physical_pts)}, Unreachable: {unreachable}")

# Test full PVA planning
planner = TrajectoryPlanner(dt=0.02)
pva = planner.plan_full_trajectory(physical_pts, kin_solver=kin, v_write=2.0, v_travel=3.5, a_write=8.0, a_travel=10.0)
print(f"PVA points: {len(pva)}, total duration: {pva[-1]['t']:.2f} s")

# Joint range summary
q1_deg = [math.degrees(p['q1']) for p in pva]
q2_deg = [math.degrees(p['q2']) for p in pva]
q3_deg = [math.degrees(p['q3']) for p in pva]
z_cm = [p['z'] for p in pva]

print(f"J1 range: [{min(q1_deg):.1f}°, {max(q1_deg):.1f}°] (limit 0°..111°)")
print(f"J2 range: [{min(q2_deg):.1f}°, {max(q2_deg):.1f}°] (limit 105°..180°)")
print(f"J3 range: [{min(q3_deg):.1f}°, {max(q3_deg):.1f}°] (limit 0°..90°)")
print(f"Z  range: [{min(z_cm):.2f}, {max(z_cm):.2f}] cm")

# Plot 3D trajectory
fig = plt.figure(figsize=(12, 5))
ax1 = fig.add_subplot(1, 2, 1)
# Plot write segments in red, travel in dashed gray
for i in range(1, len(pva)):
    p1 = pva[i-1]
    p2 = pva[i]
    if p2.get('seg_type') == 'write':
        ax1.plot([p1['x'], p2['x']], [p1['y'], p2['y']], 'r-', linewidth=2)
    else:
        ax1.plot([p1['x'], p2['x']], [p1['y'], p2['y']], 'gray', linestyle=':', alpha=0.5)
ax1.set_title("2D Cartesian Path of '永' (X-Y cm)")
ax1.set_xlabel("X (cm)")
ax1.set_ylabel("Y (cm)")
ax1.grid(True)
ax1.axis('equal')

ax2 = fig.add_subplot(1, 2, 2)
ax2.plot([p['t'] for p in pva], z_cm, 'b-', label='Z height')
ax2.set_title("Z Height vs Time (Pen Lift / Press)")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Z (cm)")
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.savefig('Software/ros2_ws/yong_trajectory_preview.png')
print("Saved preview to Software/ros2_ws/yong_trajectory_preview.png")

