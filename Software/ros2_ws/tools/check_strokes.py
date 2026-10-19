import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

with open('Software/ros2_ws/robot_path.json', 'r') as f:
    data = json.load(f)

wps = data['waypoints']

fig, axs = plt.subplots(1, 3, figsize=(15, 5))

strokes = []
curr = []
for wp in wps:
    if wp['type'] == 'travel':
        if curr:
            strokes.append(curr)
            curr = []
    else:
        curr.append(wp)
if curr:
    strokes.append(curr)

print(f"Total strokes: {len(strokes)}")
for i, s in enumerate(strokes):
    print(f"Stroke {i+1}: start=({s[0]['x']}, {s[0]['y']}), end=({s[-1]['x']}, {s[-1]['y']}), pts={len(s)}")

# Plot 1: raw screen coords (0 at top)
for s in strokes:
    xs = [p['x'] for p in s]
    ys = [p['y'] for p in s]
    axs[0].plot(xs, ys, 'b-', linewidth=2)
axs[0].invert_yaxis()
axs[0].set_title('Inverted Y (0 at top)')
axs[0].grid(True)

# Plot 2: y as is (0 at bottom)
for s in strokes:
    xs = [p['x'] for p in s]
    ys = [p['y'] for p in s]
    axs[1].plot(xs, ys, 'r-', linewidth=2)
axs[1].set_title('Normal Y (0 at bottom)')
axs[1].grid(True)

# Plot 3: 180 rotated
for s in strokes:
    xs = [1024 - p['x'] for p in s]
    ys = [1024 - p['y'] for p in s]
    axs[2].plot(xs, ys, 'g-', linewidth=2)
axs[2].set_title('Rotated 180 deg')
axs[2].grid(True)

plt.savefig('Software/ros2_ws/strokes_comparison.png')
print("Saved comparison to Software/ros2_ws/strokes_comparison.png")

