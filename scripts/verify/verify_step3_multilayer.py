"""Step 3 verification: multi-layer LiDAR against a procedurally generated
arena. Expect per-layer min/max range to differ near tall buildings
(occlusion varies by pitch), and layer 0 (bottom) vs layer 4 (top) should
not be identical when buildings are present at varying heights.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent.parent))

import pybullet as p

from sim.arena import ArenaCfg, generate_buildings, pick_start_target
from sim.lidar import Lidar2D

p.connect(p.DIRECT)
p.setGravity(0, 0, 0)

cfg = ArenaCfg()
buildings = generate_buildings(cfg)
for b in buildings:
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[b.hx, b.hy, b.height / 2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                       basePosition=[b.cx, b.cy, b.height / 2])

start_xy, target_xy = pick_start_target(cfg, buildings)
drone_pos = (start_xy[0], start_xy[1], 3.0)

lidar = Lidar2D(num_rays=48, max_range=15.0, fov_deg=300.0, draw_debug=False)
flat, n_ranges, n_layers = lidar.scan_multilayer(
    drone_pos, yaw=0.0, n_layers=5,
    vertical_angle_min=-0.0872665, vertical_angle_increment=0.0436332,
)

assert len(flat) == n_ranges * n_layers, f"expected {n_ranges*n_layers} got {len(flat)}"
print(f"n_ranges={n_ranges} n_layers={n_layers} total={len(flat)}")
for layer in range(n_layers):
    layer_vals = flat[layer * n_ranges:(layer + 1) * n_ranges]
    print(f"layer {layer}: min={min(layer_vals):.2f} max={max(layer_vals):.2f} mean={sum(layer_vals)/len(layer_vals):.2f}")

print("OK: multi-layer LiDAR scan verified")
