#!/usr/bin/env python3
"""PyBullet world/asset construction: arena ground/walls/buildings, cosmetic
city ground detail, and drone/target/marker visuals. Pure presentation --
never touches simulation state or navigation decisions.
"""
import math
import os
import random
import shutil
import subprocess
from typing import Dict

import pybullet as p
import pybullet_data
from PIL import Image, ImageDraw

DRONE_VIS_RGBA = [0.1, 0.4, 1.0, 1.0]
TARGET_RADIUS = 0.6


def _prefer_nvidia_gpu() -> None:
    """On a hybrid AMD/NVIDIA (PRIME) laptop, PyBullet's GUI would otherwise
    render through the slow Mesa/integrated path. Sets PRIME render-offload
    env vars (read lazily by p.connect) so it uses the NVIDIA GPU instead.
    No-op if there's no NVIDIA driver or the vars are already set."""
    if "__GLX_VENDOR_LIBRARY_NAME" in os.environ:
        return
    if not shutil.which("nvidia-smi"):
        return
    try:
        subprocess.run(["nvidia-smi", "-L"], capture_output=True, timeout=2, check=True)
    except Exception:
        return
    os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"


WALL_RGBA = [0.15, 0.55, 0.9, 0.08]  # faint translucent tint, matching airport_world.sdf's bounds_visible

# Muted urban palette for building color variety -- kept in a narrow,
# coherent range (grays/tans/blue-grays) rather than pure random RGB, so
# the skyline reads as a city, not a rainbow.
BUILDING_PALETTE = [
    [0.55, 0.55, 0.60, 1.0],
    [0.62, 0.58, 0.52, 1.0],
    [0.48, 0.50, 0.55, 1.0],
    [0.58, 0.54, 0.46, 1.0],
    [0.45, 0.48, 0.52, 1.0],
    [0.60, 0.60, 0.58, 1.0],
]

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_WINDOW_TEX_PATH = os.path.join(_PROJECT_ROOT, "models", "generated", "window_tex.png")
_GROUND_TEX_PATH = os.path.join(_PROJECT_ROOT, "models", "generated", "ground_tex.png")


def _ensure_window_texture() -> str:
    """Procedurally generate (once, cached to disk) a small tileable
    window-grid texture so buildings read as buildings instead of plain
    flat boxes. A base wall tone with a grid of slightly lit/dark squares
    for windows."""
    if not os.path.exists(_WINDOW_TEX_PATH):
        os.makedirs(os.path.dirname(_WINDOW_TEX_PATH), exist_ok=True)
        size = 128
        img = Image.new("RGB", (size, size), (235, 235, 232))
        draw = ImageDraw.Draw(img)
        rng = random.Random(7)
        cell = 16
        margin = 3
        for gy in range(0, size, cell):
            for gx in range(0, size, cell):
                lit = rng.random() < 0.35
                color = (255, 244, 190) if lit else (70, 78, 92)
                draw.rectangle(
                    [gx + margin, gy + margin, gx + cell - margin, gy + cell - margin],
                    fill=color,
                )
        img.save(_WINDOW_TEX_PATH)
    return _WINDOW_TEX_PATH


def _ensure_ground_texture() -> str:
    """Procedurally generate (once, cached to disk) a subtle speckled
    asphalt/concrete-look ground texture, replacing PyBullet's default
    blue/white checkerboard."""
    if not os.path.exists(_GROUND_TEX_PATH):
        os.makedirs(os.path.dirname(_GROUND_TEX_PATH), exist_ok=True)
        size = 256
        base = (112, 110, 106)
        img = Image.new("RGB", (size, size), base)
        rng = random.Random(11)
        px = img.load()
        for _ in range(size * size // 6):
            x, y = rng.randrange(size), rng.randrange(size)
            jitter = rng.randint(-14, 14)
            r, g, b = base
            px[x, y] = (max(0, min(255, r + jitter)), max(0, min(255, g + jitter)), max(0, min(255, b + jitter)))
        img.save(_GROUND_TEX_PATH)
    return _GROUND_TEX_PATH


def build_arena_bodies(buildings, arena_cfg):
    """Ground + solid boundary walls (matching CANavigator's
    worlds/airport_world.sdf 'bounds_visible' model) plus the generated
    NFZ buildings.

    Buildings need a real collision shape: PyBullet's rayTestBatch (unlike
    Gazebo's GPU LiDAR) only intersects collision geometry, so without one
    this port's LiDAR would be blind to them. pass_through=True instead
    means no rigid-body contact force -- a nav miss is a soft
    zone_violation, not a bounce -- so that's reproduced separately via a
    per-pair collision filter (see experiment.orchestrator.run_attempt())
    that disables contact resolution but keeps collision geometry intact
    for rayTestBatch.
    """
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    ground_id = p.loadURDF("plane.urdf")
    ground_tex = p.loadTexture(_ensure_ground_texture())
    p.changeVisualShape(ground_id, -1, textureUniqueId=ground_tex)

    window_tex = p.loadTexture(_ensure_window_texture())
    palette_rng = random.Random(arena_cfg.seed)
    building_ids = []
    for b in buildings:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[b.hx, b.hy, b.height / 2])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[b.hx, b.hy, b.height / 2],
                                   rgbaColor=palette_rng.choice(BUILDING_PALETTE))
        bid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                 basePosition=[b.cx, b.cy, b.height / 2])
        p.changeVisualShape(bid, -1, textureUniqueId=window_tex)
        building_ids.append(bid)

    thickness, height, z_pose = 1.0, 50.0, 5.0
    x_min, x_max, y_min, y_max = arena_cfg.x_min, arena_cfg.x_max, arena_cfg.y_min, arena_cfg.y_max
    x_span, y_span = x_max - x_min, y_max - y_min
    walls = [
        ([x_span / 2, thickness / 2, height / 2], [0, y_max, z_pose]),   # north
        ([x_span / 2, thickness / 2, height / 2], [0, y_min, z_pose]),   # south
        ([thickness / 2, y_span / 2, height / 2], [x_max, 0, z_pose]),   # east
        ([thickness / 2, y_span / 2, height / 2], [x_min, 0, z_pose]),   # west
        ([x_span / 2, y_span / 2, thickness / 2], [0, 0, 30.0]),         # ceiling
    ]
    for half_extents, pos in walls:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=WALL_RGBA)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                           basePosition=pos)

    return building_ids


ROAD_RGBA = [0.12, 0.12, 0.13, 1.0]
PAVEMENT_RGBA = [0.72, 0.70, 0.65, 1.0]
GRASS_RGBA = [0.22, 0.50, 0.18, 1.0]


def render_city_ground_detail(layout) -> None:
    """Flat, no-collision ground decals for the road grid, developed-block
    pavement, and grass on undeveloped lots (sim.arena.CityLayout). Purely
    cosmetic (no collisionShapeIndex, can't affect LiDAR). Stacked at
    increasing z (road < pavement < grass) to avoid z-fighting.
    """
    def _flat_rect(cx, cy, w, h, z, rgba):
        half = [max(w / 2.0, 0.05), max(h / 2.0, 0.05), 0.01]
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=[cx, cy, z])

    for (cx, cy, w, h) in layout.road_rects:
        _flat_rect(cx, cy, w, h, 0.01, ROAD_RGBA)
    for (cx, cy, w, h) in layout.pavement_rects:
        _flat_rect(cx, cy, w, h, 0.02, PAVEMENT_RGBA)
    for (cx, cy, w, h) in layout.grass_rects:
        rng = random.Random(int(cx * 1000) + int(cy * 1000))
        jitter = rng.uniform(-0.05, 0.05)
        rgba = [max(0.0, min(1.0, c + jitter)) for c in GRASS_RGBA[:3]] + [1.0]
        _flat_rect(cx, cy, w, h, 0.03, rgba)


NOSE_RGBA = [1.0, 0.15, 0.1, 1.0]


def make_drone(start_xy, flight_z, radius):
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    # Compound visual (sphere + forward-offset "nose" box) makes heading
    # visible at a glance. Nose is visual-only, baked into the same
    # multibody frame so it tracks the body with no manual pose sync.
    vis = p.createVisualShapeArray(
        shapeTypes=[p.GEOM_SPHERE, p.GEOM_BOX],
        radii=[radius, 0.0],
        halfExtents=[[0, 0, 0], [radius * 0.4, radius * 0.22, radius * 0.22]],
        visualFramePositions=[[0, 0, 0], [radius * 0.85, 0, 0]],
        rgbaColors=[DRONE_VIS_RGBA, NOSE_RGBA],
    )
    body = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                              basePosition=[start_xy[0], start_xy[1], flight_z])
    p.changeDynamics(body, -1, linearDamping=0.0, angularDamping=0.0)
    return body


BEACON_RGB = [1.0, 0.1, 0.7]
START_MARKER_RGB = [0.15, 1.0, 0.3]


def make_target(target_xy, flight_z, gui: bool = False):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=TARGET_RADIUS, rgbaColor=[1.0, 0.85, 0.0, 1.0])
    body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                              basePosition=[target_xy[0], target_xy[1], flight_z])
    if gui:
        # The 0.6m target sphere is easy to lose track of across a
        # 200x100m arena -- a tall beacon line is visible from anywhere.
        p.addUserDebugLine([target_xy[0], target_xy[1], flight_z], [target_xy[0], target_xy[1], 40.0],
                            lineColorRGB=BEACON_RGB, lineWidth=3.0, lifeTime=0)
    return body


def make_start_marker(start_xy, flight_z):
    """Ring + pole at the drone's fixed spawn point. Drawn once per attempt
    since the drone teleports back to the same point each strategy."""
    n = 24
    cx, cy = start_xy
    ring = [(cx + 1.5 * math.cos(2 * math.pi * i / n), cy + 1.5 * math.sin(2 * math.pi * i / n), 0.05)
            for i in range(n + 1)]
    for a, b in zip(ring, ring[1:]):
        p.addUserDebugLine(a, b, lineColorRGB=START_MARKER_RGB, lineWidth=3.0, lifeTime=0)
    p.addUserDebugLine([cx, cy, 0.0], [cx, cy, flight_z + 2.0],
                        lineColorRGB=START_MARKER_RGB, lineWidth=2.0, lifeTime=0)


# Event markers are visual-only timing/kind cues, NOT literal obstacles:
# events carry no world position (EventEmitter._emit() only ever produces
# {kind, t_emit, deadline_s, meta}, sim/event_source.py) -- nav/algorithm.py
# reacts via a timing-pressure race, not a spatial one. Markers spawn at a
# plausible stand-in distance ahead of the drone, just to make an event's
# kind/timing visible alongside the drone's reaction.
EVENT_MARKER_SPECS = {
    # kind -> (shapeType, shape_kwargs, rgb)
    "ENEMY": (p.GEOM_SPHERE, {"radius": 0.9}, [0.95, 0.1, 0.1]),
    "SUDDEN_OBSTACLE": (p.GEOM_CYLINDER, {"radius": 0.7, "length": 2.2}, [1.0, 0.55, 0.0]),
    "LANE_BLOCK": (p.GEOM_BOX, {"halfExtents": [3.5, 0.3, 1.0]}, [0.95, 0.9, 0.1]),
}
EVENT_MARKER_AHEAD_M = 7.0       # plausible stand-in distance ahead of the drone
EVENT_MARKER_BEARING_JITTER_DEG = 25.0
EVENT_MARKER_VISIBLE_S = 1.5     # auto-hide if nothing supersedes it


def make_event_markers() -> Dict[str, int]:
    """Pre-create one body per event kind, once per attempt, transparent
    (alpha 0) and with no collision shape. update_event_marker only ever
    touches alpha/position afterward, avoiding per-event
    createMultiBody/removeBody churn."""
    markers: Dict[str, int] = {}
    for kind, (shape_type, kwargs, rgb) in EVENT_MARKER_SPECS.items():
        vis = p.createVisualShape(shape_type, rgbaColor=rgb + [0.0], **kwargs)
        markers[kind] = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=[0, 0, -50])
    return markers


def hide_event_markers(markers: Dict[str, int]) -> None:
    for kind, body in markers.items():
        rgb = EVENT_MARKER_SPECS[kind][2]
        p.changeVisualShape(body, -1, rgbaColor=rgb + [0.0])


def update_event_marker(markers: Dict[str, int], emitter, last_seen_seq: int,
                         drone_pos, drone_yaw: float, sim_now: float, hide_at: list) -> int:
    """Call every loop iteration, unthrottled (period must stay <=
    EventCfg.dt_min_s or a fast event burst could skip a marker). hide_at
    is a 1-element list used as a mutable float cell."""
    if emitter.last_emit_seq > last_seen_seq:
        last_seen_seq = emitter.last_emit_seq
        evt = emitter.last_event
        kind = evt["kind"] if evt else None
        if kind in markers:
            bearing = drone_yaw + math.radians(random.uniform(-EVENT_MARKER_BEARING_JITTER_DEG,
                                                                EVENT_MARKER_BEARING_JITTER_DEG))
            mx = drone_pos[0] + EVENT_MARKER_AHEAD_M * math.cos(bearing)
            my = drone_pos[1] + EVENT_MARKER_AHEAD_M * math.sin(bearing)
            for k, body in markers.items():
                rgb = EVENT_MARKER_SPECS[k][2]
                if k == kind:
                    # LANE_BLOCK's long axis must span the drone's path, so
                    # orient it perpendicular to heading (not the jittered
                    # placement bearing). No-op for the Z-symmetric markers.
                    orn = p.getQuaternionFromEuler([0, 0, drone_yaw + math.pi / 2.0]) \
                        if k == "LANE_BLOCK" else [0, 0, 0, 1]
                    p.resetBasePositionAndOrientation(body, [mx, my, drone_pos[2]], orn)
                    p.changeVisualShape(body, -1, rgbaColor=rgb + [0.85])
                else:
                    p.changeVisualShape(body, -1, rgbaColor=rgb + [0.0])
            hide_at[0] = sim_now + EVENT_MARKER_VISIBLE_S

    if sim_now > hide_at[0]:
        hide_event_markers(markers)
        hide_at[0] = float("inf")

    return last_seen_seq
