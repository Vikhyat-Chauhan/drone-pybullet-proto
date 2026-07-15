"""Procedural NFZ arena generation: city-grid and Perlin-noise modes, ported
from CANavigator's arena_generator_{city,perlin}.py (self-contained, no
Gazebo SDF -- rectangles are plain Building dataclasses; write_meta() emits
the JSON format sim/violations.py::load_rects() consumes).

These rectangles are a horizontal no-fly-zone *scoring overlay*, not
physical obstacles -- viz/world_builder.py gives them no collision/visual
shape.
"""
from dataclasses import dataclass, field
import json
import os
import random
from typing import List, Optional, Tuple

import numpy as np
from noise import pnoise2


# Flat NFZ slab height (matches CANavigator's Z_THICK/Z_EPS): every rectangle
# gets the same uniform z-extent since these are a scoring overlay, not
# literal building models.
LEGACY_Z_SIZE = 5.0
LEGACY_Z_CENTER = 0.0


@dataclass
class Building:
    cx: float
    cy: float
    hx: float  # half-extent x
    hy: float  # half-extent y
    height: float = LEGACY_Z_SIZE


@dataclass
class ArenaCfg:
    # Domain + city-grid knobs restored to match CANavigator's
    # ArenaGenCfg/arena_generator_city.py exactly -- they were never meant
    # to be tuned down for this port.
    x_min: float = -100.0
    x_max: float = 100.0
    y_min: float = -50.0
    y_max: float = 50.0
    major_m: float = 60.0    # city_major_m: avenue spacing
    minor_m: float = 22.0    # city_minor_m: street spacing
    road_w_m: float = 4.5
    lot_w_m: float = 13.0
    lot_h_m: float = 11.0
    setback_m: float = 0.8   # sidewalk inset from road into block
    fill_prob: float = 0.9
    jitter_m: float = 2.0    # size jitter shrinking a filled lot's footprint
    seed: int = 42


def _axis_lines(vmin: float, vmax: float, major: float, minor: float) -> List[float]:
    """Ported verbatim from _NoFlyCity._axis_lines: major+minor grid lines,
    centered at 0, deduped."""
    lines = {vmin, vmax, 0.0}

    def add(step):
        if step == 0:
            return
        k = 0
        while True:
            pos = 0.0 + k * step
            if pos > vmax + 2 * abs(step):
                break
            if pos >= vmin - 2 * abs(step):
                lines.add(pos)
            k += 1
        k = 1
        while True:
            pos = 0.0 - k * step
            if pos < vmin - 2 * abs(step):
                break
            if pos <= vmax + 2 * abs(step):
                lines.add(pos)
            k += 1

    add(major)
    add(minor)
    dedup = sorted(lines)
    out: List[float] = []
    eps = 1e-6
    for L in dedup:
        if not out or abs(L - out[-1]) > eps:
            out.append(L)
    return out


def _adjacent_pairs(lines: List[float], vmin: float, vmax: float) -> List[Tuple[float, float]]:
    """Ported verbatim from _NoFlyCity._adjacent_pairs."""
    pairs = []
    last = None
    for L in lines:
        if last is None:
            last = L
            continue
        a, b = last, L
        if b > a:
            pairs.append((a, b))
        last = L
    return [(max(vmin, a), min(vmax, b)) for (a, b) in pairs]


def _tiles(a: float, b: float, target: float) -> List[Tuple[float, float]]:
    """Ported verbatim from _NoFlyCity._tiles."""
    spans = []
    if target <= 0:
        return [(a, b)]
    x = a
    while x + target < b:
        spans.append((x, x + target))
        x += target
    if x < b:
        spans.append((x, b))
    return spans


@dataclass
class CityLayout:
    """Extends generate_buildings()'s output with cosmetic-only ground detail
    for the renderer: roads (grid line strips), pavement (developed block
    footprints), grass (unfilled lots). Only `.buildings` feeds
    violations.py / LiDAR-collision setup."""
    buildings: List[Building]
    grass_rects: List[Tuple[float, float, float, float]]     # cx, cy, w, h
    pavement_rects: List[Tuple[float, float, float, float]]  # cx, cy, w, h
    road_rects: List[Tuple[float, float, float, float]]      # cx, cy, w, h


def generate_city_layout(cfg: ArenaCfg) -> CityLayout:
    """Ported from _NoFlyCity.run(): road lines carve the domain into blocks,
    blocks are inset by road-half-width + sidewalk setback, each block is
    tiled into lots, each lot independently filled with probability
    fill_prob. Also records road/pavement/grass rectangles for rendering.

    IMPORTANT: `.buildings` are a scoring overlay for zone_violations, not
    physical obstacles (pass_through=True, visual_alpha=0.0) -- yet
    experiment/orchestrator.py still gives them a PyBullet collision shape
    (see viz/world_builder.py's build_arena_bodies() docstring for why).
    Road/pavement/grass rects are cosmetic-only and never get a collision
    shape.
    """
    rng = random.Random(cfg.seed)
    buildings: List[Building] = []
    grass_rects: List[Tuple[float, float, float, float]] = []
    pavement_rects: List[Tuple[float, float, float, float]] = []

    x_roads = _axis_lines(cfg.x_min, cfg.x_max, cfg.major_m, cfg.minor_m)
    y_roads = _axis_lines(cfg.y_min, cfg.y_max, cfg.major_m, cfg.minor_m)
    x_blocks = _adjacent_pairs(x_roads, cfg.x_min, cfg.x_max)
    y_blocks = _adjacent_pairs(y_roads, cfg.y_min, cfg.y_max)
    road_half = cfg.road_w_m * 0.5

    # _axis_lines() intentionally over-generates candidate lines beyond
    # the domain (relied on by _adjacent_pairs()'s own clamping for block
    # edges) -- filter to the visible/reachable arena before turning them
    # into road strips, or spurious roads render outside the boundary walls.
    road_rects = [(L, (cfg.y_min + cfg.y_max) / 2.0, cfg.road_w_m, cfg.y_max - cfg.y_min)
                  for L in x_roads if cfg.x_min <= L <= cfg.x_max]
    road_rects += [((cfg.x_min + cfg.x_max) / 2.0, L, cfg.x_max - cfg.x_min, cfg.road_w_m)
                   for L in y_roads if cfg.y_min <= L <= cfg.y_max]

    for (xa, xb) in x_blocks:
        for (ya, yb) in y_blocks:
            xa_i = xa + road_half + cfg.setback_m
            xb_i = xb - road_half - cfg.setback_m
            ya_i = ya + road_half + cfg.setback_m
            yb_i = yb - road_half - cfg.setback_m
            if xb_i <= xa_i or yb_i <= ya_i:
                continue

            x_lots = _tiles(xa_i, xb_i, cfg.lot_w_m)
            y_lots = _tiles(ya_i, yb_i, cfg.lot_h_m)
            block_developed = False
            for (lx0, lx1) in x_lots:
                for (ly0, ly1) in y_lots:
                    cx, cy = (lx0 + lx1) * 0.5, (ly0 + ly1) * 0.5
                    if rng.random() > cfg.fill_prob:
                        grass_rects.append((cx, cy, lx1 - lx0, ly1 - ly0))
                        continue
                    block_developed = True
                    w = max(1e-3, min(lx1 - lx0, (lx1 - lx0) - rng.uniform(0, cfg.jitter_m)))
                    h = max(1e-3, min(ly1 - ly0, (ly1 - ly0) - rng.uniform(0, cfg.jitter_m)))
                    buildings.append(Building(cx=cx, cy=cy, hx=w / 2.0, hy=h / 2.0))

            if block_developed:
                # Block-granularity pavement pad (whole inset lot footprint),
                # not per-building driveways -- stays inside the setback
                # margin so it never overlaps road_rects.
                pavement_rects.append(((xa_i + xb_i) / 2.0, (ya_i + yb_i) / 2.0, xb_i - xa_i, yb_i - ya_i))

    return CityLayout(buildings=buildings, grass_rects=grass_rects,
                       pavement_rects=pavement_rects, road_rects=road_rects)


def generate_buildings(cfg: ArenaCfg) -> list[Building]:
    """Buildings only -- see generate_city_layout() for the fuller
    road/pavement/grass ground-detail layout used by viz/world_builder.py's
    render_city_ground_detail()."""
    return generate_city_layout(cfg).buildings


def pick_start_target(cfg: ArenaCfg, buildings: list[Building], seed_offset: int = 1,
                       min_dist: float = 30.0):
    """Kept for the earlier verification scripts / quick prototyping.
    For the real orchestrator, prefer pick_target() below, which ports
    CANavigator's two-phase (random-sample + deterministic-grid-fallback)
    target picker -- this one can silently return (0,0) if unlucky."""
    rng = random.Random(cfg.seed + seed_offset)

    def collides(px, py, margin=1.5):
        for b in buildings:
            if abs(px - b.cx) < b.hx + margin and abs(py - b.cy) < b.hy + margin:
                return True
        return False

    def sample():
        for _ in range(5000):
            px = rng.uniform(cfg.x_min + 2, cfg.x_max - 2)
            py = rng.uniform(cfg.y_min + 2, cfg.y_max - 2)
            if not collides(px, py):
                return px, py
        return 0.0, 0.0

    start = sample()
    for _ in range(5000):
        target = sample()
        dx, dy = target[0] - start[0], target[1] - start[1]
        if (dx * dx + dy * dy) ** 0.5 >= min_dist:
            return start, target
    return start, sample()


# ===================== Perlin-noise mode =====================

@dataclass
class PerlinArenaCfg:
    x_min: float = -100.0
    x_max: float = 100.0
    y_min: float = -50.0
    y_max: float = 50.0
    cell_m: float = 5.0
    density: float = 0.2
    corr_len_m: float = 10.0
    height_min: float = 3.0
    height_max: float = 10.0
    seed: int = 42


def generate_buildings_perlin(cfg: PerlinArenaCfg) -> List[Building]:
    """Ported from arena_generator_perlin.py's _NoFly: sample 2D Perlin
    noise over a grid, threshold by density quantile, greedily merge into
    maximal axis-aligned rectangles."""
    xs = np.arange(cfg.x_min, cfg.x_max, cfg.cell_m, dtype=float)
    ys = np.arange(cfg.y_min, cfg.y_max, cfg.cell_m, dtype=float)

    freq = 1.0 / max(cfg.corr_len_m, 1e-6)
    z = np.zeros((len(xs), len(ys)), dtype=np.float32)
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            zx = (x - xs[0]) * freq + cfg.seed * 1.1337
            zy = (y - ys[0]) * freq - cfg.seed * 0.7331
            z[i, j] = pnoise2(zx, zy, octaves=3, repeatx=1024, repeaty=1024, base=cfg.seed)
    zmin, zptp = float(np.min(z)), float(np.ptp(z))
    nmap = (z - zmin) / max(zptp, 1e-9)

    thr = float(np.quantile(nmap, 1.0 - min(max(cfg.density, 0.0), 1.0)))
    mask = nmap >= thr

    used = np.zeros_like(mask, dtype=bool)
    rects: List[Tuple[int, int, int, int]] = []
    nx, ny = mask.shape
    for i in range(nx):
        j = 0
        while j < ny:
            if mask[i, j] and not used[i, j]:
                w = 1
                while j + w < ny and mask[i, j + w] and not used[i, j + w]:
                    w += 1
                h = 1
                good = True
                while i + h < nx and good:
                    for jj in range(j, j + w):
                        if not (mask[i + h, jj] and not used[i + h, jj]):
                            good = False
                            break
                    if good:
                        h += 1
                for ii in range(i, i + h):
                    for jj in range(j, j + w):
                        used[ii, jj] = True
                rects.append((i, j, h, w))
                j += w
            else:
                j += 1

    rng = random.Random(cfg.seed)
    buildings: List[Building] = []
    for (i0, j0, h, w) in rects:
        x0, y0 = xs[i0], ys[j0]
        w_m, h_m = w * cfg.cell_m, h * cfg.cell_m
        cx, cy = float(x0 + w_m / 2.0), float(y0 + h_m / 2.0)
        height = rng.uniform(cfg.height_min, cfg.height_max)
        buildings.append(Building(cx=cx, cy=cy, hx=w_m / 2.0, hy=h_m / 2.0, height=height))
    return buildings


# ===================== Target picking (robust, two-phase) =====================

def pick_target(x_min: float, x_max: float, y_min: float, y_max: float,
                 buildings: List[Building], start_xy: Tuple[float, float],
                 seed: int, radius: float = 0.6, margin_walls: float = 5.0,
                 margin_rect: float = 1.0, min_dist_start: float = 25.0,
                 max_tries: int = 5000) -> Tuple[float, float]:
    """Ported from arena_generator_{city,perlin}.py's _Target._pick_xy:
    random sampling (rejecting points inside any NFZ rect inflated by a
    margin, and within min_dist_start of start_xy) up to max_tries, then a
    deterministic grid-scan fallback so dense arenas can't fail silently."""
    rnd = random.Random(seed)
    wall = margin_walls + radius
    left, right = x_min + wall, x_max - wall
    bottom, top = y_min + wall, y_max - wall
    keepout = margin_rect + radius
    min_d2 = min_dist_start ** 2

    def safe(px, py):
        for b in buildings:
            hw, hh = b.hx + keepout, b.hy + keepout
            if abs(px - b.cx) <= hw and abs(py - b.cy) <= hh:
                return False
        dx, dy = px - start_xy[0], py - start_xy[1]
        if dx * dx + dy * dy < min_d2:
            return False
        return True

    for _ in range(max_tries):
        x, y = rnd.uniform(left, right), rnd.uniform(bottom, top)
        if safe(x, y):
            return x, y

    step = max(2.0 * radius, 0.5)
    xg = left
    while xg <= right + 1e-9:
        yg = bottom
        while yg <= top + 1e-9:
            if safe(xg, yg):
                return xg, yg
            yg += step
        xg += step
    raise RuntimeError("No collision-free target position found.")


# ===================== Plain-data JSON meta writer =====================

def write_nofly_meta(path: str, x_min: float, x_max: float, y_min: float, y_max: float,
                      buildings: List[Building], seed: int, mode: str = "city") -> None:
    """Same JSON shape CANavigator's arena_generator_*.py::_write_meta()
    produced (minus the Gazebo SDF), so violations.py::load_rects() ports
    with zero parsing changes."""
    meta = {
        "x_min": float(x_min), "x_max": float(x_max),
        "y_min": float(y_min), "y_max": float(y_max),
        "mode": mode, "seed": int(seed),
        "rectangles_xywh": [[b.cx, b.cy, 2 * b.hx, 2 * b.hy] for b in buildings],
        "height_mode": {"type": "legacy_only", "legacy": {"z_center": 0.0, "z_size": 5.0}},
        "pass_through": True, "visual_alpha": 0.0, "color_rgb": [1.0, 0.0, 0.0],
    }
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(meta, f, indent=2)


def write_target_meta(path: str, x: float, y: float, z: float, radius: float,
                       in_meta_nofly: str, start_xy: Tuple[float, float], min_dist_start: float) -> None:
    meta = {
        "name": "target_sphere", "x": x, "y": y, "z": z, "radius": radius,
        "color": [1.0, 0.9, 0.0, 1.0], "in_meta_nofly": in_meta_nofly,
        "avoid_start_xy": list(start_xy), "min_dist_start": min_dist_start,
    }
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(meta, f, indent=2)
