"""2D horizontal LiDAR sensor implemented with PyBullet batch raycasting."""
import math
import pybullet as p


class Lidar2D:
    def __init__(self, num_rays: int = 36, max_range: float = 15.0, fov_deg: float = 270.0,
                 draw_debug: bool = True):
        self.num_rays = num_rays
        self.max_range = max_range
        self.fov = math.radians(fov_deg)
        self.draw_debug = draw_debug
        self._debug_ids = []

    def scan(self, drone_pos, yaw: float = 0.0):
        """Cast rays from drone_pos in a horizontal fan. Returns list of (angle, range)."""
        froms, tos = [], []
        angles = [
            -self.fov / 2 + i * self.fov / (self.num_rays - 1)
            for i in range(self.num_rays)
        ]
        for a in angles:
            ang = yaw + a
            dx = math.cos(ang) * self.max_range
            dy = math.sin(ang) * self.max_range
            froms.append(drone_pos)
            tos.append((drone_pos[0] + dx, drone_pos[1] + dy, drone_pos[2]))

        results = p.rayTestBatch(froms, tos)

        ranges = []
        for a, res, f, t in zip(angles, results, froms, tos):
            hit_fraction = res[2]
            rng = hit_fraction * self.max_range
            ranges.append((a, rng))

        if self.draw_debug:
            self._redraw(froms, tos, results)

        return ranges

    def _redraw(self, froms, tos, results):
        # Update existing debug lines in place (replaceItemUniqueId) rather
        # than remove+recreate every tick -- at 60Hz the latter is a known
        # PyBullet GUI perf killer (each add/remove round-trips the
        # renderer) and was making the GUI crawl in slow motion.
        for i, (f, t, res) in enumerate(zip(froms, tos, results)):
            hit_fraction = res[2]
            hit_point = res[3] if hit_fraction < 1.0 else t
            color = [1, 0, 0] if hit_fraction < 1.0 else [0, 1, 0]
            replace_id = self._debug_ids[i] if i < len(self._debug_ids) else -1
            did = p.addUserDebugLine(f, hit_point, color, lineWidth=1, lifeTime=0,
                                      replaceItemUniqueId=replace_id)
            if i < len(self._debug_ids):
                self._debug_ids[i] = did
            else:
                self._debug_ids.append(did)

    def scan_multilayer(self, drone_pos, yaw: float, n_layers: int,
                         vertical_angle_min: float, vertical_angle_increment: float):
        """Stack n_layers horizontal fans at different pitch offsets, one
        batched rayTestBatch call across all layers. Returns
        (flat_ranges, n_ranges_per_layer, n_layers), row-major: layer 0's
        n_ranges values first, then layer 1's, etc -- matching the
        organized-cloud layout _CloudSub assumed (height=n_layers,
        width=n_ranges, row-major)."""
        angles = [
            -self.fov / 2 + i * self.fov / (self.num_rays - 1)
            for i in range(self.num_rays)
        ]

        froms, tos = [], []
        for layer_idx in range(n_layers):
            pitch = vertical_angle_min + layer_idx * vertical_angle_increment
            for a in angles:
                ang = yaw + a
                dx = math.cos(pitch) * math.cos(ang) * self.max_range
                dy = math.cos(pitch) * math.sin(ang) * self.max_range
                dz = math.sin(pitch) * self.max_range
                froms.append(drone_pos)
                tos.append((drone_pos[0] + dx, drone_pos[1] + dy, drone_pos[2] + dz))

        results = p.rayTestBatch(froms, tos)
        flat = [res[2] * self.max_range for res in results]
        return flat, self.num_rays, n_layers
