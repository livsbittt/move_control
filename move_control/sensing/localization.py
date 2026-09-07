"""Subject: saved-map observation agreement and expiring localization authority."""
import math
import numpy as np


def planar_yaw(x, y, z, w):
    values = (x, y, z, w)
    if (not all(math.isfinite(v) for v in values) or
            abs(sum(v*v for v in values)-1.) > .01 or
            1.-2.*(x*x+y*y) < math.cos(math.radians(5))):
        raise ValueError('Localization requires a finite unit planar rotation')
    return math.atan2(2.*(w*z+x*y), 1.-2.*(y*y+z*z))


def lease_ready(status, now, timeout=.5):
    if not isinstance(status, dict) or status.get('ready') is not True:
        return False
    stamp = status.get('stamp_ns')
    if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
        return False
    age = now - stamp * 1e-9
    return math.isfinite(age) and 0. <= age <= timeout


class Confidence:
    def __init__(self, stable_scans=10):
        self.required = max(1, stable_scans)
        self.count = 0
        self.last_stamp = -math.inf

    def observe(self, stamp, good):
        if not good or stamp < self.last_stamp:
            self.count = 0
        elif stamp > self.last_stamp:
            self.count += 1
        self.last_stamp = stamp
        return good and self.count >= self.required


class MapAgreement:
    def __init__(self, grid, resolution, origin, tolerance=.04):
        self.grid = np.asarray(grid)
        self.resolution = resolution
        self.origin = np.asarray(origin)
        # The occupied surface is uncertain by one grid cell. Unknown is never
        # a wall observation; matching unknown cells cannot authorize motion.
        occupied = self.grid >= 65
        self.likelihood = np.zeros(occupied.shape, dtype=float)
        self.near = np.zeros_like(occupied)
        radius = int(math.ceil(tolerance / resolution))
        h, w = occupied.shape
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if math.hypot(dx, dy) * resolution > tolerance + 1e-7:
                    continue
                y0, y1 = max(0, dy), min(h, h + dy)
                x0, x1 = max(0, dx), min(w, w + dx)
                self.near[y0:y1, x0:x1] |= occupied[y0-dy:y1-dy, x0-dx:x1-dx]
        spread = int(math.ceil(.12 / resolution))
        for dy in range(-spread, spread + 1):
            for dx in range(-spread, spread + 1):
                y0, y1 = max(0, dy), min(h, h + dy)
                x0, x1 = max(0, dx), min(w, w + dx)
                if y1 <= y0 or x1 <= x0:
                    continue
                value = math.exp(-((dx*resolution)**2 + (dy*resolution)**2) / (2*.025**2))
                dest = self.likelihood[y0:y1, x0:x1]
                np.maximum(dest, occupied[y0-dy:y1-dy, x0-dx:x1-dx] * value, out=dest)

    def score(self, sensor_pose, ranges, angles):
        ranges, angles = np.asarray(ranges), np.asarray(angles)
        valid = np.isfinite(ranges) & (ranges > .05) & np.isfinite(angles)
        if np.count_nonzero(valid) < 30 or not all(map(math.isfinite, sensor_pose)):
            return 0.
        ranges, angles = ranges[valid], angles[valid] + sensor_pose[2]
        xy = np.column_stack((np.cos(angles), np.sin(angles))) * ranges[:, None]
        xy += sensor_pose[:2]
        cells = np.floor((xy - self.origin) / self.resolution).astype(int)
        h, w = self.near.shape
        inside = (cells[:, 0] >= 0) & (cells[:, 0] < w) & (cells[:, 1] >= 0) & (cells[:, 1] < h)
        hit = np.zeros(len(cells), dtype=bool)
        hit[inside] = self.near[cells[inside, 1], cells[inside, 0]]
        # An endpoint behind an intervening wall is a false match, common in
        # repeated maze corridors. Check the observed free ray as well.
        steps = np.arange(.05, max(ranges), self.resolution * .75)
        if len(steps):
            distances = np.minimum(steps[None, :], np.maximum(0., ranges[:, None] - .06))
            direction = np.column_stack((np.cos(angles), np.sin(angles)))
            ray = np.asarray(sensor_pose[:2]) + direction[:, None, :] * distances[:, :, None]
            ray_cells = np.floor((ray-self.origin) / self.resolution).astype(int)
            rx, ry = ray_cells[:, :, 0], ray_cells[:, :, 1]
            in_map = (rx >= 0) & (rx < w) & (ry >= 0) & (ry < h)
            blocked = np.ones(rx.shape, dtype=bool)
            blocked[in_map] = self.grid[ry[in_map], rx[in_map]] >= 65
            obstructed = np.any(blocked, axis=1)
            # Grid rasterization moves a grazing wall by up to one cell. A
            # parallel ray on either side handles that uncertainty; an actual
            # intervening wall still blocks all three rays.
            normal = np.column_stack((-np.sin(angles), np.cos(angles)))
            for sign in (-1., 1.):
                shifted = ray + sign*self.resolution*normal[:, None, :]
                indices = np.floor((shifted-self.origin)/self.resolution).astype(int)
                sx, sy = indices[:, :, 0], indices[:, :, 1]
                valid = (sx >= 0) & (sx < w) & (sy >= 0) & (sy < h)
                blocked = np.ones(sx.shape, dtype=bool)
                blocked[valid] = self.grid[sy[valid], sx[valid]] >= 65
                obstructed &= np.any(blocked, axis=1)
            hit &= ~obstructed
        return float(np.mean(hit))

    def _qualities(self, poses, ranges, angles):
        a = poses[:, 2, None] + angles[None, :]
        x = poses[:, 0, None] + np.cos(a)*ranges
        y = poses[:, 1, None] + np.sin(a)*ranges
        ix = np.floor((x-self.origin[0]) / self.resolution).astype(int)
        iy = np.floor((y-self.origin[1]) / self.resolution).astype(int)
        h, w = self.grid.shape
        inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        values = np.zeros(ix.shape)
        values[inside] = self.likelihood[iy[inside], ix[inside]]
        return np.mean(values, axis=1)

    def footprint_clear(self, x, y, radius):
        if not all(math.isfinite(v) for v in (x, y, radius)) or radius <= 0:
            return False
        h, w = self.grid.shape
        ox, oy = self.origin
        if x-radius < ox or y-radius < oy or x+radius >= ox+w*self.resolution or y+radius >= oy+h*self.resolution:
            return False
        ix0, ix1 = int((x-radius-ox)/self.resolution), int((x+radius-ox)/self.resolution)+1
        iy0, iy1 = int((y-radius-oy)/self.resolution), int((y+radius-oy)/self.resolution)+1
        yy, xx = np.mgrid[iy0:iy1, ix0:ix1]
        overlaps = np.hypot(ox+(xx+.5)*self.resolution-x, oy+(yy+.5)*self.resolution-y) <= radius+self.resolution/math.sqrt(2)
        return bool(np.all(self.grid[yy[overlaps], xx[overlaps]] == 0))

    def global_match(self, ranges, angles, radius, minimum=.9, margin=.04):
        """Coarse-to-fine scan search. Repeated geometry must remain ambiguous.

        Returns a candidate, not driving authority. AMCL and subsequent fresh
        scans still have to confirm it. Inputs are sensor-frame observations.
        """
        ranges, angles = np.asarray(ranges), np.asarray(angles)
        valid = np.isfinite(ranges) & (ranges > .05) & np.isfinite(angles)
        ranges, angles = ranges[valid], angles[valid]
        if len(ranges) < 30:
            return {'unique': False, 'reason': 'insufficient-scan'}
        h, w = self.grid.shape
        free = self.grid == 0
        # Candidate origins need room for the footprint, never inside walls.
        blocked = ~free
        clear = free.copy()
        inflated_radius = radius + self.resolution/math.sqrt(2)
        cells = int(math.ceil(inflated_radius/self.resolution))
        for dy in range(-cells, cells+1):
            for dx in range(-cells, cells+1):
                if math.hypot(dx, dy)*self.resolution > inflated_radius:
                    continue
                y0, y1 = max(0, dy), min(h, h+dy)
                x0, x1 = max(0, dx), min(w, w+dx)
                if y1 > y0 and x1 > x0:
                    clear[y0:y1, x0:x1] &= ~blocked[y0-dy:y1-dy, x0-dx:x1-dx]
        stride = max(1, round(.04/self.resolution))
        yy, xx = np.nonzero(clear[::stride, ::stride])
        if not len(xx):
            return {'unique': False, 'reason': 'no-footprint-clear-candidate'}
        xy = np.column_stack((xx*stride+.5, yy*stride+.5))*self.resolution + self.origin
        candidates = []
        for theta in np.arange(-math.pi, math.pi, math.radians(5)):
            poses = np.column_stack((xy, np.full(len(xy), theta)))
            quality = self._qualities(poses, ranges[::2], angles[::2])
            for idx in np.argsort(quality)[-16:]:
                candidates.append((float(quality[idx]), poses[idx]))
        seeds = []
        for quality, pose in sorted(candidates, key=lambda item: item[0], reverse=True):
            if all(math.dist(pose[:2], other[:2]) > .18 or
                   abs(math.atan2(math.sin(pose[2]-other[2]), math.cos(pose[2]-other[2]))) > .3
                   for other in seeds):
                seeds.append(pose)
            if len(seeds) >= 48:
                break
        offsets = np.array(np.meshgrid(np.arange(-.05, .051, .01),
            np.arange(-.05, .051, .01), np.radians(np.arange(-5., 5.1, 1.)), indexing='ij')).reshape(3, -1).T
        results = []
        for seed in seeds:
            poses = seed + offsets
            indices = np.floor((poses[:, :2]-self.origin)/self.resolution).astype(int)
            inside = (indices[:, 0] >= 0) & (indices[:, 0] < w) & (indices[:, 1] >= 0) & (indices[:, 1] < h)
            allowed = np.zeros(len(poses), dtype=bool)
            allowed[inside] = clear[indices[inside, 1], indices[inside, 0]]
            poses = poses[allowed]
            if not len(poses):
                continue
            quality = self._qualities(poses, ranges, angles)
            for idx in np.argsort(quality)[-3:]:
                pose = poses[idx]
                agreement = self.score(pose, ranges, angles)
                results.append((agreement*.7 + float(quality[idx])*.3, agreement, pose))
        if not results:
            return {'unique': False, 'reason': 'no-refined-candidate'}
        results.sort(key=lambda item: item[0], reverse=True)
        best = results[0]
        competitors = [r for r in results[1:] if math.dist(r[2][:2], best[2][:2]) > .18 or
                       abs(math.atan2(math.sin(r[2][2]-best[2][2]), math.cos(r[2][2]-best[2][2]))) > .3]
        gap = best[0] - competitors[0][0] if competitors else 1.
        return {'unique': bool(best[1] >= minimum and gap >= margin),
                'pose': best[2].tolist(), 'agreement': best[1], 'quality': best[0], 'margin': gap}
