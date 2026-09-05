"""Subject: zigzag mapping. Boustrophedon coverage of the mapped free space.

Lanes run along the longer map axis at lane_width pitch (robot swath ~15 cm,
lane_width 0.12 m overlaps). Serpentine order starts at the lane nearest the
robot; connectors between waypoints are planned separately (best_route).
covered cells are excluded before lane building, so replanning with a fresh
covered set advances coverage instead of repeating it.
"""
import math

from .gridmap import nearest_free


class ZigzagPlanner:
    """Ordered coverage waypoints over the reachable free region."""

    def __init__(self, m, start=None, lane_width=0.12, lane_step=0.20,
                 covered=None):
        self.m = m
        res = m.res
        covered = covered if covered is not None else set()
        stride = max(1, int(round(lane_width / res)))
        step = max(2, int(round(lane_step / res)))
        min_run = 3  # below ~15 cm the robot doesn't fit
        if start is not None:
            sc = nearest_free(m, m.world_to_grid(*start))
            region = m.flood(sc) if sc else set()
        else:
            region = set(m.free_cells())
        self.region = region
        self.lanes = self._build(region, stride, step, min_run, covered)
        self.wps = self._order(self.lanes, start)
        self.points = [m.grid_to_world(c, r) for c, r in self.wps]

    def waypoints(self):
        return list(self.points)

    def _build(self, region, stride, step, min_run, covered):
        lanes = []
        if not region:
            return lanes
        horiz = self.m.w >= self.m.h
        keys = sorted({(r if horiz else c) for (c, r) in region})
        for key in keys[::stride]:
            cells = sorted(c for c, r2 in region
                           if (r2 if horiz else c) == key
                           and (c, r2) not in covered)
            runs = []
            run = []
            for cval in cells:
                if run and cval == run[-1] + 1:
                    run.append(cval)
                else:
                    if run:
                        runs.append(run)
                    run = [cval]
            if run:
                runs.append(run)
            qualifying = []
            for run in runs:
                if len(run) < min_run:
                    continue
                wps = list(dict.fromkeys(run[::step] + [run[-1]]))
                qualifying.append([(c, key) for c in wps])
            if qualifying:
                lanes.append({'key': key, 'runs': qualifying})
        return lanes

    def _order(self, lanes, start):
        """Serpentine, rotated to the lane nearest the robot."""
        if not lanes:
            return []
        horiz = self.m.w >= self.m.h
        sc = self.m.world_to_grid(*start) if start is not None else None
        k0 = 0
        if sc is not None:
            skey = sc[1 if horiz else 0]
            k0 = min(range(len(lanes)),
                     key=lambda i: abs(lanes[i]['key'] - skey))
        order = list(range(k0, len(lanes))) + list(range(k0 - 1, -1, -1))
        out = []
        dir0 = True
        for i, li in enumerate(order):
            flat = [wp for run in lanes[li]['runs'] for wp in run]
            if i == 0:
                if sc is not None and flat:
                    k = min(range(len(flat)),
                            key=lambda j: abs(flat[j][0] - sc[0]) + abs(flat[j][1] - sc[1]))
                    flat = flat[k:] if 2 * k <= len(flat) else \
                        list(reversed(flat[:k + 1]))
                dir0 = flat[0] <= flat[-1]
            else:
                fwd = (i % 2 == 0) == dir0
                if not fwd:
                    flat = list(reversed(flat))
            out.extend(flat)
        return out


def cover_ring(covered, m, x, y, radius_m=0.16):
    """Cells within radius of (x, y) count as swept (mapping coverage).

    Marks known-free cells only, so nothing beyond a wall joins the set.
    """
    c0, r0 = m.world_to_grid(x, y)
    rad = int(math.ceil(radius_m / m.res))
    for dc in range(-rad, rad + 1):
        for dr in range(-rad, rad + 1):
            if dc * dc + dr * dr <= rad * rad and m.is_free(c0 + dc, r0 + dr):
                covered.add((c0 + dc, r0 + dr))
