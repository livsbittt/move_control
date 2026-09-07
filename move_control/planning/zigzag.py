"""Subject: zigzag mapping. Boustrophedon coverage of the mapped free space.

Lanes run along the longer map axis at lane_width pitch (robot swath ~15 cm,
lane_width 0.12 m overlaps). Serpentine order starts at the lane nearest the
robot; connectors between waypoints are planned separately (best_route).
covered cells are excluded before lane building, so replanning with a fresh
covered set advances coverage instead of repeating it.
"""
import heapq
import math

from .gridmap import nearest_free

# 8-connected steps shared by the walk-distance probe flood (no corner
# cutting — the same rule best_route's A* uses).
_STEPS8 = ((1, 0), (-1, 0), (0, 1), (0, -1),
           (1, 1), (1, -1), (-1, 1), (-1, -1))


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

    def probe_point(self, pose, walk_min_m=0.10, covered=None,
                    straight_min_m=None):
        """Farthest free cell by walk distance, no corner cutting.

        Used when coverage is done but the world may still be bigger than
        the map: driving there pushes the sensor envelope outward.

        The region flood is 8-conn and may cross corner-cut diagonals A*
        cannot drive (best_route forbids cutting corners), so a probe on
        one of those dead-flood cells returned best_route=None at every
        clearance and the loop sat frozen on 'coverage done'. Farthest by
        walk distance uses best_route's own connectivity, so the raw-map
        route always exists. walk_min_m skips cells within the driver's
        arrival tolerance so standing on the target can't re-deadlock.

        covered (sim rig): cells already swept are skipped, so the probe
        ADVANCES — the farthest-walk cell otherwise stays the same cell
        after the robot stands on it and the loop re-picks it forever.
        straight_min_m optionally rejects walk-far-but-straight-near cells
        (a point 2 cm away across a wall corner); None keeps the plain
        walk-farthest contract. Failing that gate, the walk-farthest cell
        is still returned — an imperfect probe beats 'coverage done'.

        The gates filter only the target pick, never the expansion: an
        early continue once truncated the flood at the start cell's own
        neighbours (all straight-near) and probe_point returned None — or
        worse, a next-door cell — on every map.
        """
        if not self.region:
            return None
        start = nearest_free(self.m, self.m.world_to_grid(*pose))
        if start is None:
            return None
        wmin = walk_min_m / self.m.res  # m -> cells; the flood counts cells
        dist = {start: 0.0}
        q = [(0.0, start)]
        far_c, far_d = None, -1.0
        far_any, far_any_d = None, -1.0
        while q:
            d, cur = heapq.heappop(q)
            if d > dist.get(cur, 1e9):
                continue
            if d >= wmin:
                if d > far_any_d:
                    far_any, far_any_d = cur, d
                if covered is None or cur not in covered:
                    wx, wy = self.m.grid_to_world(*cur)
                    if (straight_min_m is None or
                            math.hypot(wx - pose[0],
                                       wy - pose[1]) >= straight_min_m):
                        if d > far_d:
                            far_c, far_d = cur, d
            for dc, dr in _STEPS8:
                if dc and dr and not (
                        self.m.is_free(cur[0] + dc, cur[1])
                        and self.m.is_free(cur[0], cur[1] + dr)):
                    continue  # no corner cutting, like best_route
                nx = (cur[0] + dc, cur[1] + dr)
                if not self.m.is_free(*nx) or nx in dist:
                    continue
                nd = d + math.hypot(dc, dr)
                if nd < dist.get(nx, 1e9):
                    dist[nx] = nd
                    heapq.heappush(q, (nd, nx))
        # No cell passed the straight gate: return the walk-far fallback
        # rather than None — 'coverage done' on a walk-far region is the
        # exact deadlock this probe exists to break. The caller's latch +
        # reach_tol release keeps a fallback pick from re-pinging.
        far_c = far_c if far_c is not None else far_any
        if far_c is None:
            return None
        return self.m.grid_to_world(*far_c)

    def _build(self, region, stride, step, min_run, covered):
        lanes = []
        if not region:
            return lanes
        horiz = self.m.w >= self.m.h
        keys = sorted({(r if horiz else c) for (c, r) in region})
        for key in keys[::stride]:
            cells = sorted((c if horiz else r2) for c, r2 in region
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
                qualifying.append([(c, key) if horiz else (key, c) for c in wps])
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
            wx, wy = m.grid_to_world(c0 + dc, r0 + dr)
            if math.hypot(wx - x, wy - y) <= radius_m and m.is_free(c0 + dc, r0 + dr):
                covered.add((c0 + dc, r0 + dr))
