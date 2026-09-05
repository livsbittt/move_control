"""Subject: goal pick. Frontiers on the map = free cells touching unknown.

Clusters boundary cells, snaps each cluster centroid to the nearest free
cell, sizes clusters, and returns the first reachable one as the point to go.
"""
from collections import deque

from .astar import best_route
from .gridmap import UNKNOWN, _snap


def frontier_points(m, min_size=6):
    """Frontier clusters, biggest first.

    Returns [{'cell': (c,r), 'x': .., 'y': .., 'size': n}, ..] (x/y world m,
    snapped to free space).
    """
    seed_set = set()
    for (c, r) in m.free_cells():
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if m.cell(c + dc, r + dr) == UNKNOWN:
                seed_set.add((c, r))
                break
    seen = set()
    out = []
    for s in sorted(seed_set):
        if s in seen:
            continue
        seen.add(s)
        comp = [s]
        q = deque((s,))
        while q:
            c, r = q.popleft()
            for dc, dr in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                nc = (c + dc, r + dr)
                if nc not in seen and nc in seed_set:
                    seen.add(nc)
                    comp.append(nc)
                    q.append(nc)
        if len(comp) < min_size:
            continue
        cx = sum(c for c, _r in comp) / len(comp)
        cy = sum(r for _c, r in comp) / len(comp)
        snap = _snap(m, (cx, cy))
        if snap is None:
            continue
        x, y = m.grid_to_world(*snap)
        out.append({'cell': snap, 'x': x, 'y': y, 'size': len(comp)})
    out.sort(key=lambda f: f['size'], reverse=True)
    return out


def pick_goal(m, start, min_size=6, clear_m=0.06, retry_clear_m=None,
              min_route_m=0.08):
    """First usable frontier goal (size desc). None if nothing worth driving.

    Returns {'kind': 'frontier', 'x', 'y', 'size', 'route', 'clear_m'}.
    retry_clear_m: when the inflated map seals thin corridors, a second pass
    at retry_clear_m (0.0 = raw map) still finds an approach; the safety gate
    still guards the hardware in that last stretch.
    min_route_m: routes shorter than this are skipped — a frontier the robot
    already sits on reveals nothing new, and returning it as the point to go
    would just spin the replan loop.
    """
    for cm in (clear_m, retry_clear_m):
        if cm is None:
            continue
        for f in frontier_points(m, min_size):
            route = best_route(m, start, (f['x'], f['y']), clear_m=cm)
            if route and route['length'] >= min_route_m:
                return {'kind': 'frontier', 'x': f['x'], 'y': f['y'],
                        'size': f['size'], 'route': route, 'clear_m': cm}
    return None
