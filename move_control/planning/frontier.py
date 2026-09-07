"""Subject: goal pick. Frontiers on the map = free cells touching unknown.

Clusters boundary cells, snaps each cluster centroid to the nearest free
cell, sizes clusters, and returns the first reachable one as the point to go.
"""
from collections import deque
import heapq
import math

from .astar import best_route
from .gridmap import UNKNOWN, nearest_free


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
        snap = nearest_free(m, (cx, cy))
        if snap is None:
            continue
        x, y = m.grid_to_world(*snap)
        # cells: the full boundary cluster — the watchdog benches the whole
        # cluster, not just the snapped cell (re-snapping picks a neighbour
        # cell of the same unreachable cluster next plan and the bench
        # churned inside it forever).
        out.append({'cell': snap, 'x': x, 'y': y, 'size': len(comp),
                    'cells': comp})
    out.sort(key=lambda f: f['size'], reverse=True)
    return out


def _reachable_costs(grid, start):
    """One search per clearance pass, with the same corner rules as A*."""
    if not (grid.ox <= start[0] < grid.ox + grid.w * grid.res and
            grid.oy <= start[1] < grid.oy + grid.h * grid.res):
        return {}
    cell = grid.world_to_grid(*start)
    if not grid.is_free(*cell):
        return {}
    costs = {cell: 0.0}
    queue = [(0.0, cell)]
    while queue:
        cost, cell = heapq.heappop(queue)
        if cost > costs[cell]:
            continue
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = cell[0] + dc, cell[1] + dr
            if not grid.is_free(*nxt):
                continue
            if dc and dr and (not grid.is_free(cell[0] + dc, cell[1]) or
                              not grid.is_free(cell[0], cell[1] + dr)):
                continue
            value = cost + math.hypot(dc, dr) * grid.res
            if value < costs.get(nxt, math.inf):
                costs[nxt] = value
                heapq.heappush(queue, (value, nxt))
    return costs


def pick_goal(m, start, min_size=6, clear_m=0.06, retry_clear_m=None,
              min_route_m=0.08, max_options=3, exclude=None):
    """Best frontier by unknown gained per metre driven (size / length).

    Every reachable frontier is scored, not just the biggest: a small
    frontier next door can beat a big one across the maze. Returns
    {'kind', 'x', 'y', 'size', 'route', 'clear_m', 'score', 'options'}
    where options are the top max_options candidates (chosen first), so the
    driver can show the alternatives.
    exclude: set of grid cells to skip — the stall watchdog benches
    frontiers the robot repeatedly fails to approach.
    retry_clear_m: when the inflated map seals thin corridors, a second pass
    at retry_clear_m (0.0 = raw map) still finds an approach; the safety gate
    still guards the hardware in that last stretch.
    min_route_m: routes shorter than this are skipped — a frontier the robot
    already sits on reveals nothing new, and returning it as the point to go
    would just spin the replan loop.
    """
    exclude = exclude or set()
    best_safe = None
    best_raw = None
    options = []
    clusters = frontier_points(m, min_size)
    # Robot configuration often uses the same margin for both passes.
    # Repeating it doubles graph work and fills alternatives with duplicates.
    margins = [clear_m]
    if retry_clear_m != clear_m:
        margins.append(retry_clear_m)
    for cm in margins:
        if cm is None:
            continue
        pass_best = None
        inflated = m.inflate(cm / m.res)
        reachable = _reachable_costs(inflated, start)
        for f in clusters:
            # Inflation can split a raw cluster across disconnected rooms.
            # Its centroid must not hide the safely reachable near side.
            center = nearest_free(inflated, f['cell'], max_occ=2)
            candidates = [cell for cell in f['cells'] if cell in reachable
                          and cell not in exclude
                          and reachable[cell] >= min_route_m]
            if (center in reachable and center not in exclude
                    and reachable[center] >= min_route_m):
                target = center
            elif candidates:
                target = min(candidates, key=lambda cell: (
                    math.dist(cell, f['cell']), reachable[cell], cell))
            else:
                continue
            route = best_route(m, start, m.grid_to_world(*target), clear_m=cm)
            if not route or route['length'] < min_route_m:
                continue
            if route['cells'][-1] in exclude:
                continue  # a snapped goal may land in a failed target region
            # NOTE: f['cells'] (the whole frontier cluster) is deliberately
            # NOT forwarded to the watchdog: measured on the isolated rig,
            # whole-cluster benching + the goal latch mass-benches every
            # cluster and latch-chases one unreachable goal — the map froze
            # at 383 known cells at minute 7 (run 6) vs 1771 with the
            # single-cell bench (run 2). Keep the single-cell bench.
            tx, ty = route['points'][-1]
            cand = {'kind': 'frontier', 'x': tx, 'y': ty,
                    'size': f['size'], 'route': route, 'clear_m': cm,
                    'score': f['size'] / max(route['length'], 1e-6)}
            options.append(cand)
            if pass_best is None or cand['score'] > pass_best['score']:
                pass_best = cand
        if cm == clear_m:
            best_safe = pass_best
        else:
            best_raw = pass_best
    # The safe-margin pass wins the choice; the raw pass only backs it up
    # when nothing is reachable with clearance. All candidates stay in
    # options either way, so the driver can show every alternative.
    best = best_safe if best_safe is not None else best_raw
    if best is None:
        return None
    # Chosen first, then alternatives by score. A cross-pass score sort put
    # a raw-clearance candidate (shorter route -> higher score) at index 0
    # while the published goal was the safe-pass winner: marker 0 on
    # /goal/options showed a wall-hugging route the brain never chose.
    options.sort(key=lambda o: (o is not best, -o['score']))
    best = dict(best)
    best['options'] = options[:max(1, int(max_options))]
    return best
