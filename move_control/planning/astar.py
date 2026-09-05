"""Subject: best route. A* on the inflated grid, world-metre interface.

8-connected, no corner cutting (a diagonal step needs both orthogonal
neighbours free). Unknown never traversable; inflation grows only from OCC,
so frontier free cells stay reachable. Returns world points + length.
"""
import heapq
import math

from .gridmap import FREE, nearest_free

_STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1),
          (1, 1), (1, -1), (-1, 1), (-1, -1))


def best_route(m, start, goal, clear_m=0.06):
    """Best route start->goal (world metres in). None if walled off.

    clear_m inflates walls so the 5 cm grid can't hug corners (robot ~15 cm
    wide). Inflation from OCC only; unknown blocks by absence.
    """
    grid = m.inflate(int(round(clear_m / m.res)))
    sc = nearest_free(grid, m.world_to_grid(*start), max_occ=2)
    gc = nearest_free(grid, m.world_to_grid(*goal), max_occ=2)
    if sc is None or gc is None:
        return None
    g = {sc: 0.0}
    came = {}
    closed = set()
    heap = [(math.hypot(gc[0] - sc[0], gc[1] - sc[1]), 0, sc)]
    cnt = 0
    while heap:
        _f, _n, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == gc:
            cells = _walk(came, cur)
            return {
                'points': [m.grid_to_world(c, r) for c, r in cells],
                'cells': cells,
                'length': g[cur] * m.res,
            }
        for dc, dr in _STEPS:
            nx = (cur[0] + dc, cur[1] + dr)
            if grid.cell(*nx) != FREE:
                continue
            if dc and dr:
                a = (cur[0] + dc, cur[1])
                b = (cur[0], cur[1] + dr)
                if grid.cell(*a) != FREE or grid.cell(*b) != FREE:
                    continue  # no corner cutting
            ng = g[cur] + math.hypot(dc, dr)
            if nx not in closed and ng < g.get(nx, 1e9):
                g[nx] = ng
                came[nx] = cur
                cnt += 1
                heapq.heappush(
                    heap,
                    (ng + math.hypot(gc[0] - nx[0], gc[1] - nx[1]), cnt, nx))
    return None


def _walk(came, cur):
    cells = [cur]
    while cur in came:
        cur = came[cur]
        cells.append(cur)
    cells.reverse()
    return cells
