"""Subject: best route. A* on the inflated grid, world-metre interface.

8-connected, no corner cutting (a diagonal step needs both orthogonal
neighbours free). Unknown never traversable; inflation grows only from OCC,
so frontier free cells stay reachable. Returns world points + length.
"""
import heapq
import math

from .gridmap import FREE, nearest_free
from .start_connection import start_connections, connection_route_clear

_STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1),
          (1, 1), (1, -1), (-1, 1), (-1, -1))


def best_route(m, start, goal, clear_m=0.06, avoid_points=()):
    """Best route start->goal (world metres in). None if walled off.

    clear_m inflates walls so the 5 cm grid can't hug corners (robot ~15 cm
    wide). Inflation from OCC only; unknown blocks by absence.
    """
    if not (m.ox <= start[0] < m.ox + m.w * m.res and
            m.oy <= start[1] < m.oy + m.h * m.res):
        return None
    grid = m.inflate(clear_m / m.res)
    sc = m.world_to_grid(*start)
    # Execution failures are temporary path exclusions, not fake SLAM walls.
    avoided = {cell for cell in m.free_cells() if cell != sc and any(
        math.dist(m.grid_to_world(*cell), point) <= .05 for point in avoid_points)} if avoid_points else set()
    connected = not grid.is_free(*sc)
    seeds = start_connections(m, start, clear_m, grid, avoid_points) if connected else [(0.,sc)]
    if not seeds:
        return None
    gc = nearest_free(grid, m.world_to_grid(*goal), max_occ=2)
    if sc is None or gc is None:
        return None
    g = {cell: distance/m.res for distance,cell in seeds}
    came = {}
    closed = set()
    heap = [(cost+math.hypot(gc[0]-cell[0],gc[1]-cell[1]), i, cell)
            for i,(cell,cost) in enumerate(g.items())]
    heapq.heapify(heap)
    cnt = len(heap)
    while heap:
        _f, _n, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == gc:
            cells = _walk(came, cur)
            points = ([tuple(start)] if connected else []) + [m.grid_to_world(c, r) for c, r in cells]
            if connected and not connection_route_clear(m, points, clear_m):
                return None
            return {
                'points': points,
                'cells': cells,
                'length': g[cur] * m.res,
                'clearance_m': clear_m,
            }
        for dc, dr in _STEPS:
            nx = (cur[0] + dc, cur[1] + dr)
            if grid.cell(*nx) != FREE or nx in avoided:
                continue
            if dc and dr:
                a = (cur[0] + dc, cur[1])
                b = (cur[0], cur[1] + dr)
                if grid.cell(*a) != FREE or grid.cell(*b) != FREE or a in avoided or b in avoided:
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
