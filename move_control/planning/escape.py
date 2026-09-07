"""Bounded start escape from preferred inflation, with a hard footprint floor."""
import math
from collections import deque
from .gridmap import OCC_THRESH


def start_escape(m, pose, preferred_m, minimum_m, max_distance_m, excluded=()):
    if not 0 < minimum_m < preferred_m or max_distance_m <= 0:
        return None
    if not (m.ox <= pose[0] < m.ox+m.w*m.res and m.oy <= pose[1] < m.oy+m.h*m.res):
        return None
    hard = m.inflate(minimum_m / m.res)
    soft = m.inflate(preferred_m / m.res)
    start = m.world_to_grid(*pose)
    if not hard.is_free(*start) or soft.is_free(*start):
        return None
    walls = [(c,r) for r in range(m.h) for c in range(m.w) if m.cell(c,r) >= OCC_THRESH]
    def clearance(cell):
        c,r = cell
        return min(((c-x)**2+(r-y)**2 for x,y in walls), default=math.inf)
    queue = deque([(start, [start])])
    seen = {start}
    offset = math.dist(pose, m.grid_to_world(*start))
    limit = int((max_distance_m-offset) / m.res + 1e-6)
    while queue:
        cell, path = queue.popleft()
        # The follower's 25mm arrival tolerance must not consume the escape.
        if (soft.is_free(*cell) and math.dist(pose, m.grid_to_world(*cell)) >= .04
                and all(math.dist(m.grid_to_world(*cell),goal)>.10 for goal in excluded)):
            points = [tuple(pose)] + [m.grid_to_world(*p) for p in path]
            return {'points': points, 'cells': path, 'length': offset+(len(path)-1)*m.res}
        if len(path)-1 >= limit:
            continue
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nxt = cell[0]+dx, cell[1]+dy
            # Never enter unknown, reduce wall clearance, or cut a corner.
            if nxt in seen or not hard.is_free(*nxt) or clearance(nxt) < clearance(cell):
                continue
            seen.add(nxt)
            queue.append((nxt,path+[nxt]))
    return None
