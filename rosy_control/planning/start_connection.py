"""Continuous, bounded connection from an off-centre pose to the grid."""
import math


def _distance(point, start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    length2 = dx*dx+dy*dy
    t = 0. if length2 == 0 else max(0., min(1., (
        (point[0]-start[0])*dx+(point[1]-start[1])*dy)/length2))
    return math.hypot(point[0]-start[0]-t*dx, point[1]-start[1]-t*dy)


def swept_clear(m, start, end, clearance):
    """Same occupied-centre radius as grid inflation; unknown stays blocked.

    Unknown cell circumcircles conservatively contain their entire squares.
    This is used only for a short connector, never to clear or rewrite cells.
    """
    if not all(math.isfinite(v) for v in (*start, *end, clearance)) or clearance <= 0:
        return False
    if any(not (m.ox+clearance <= p[0] <= m.ox+m.w*m.res-clearance
                and m.oy+clearance <= p[1] <= m.oy+m.h*m.res-clearance)
           for p in (start, end)):
        return False
    padding = clearance+m.res/math.sqrt(2)
    lo_c = max(0, math.floor((min(start[0],end[0])-padding-m.ox)/m.res))
    hi_c = min(m.w-1, math.floor((max(start[0],end[0])+padding-m.ox)/m.res))
    lo_r = max(0, math.floor((min(start[1],end[1])-padding-m.oy)/m.res))
    hi_r = min(m.h-1, math.floor((max(start[1],end[1])+padding-m.oy)/m.res))
    for r in range(lo_r, hi_r+1):
        for c in range(lo_c, hi_c+1):
            value = m.cell(c,r)
            if value < 0 or value >= 65:
                required = padding if value < 0 else clearance
                if _distance(m.grid_to_world(c,r), start, end) <= required:
                    return False
    return True


def start_connections(m, start, clearance, grid=None, avoid_points=()):
    """At most 24 local anchors, each joined by a checked straight segment."""
    if not all(math.isfinite(v) for v in (*start, clearance)) or clearance <= 0:
        return []
    if not swept_clear(m, start, start, clearance):
        return []
    grid = m.inflate(clearance/m.res) if grid is None else grid
    sc = m.world_to_grid(*start)
    anchors = []
    for dc in range(-2,3):
        for dr in range(-2,3):
            cell = sc[0]+dc, sc[1]+dr
            if cell == sc or not grid.is_free(*cell):
                continue
            point = m.grid_to_world(*cell)
            if any(_distance(p, start, point) <= .05 for p in avoid_points):
                continue
            if swept_clear(m, start, point, clearance):
                anchors.append((math.dist(start,point), cell))
    return sorted(anchors)


def start_clearance_diagnostic(m, pose, clearance):
    center = m.grid_to_world(*m.world_to_grid(*pose))
    walls = [m.grid_to_world(c,r) for r in range(m.h) for c in range(m.w) if m.cell(c,r)>=65]
    actual = min((math.dist(pose,p) for p in walls), default=math.inf)
    quantized = min((math.dist(center,p) for p in walls), default=math.inf)
    return (f'planning blocked: map start clearance (obstacle clearance); actual={actual*1000:.1f}mm '
            f'grid={quantized*1000:.1f}mm required={clearance*1000:.1f}mm; '
            'no safe local connection')


def connection_route_clear(m, points, clearance):
    """Check initial pursuit shortcuts, including the next point past 25 cm.

    The live follower uses 6 cm lookahead. Validating a longer prefix also
    covers its reusable pursuit helper's 25 cm default and corner skipping.
    """
    travelled = 0.
    for previous, point in zip(points, points[1:]):
        if not swept_clear(m, points[0], point, clearance):
            return False
        travelled += math.dist(previous, point)
        if travelled >= .25:
            break
    return True
