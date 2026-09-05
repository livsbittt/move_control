"""Subject: occupancy grid. OccupancyGrid-shaped map with transforms.

Pure logic (no ROS imports) so the unit tests cover it.

Grid like nav_msgs/OccupancyGrid: (0,0) bottom-left, data[r*w + c].
Values -1 unknown / 0..64 free-ish / >=65 occupied (map_saver threshold).
World = origin + (c+0.5)*res.

Route safety: inflation grows only from OCC, never from UNKNOWN. Frontier
cells sit right against unknown space, so inflating unknown would close every
frontier. Unknown still blocks A* by absence (only known-free traverses), so
a route never crosses unmapped space.
"""
from collections import deque

FREE = 0
OCC = 100
UNKNOWN = -1

# Occupancy >= this counts as a wall (map_saver threshold).
OCC_THRESH = 65


class OccupancyMap:
    """SLAM grid with world<->grid transforms, flood, inflate, ASCII render."""

    def __init__(self, w, h, res=0.05, origin=(0.0, 0.0), fill=UNKNOWN):
        self.w = int(w)
        self.h = int(h)
        self.res = float(res)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.data = [fill] * (self.w * self.h)

    @classmethod
    def from_msg(cls, msg):
        info = msg.info
        m = cls(info.width, info.height, info.resolution,
                (info.origin.position.x, info.origin.position.y))
        m.data = list(msg.data)
        return m

    def world_to_grid(self, x, y):
        return _clamp_cell(self, (int((x - self.ox) / self.res),
                                  int((y - self.oy) / self.res)))

    def grid_to_world(self, c, r):
        return (self.ox + (c + 0.5) * self.res,
                self.oy + (r + 0.5) * self.res)

    def cell(self, c, r):
        if not (0 <= c < self.w and 0 <= r < self.h):
            return OCC  # outside the map is a wall
        return self.data[r * self.w + c]

    def is_free(self, c, r):
        return 0 <= self.cell(c, r) < OCC_THRESH

    def set_cell(self, c, r, v):
        if 0 <= c < self.w and 0 <= r < self.h:
            self.data[r * self.w + c] = v

    def free_cells(self):
        return [(c, r) for r in range(self.h) for c in range(self.w)
                if self.is_free(c, r)]

    def flood(self, start, max_d=None):
        """8-connected flood over free cells. Wall/unknown/outside stop it."""
        sc = _clamp_cell(self, start)
        if not self.is_free(*sc) or (max_d is not None and max_d < 0):
            return set()
        out = {sc}
        q = deque((sc,))
        d = {sc: 0}
        while q:
            c, r = q.popleft()
            if max_d is not None and d[(c, r)] >= max_d:
                continue
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    if dc == 0 and dr == 0:
                        continue
                    nc = (c + dc, r + dr)
                    if nc not in out and self.is_free(*nc):
                        out.add(nc)
                        d[nc] = d[(c, r)] + 1
                        q.append(nc)
        return out

    def inflate(self, r_cells):
        """Grow OCC by a Euclidean ring. Unknown stays unknown, walls grow."""
        out = OccupancyMap(self.w, self.h, self.res, (self.ox, self.oy), fill=UNKNOWN)
        # Pass 1: classify. Pass 2: grow rings. One loop would let later
        # FREE cells stomp on earlier rings (unit test caught exactly that).
        for i, v in enumerate(self.data):
            if v >= OCC_THRESH:
                out.data[i] = OCC
            elif v == UNKNOWN:
                out.data[i] = UNKNOWN
            else:
                out.data[i] = FREE
        for i, v in enumerate(self.data):
            if v < OCC_THRESH:
                continue
            c, r = i % self.w, i // self.w
            for dc in range(-r_cells, r_cells + 1):
                for dr in range(-r_cells, r_cells + 1):
                    if dc * dc + dr * dr <= r_cells * r_cells:
                        out.set_cell(c + dc, r + dr, OCC)
        return out

    def render(self, marks=None, unknown_char='~'):
        """ASCII, top row = r=h-1. marks: {(c,r): char} wins over the map."""
        lines = []
        for r in range(self.h - 1, -1, -1):
            row = []
            for c in range(self.w):
                ch = marks.get((c, r)) if marks else None
                if ch:
                    row.append(ch)
                    continue
                v = self.data[r * self.w + c]
                row.append('#' if v >= OCC_THRESH else ('.' if v >= 0 else unknown_char))
            lines.append(''.join(row))
        return '\n'.join(lines)


def _clamp_cell(m, cell):
    c = max(0, min(m.w - 1, int(cell[0])))
    r = max(0, min(m.h - 1, int(cell[1])))
    return (c, r)


def _snap(m, cell, max_n=4096, max_occ=0):
    """Nearest FREE cell. BFS transits unknown freely; max_occ inflated-wall
    cells lets a near-wall goal escape the safety ring on an inflated map,
    while a goal buried in a wall (max_occ exhausted) stays None."""
    cell = _clamp_cell(m, cell)
    if m.is_free(*cell):
        return cell
    seen = {cell}
    q = deque(((cell, 0),))
    n = 0
    while q and n < max_n:
        n += 1
        (c, r), occ_n = q.popleft()
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if dc == 0 and dr == 0:
                    continue
                nc = (c + dc, r + dr)
                if nc in seen:
                    continue
                seen.add(nc)
                if m.is_free(*nc):
                    return nc
                if m.cell(*nc) == OCC:
                    if occ_n >= max_occ:
                        continue
                    q.append((nc, occ_n + 1))
                else:
                    q.append((nc, occ_n))
    return None
