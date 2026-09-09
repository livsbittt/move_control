"""Subject: transient observed obstacles on a private planning-grid copy."""
import math
from .gridmap import OccupancyMap
from ..sensing.obstacle_tracks import transform_points


def obstacle_overlay(original, tracks, odom_to_map, padding=0.):
    if not math.isfinite(padding) or padding < 0:
        raise ValueError('Finite nonnegative padding required')
    out = OccupancyMap(original.w, original.h, original.res, (original.ox, original.oy))
    out.data = list(original.data)
    for track in tracks:
        x, y = transform_points([track['position']], odom_to_map)[0]
        radius = float(track['radius'])
        if not all(math.isfinite(v) for v in (x, y, radius)) or radius < 0:
            raise ValueError('Finite obstacle extent required')
        radius += padding
        # Cover every cell touched by the observed footprint. The planner adds
        # calibrated robot clearance later, so do not inflate the body twice.
        c0 = math.floor((x-radius-out.ox)/out.res)
        c1 = math.floor((x+radius-out.ox)/out.res)
        r0 = math.floor((y-radius-out.oy)/out.res)
        r1 = math.floor((y+radius-out.oy)/out.res)
        for r in range(max(0, r0), min(out.h-1, r1)+1):
            for c in range(max(0, c0), min(out.w-1, c1)+1):
                out.set_cell(c, r, 100)
    return out
