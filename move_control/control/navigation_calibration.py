"""Learn measurement margins, never robot dimensions, from a stationary scene."""
import math
from statistics import median


def environment_profile(radius, resolution, samples):
    if not (.02 <= radius <= .3 and .001 <= resolution <= .1):
        return None
    rows = [r for r in samples if all(math.isfinite(v) and .05 < v < 3 for v in r[:2])]
    if len(rows) < 20:
        return None
    sides = [sorted(r[i] for r in rows) for i in (0,1)]
    noise = max(col[int(.95*(len(col)-1))]-col[int(.05*(len(col)-1))] for col in sides)
    # A 10mm stand-off plus half a map cell and measured range variation.
    # Narrow surroundings never shrink this measurement-derived hard floor.
    margin = .010 + resolution/2 + noise
    if margin > .05:
        return None  # unstable environment must not silently clamp uncertainty
    minimum = radius + margin
    left, right = (median(c) for c in sides)
    width = left + right
    preferred = max(minimum, min(minimum+.03, width/2-noise))
    differences = []
    for i in (0,1):
        pairs = [r[i]-r[i+2] for r in rows if len(r)>i+2 and r[i+2] is not None]
        differences.append(median(pairs) if pairs else None)
    mismatch = any(v is not None and abs(v)>max(.02,resolution) for v in differences)
    return dict(body_radius_m=radius, map_resolution_m=resolution, samples=len(rows),
                left_m=left, right_m=right, corridor_width_m=width, range_noise_m=noise,
                uncertainty_margin_m=margin, minimum_clearance_m=minimum,
                preferred_clearance_m=preferred, passage_fits=width>2*minimum,
                map_lidar_difference_m=differences, map_lidar_mismatch=mismatch)


def map_ray(m, origin, angle, maximum=3.):
    """Distance to the first occupied cell; unknown is not a measured wall."""
    step = m.res/2
    distance = 0.
    while distance <= maximum:
        x,y = origin[0]+distance*math.cos(angle), origin[1]+distance*math.sin(angle)
        if not (m.ox<=x<m.ox+m.w*m.res and m.oy<=y<m.oy+m.h*m.res):
            return None
        value = m.cell(*m.world_to_grid(x,y))
        if value < 0:
            return None
        if value >= 65:
            return distance
        distance += step
    return None
