"""Onboard C1 scan geometry. One heading for safety, LCD, calib, web.

rplidar_link is yaw=π vs base_link, so scan 0 is the back.
lidar_yaw_offset is the scan-frame angle of the nose.
Live US↔lidar (nose ±40deg, close wall): best heading ≈ 180deg, not 165.
A HUD that looked −15deg off was LCD polar L/R (now −sin, left=left).
Robot yaw 0 = front, +CCW = left:  wrap(scan_angle - lidar_yaw_offset).
"""
import math

# +CCW from back. 10deg from π so the front cone matches the nose sonar.
MOUNT_YAW_DEG = 10.0
NOSE_YAW = math.pi + math.radians(MOUNT_YAW_DEG)


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def is_robot_scan(msg) -> bool:
    """Keep the local C1. Drop remote gazebo /scan on the same domain.

    Local DenseBoost: ~720 beams, range_max 40 m, wall-clock stamp.
    Remote parameter_bridge: ~640 beams, range_max 12 m, sim-time stamp.
    """
    try:
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
    except Exception:
        return False
    if t < 1.0e9:
        return False
    n = len(getattr(msg, 'ranges', ()) or ())
    if n < 500:
        return False
    rmax = float(msg.range_max) if msg.range_max > 0.0 else 0.0
    if 0.0 < rmax < 20.0:
        return False
    return True


def robot_yaw(scan_angle: float, yaw_offset: float = NOSE_YAW, mirror: bool = False) -> float:
    yaw = scan_angle - yaw_offset
    if mirror:
        yaw = -yaw
    return wrap_pi(yaw)


def _sector_hits(scan, heading: float, half_width: float, ignore_below: float, max_r: float):
    angle = float(scan.angle_min)
    inc = float(scan.angle_increment)
    hi = float(scan.range_max) if scan.range_max > 0.0 else 12.0
    if max_r > 0.0:
        hi = min(hi, float(max_r))
    lo = max(0.0, float(ignore_below))
    hits = []
    for r in scan.ranges:
        if math.isfinite(r) and lo < r < hi:
            rel = wrap_pi(angle - heading)
            if abs(rel) <= half_width:
                hits.append((rel, r))
        angle += inc
    return hits


def sector_range(
    scan,
    heading: float,
    half_width: float,
    pctl: float = 0.10,
    ignore_below: float = 0.04,
    max_r: float = 12.0,
) -> float:
    """Stable range in a cone. 10th percentile kills single-beam spikes."""
    hits = _sector_hits(scan, heading, half_width, ignore_below, max_r)
    if not hits:
        return float('inf')
    xs = sorted(r for _rel, r in hits)
    if len(xs) == 1:
        return xs[0]
    p = min(0.49, max(0.0, float(pctl)))
    return xs[int(round(p * (len(xs) - 1)))]


def sector_min(scan, heading: float, half_width: float) -> float:
    return sector_range(scan, heading, half_width, pctl=0.0)


def find_frontiers(
    scan,
    yaw_offset=NOSE_YAW,
    mirror=False,
    occ=0.16,
    free=0.22,
    max_r=0.45,
    min_width=math.radians(10.0),
    front_half=math.radians(110.0),
):
    """Open-space edges next to occupied walls. Yaw is robot frame (+left)."""
    if scan is None or not getattr(scan, 'ranges', None):
        return []
    if not is_robot_scan(scan):
        return []
    ang = float(scan.angle_min)
    inc = float(scan.angle_increment)
    hi = float(scan.range_max) if scan.range_max > 0.0 else 12.0
    lo = 0.04
    occ = max(0.06, float(occ))
    free = max(occ + 0.02, float(free))
    cap = max(free, float(max_r))
    samples = []
    for r in scan.ranges:
        yaw = robot_yaw(ang, yaw_offset, mirror)
        if abs(yaw) <= front_half:
            if math.isfinite(r) and lo < r < hi:
                samples.append((yaw, min(r, cap)))
            else:
                samples.append((yaw, cap))
        ang += inc
    if len(samples) < 8:
        return []
    samples.sort(key=lambda x: x[0])
    flags = []
    for _yaw, r in samples:
        flags.append('O' if r <= occ else 'F')
    out = []
    i = 0
    n = len(flags)
    while i < n:
        if flags[i] != 'F':
            i += 1
            continue
        j = i
        while j < n and flags[j] == 'F':
            j += 1
        left_occ = i > 0 and flags[i - 1] == 'O'
        right_occ = j < n and flags[j] == 'O'
        if not (left_occ or right_occ):
            i = j
            continue
        chunk = samples[i:j]
        width = abs(chunk[-1][0] - chunk[0][0])
        if width < min_width:
            i = j
            continue
        depths = [r for _y, r in chunk]
        depth = sorted(depths)[len(depths) // 2]
        sx = sy = 0.0
        for y, r in chunk:
            sx += math.cos(y) * r
            sy += math.sin(y) * r
        yaw = math.atan2(sy, sx) if (sx or sy) else 0.5 * (chunk[0][0] + chunk[-1][0])
        out.append({
            'yaw': yaw,
            'depth': depth,
            'width': width,
            'score': width * depth,
        })
        i = j
    out.sort(key=lambda f: f['score'], reverse=True)
    return out


def opening_max(scan, heading: float, half_width: float, max_r=1.20):
    """Farthest opening in the front fan. Yaw is mean of near-max hits. +yaw = left."""
    hits = _sector_hits(scan, heading, half_width, ignore_below=0.04, max_r=max_r)
    if not hits:
        return float('inf'), 0.0
    best_r = max(r for _rel, r in hits)
    near = [(rel, r) for rel, r in hits if r >= best_r * 0.90]
    sx = sy = 0.0
    for rel, r in near:
        sx += math.cos(rel) * r
        sy += math.sin(rel) * r
    if sx == 0.0 and sy == 0.0:
        return best_r, 0.0
    return best_r, math.atan2(sy, sx)
