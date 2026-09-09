"""Subject: straight-line route. Avoid walls. Not a circular path."""
import math

from ..sensing.lidar import NOSE_YAW, is_robot_scan, sector_range, wrap_pi


def line_route(
    scan,
    yaw_offset=NOSE_YAW,
    occ=0.12,
    max_r=0.45,
    cone_deg=8.0,
    front_half=math.radians(80.0),
    step_deg=5.0,
):
    """Longest free straight line that does not hit a wall.

    yaw is robot frame (0 = nose, +left). length is metres along that line.
    Headings with min range <= occ are rejected (wall / obstacle).
    """
    if scan is None or not getattr(scan, 'ranges', None):
        return None
    if not is_robot_scan(scan):
        return None
    occ = max(0.06, float(occ))
    cap = max(occ + 0.04, float(max_r))
    cone = math.radians(float(cone_deg))
    best = None
    deg = -math.degrees(front_half)
    end = math.degrees(front_half)
    while deg <= end + 0.01:
        yaw = math.radians(deg)
        heading = wrap_pi(yaw_offset + yaw)
        d = sector_range(scan, heading, cone, pctl=0.10, ignore_below=0.04, max_r=cap)
        if math.isfinite(d) and d > occ:
            # Prefer ahead: a side alley must be clearly longer to win.
            score = d * (0.40 + 0.60 * math.cos(yaw))
            if best is None or score > best[0]:
                best = (score, yaw, min(d, cap))
        deg += step_deg
    if best is None:
        return None
    return {'yaw': best[1], 'length': best[2], 'score': best[0]}
