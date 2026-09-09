"""Fail-closed lidar bumper decisions, independent of ROS and smoothing."""
import math

from ..sensing.body import LIDAR_X, use_radius


def lidar_limits(stop, clear, radius):
    # Circumradius plus absolute sensor offset bounds every heading even
    # before the measured 190-degree mounting yaw is transformed. Add an
    # 18 mm stand-off: 76 + 17 + 18 = 111 mm, above the C1 50 mm blind zone.
    floor = use_radius(radius) + abs(LIDAR_X) + 0.018
    stop = max(floor, stop) if math.isfinite(stop) else floor
    clear = max(stop + 0.010, clear) if math.isfinite(clear) else stop + 0.010
    return stop, clear


def directional_lidar_limits(stop, clear, radius, mount=None, half_width=math.pi/4):
    """Circle-ray extent over each bumper cone, preserving configured margin."""
    conservative = lidar_limits(stop, clear, radius)
    if (mount is None or len(mount) != 2 or
            not all(math.isfinite(v) for v in mount) or
            math.hypot(*mount) >= use_radius(radius)):
        return conservative, conservative
    x, y = mount
    radius = use_radius(radius)
    def extent(heading):
        # Max ray length occurs at minimum mount projection onto the cone.
        angles = [heading-half_width, heading+half_width]
        opposite = math.atan2(-y, -x)
        if abs(math.atan2(math.sin(opposite-heading), math.cos(opposite-heading))) <= half_width:
            angles.append(opposite)
        projection = min(x*math.cos(a)+y*math.sin(a) for a in angles)
        return -projection + math.sqrt(radius*radius-x*x-y*y+projection*projection) + .018
    front, rear = extent(0.), extent(math.pi)
    front_stop = max(front, conservative[0])
    extra = front_stop-front
    hysteresis = conservative[1]-conservative[0]
    rear_stop = max(.05+.018, rear+extra)
    return (front_stop, front_stop+hysteresis), (rear_stop, rear_stop+hysteresis)


def lidar_blocked(raw, filtered, was_blocked, stop, clear, fresh):
    # Missing echoes cannot release a bumper after a wall enters the blind
    # zone. A close raw beam brakes now; only clearing uses the smooth value.
    if not fresh or not math.isfinite(raw) or raw <= 0.0:
        return True
    if raw <= stop:
        return True
    if raw >= clear and math.isfinite(filtered) and filtered >= clear:
        return False
    return was_blocked


def lidar_can_rotate(ranges, radius, fresh, base_clearance=None, *, sweep_radius=None):
    # The body sweeps its circumradius when spinning. Unknown flank/rear
    # space is not permission to swing a corner into a wall.
    required = use_radius(radius)
    if sweep_radius is not None:
        if not math.isfinite(sweep_radius) or sweep_radius < required:
            return False
        required = sweep_radius  # Never clamp a measured swept envelope downward.
    if base_clearance is not None:
        return (fresh and bool(ranges) and
                all(math.isfinite(value) and value > 0 for value in ranges) and
                math.isfinite(base_clearance) and base_clearance > required+.010)
    limit = required + abs(LIDAR_X) + 0.010
    return fresh and bool(ranges) and all(
        math.isfinite(value) and value > limit for value in ranges)


def rotation_scan_observed(ranges, increment, minimum, maximum):
    """Unknown azimuths cannot be treated as empty swept space."""
    return (len(ranges) >= 180 and all(math.isfinite(v) for v in (increment,minimum,maximum)) and
            0 <= minimum < maximum and abs(len(ranges)*abs(increment)-2*math.pi) < .02 and
            all(math.isfinite(r) and minimum < r <= maximum for r in ranges))


def scan_body_clearance(ranges, angle_min, increment, x, y, yaw, minimum, maximum):
    """Nearest observed obstacle to base_link, including the mount translation."""
    nearest = math.inf
    for i,r in enumerate(ranges):
        if math.isfinite(r) and minimum < r <= maximum:
            a = angle_min+i*increment+yaw
            nearest = min(nearest, math.hypot(x+r*math.cos(a),y+r*math.sin(a)))
    return nearest
