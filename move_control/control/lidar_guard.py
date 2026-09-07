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


def lidar_can_rotate(ranges, radius, fresh):
    # The body sweeps its circumradius when spinning. Unknown flank/rear
    # space is not permission to swing a corner into a wall.
    limit = use_radius(radius) + abs(LIDAR_X) + 0.010
    return fresh and bool(ranges) and all(
        math.isfinite(value) and value > limit for value in ranges)
