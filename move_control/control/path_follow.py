"""Fail-closed map-route following for the slow desk robot."""
import math

from .pursuit import pursuit_index, pursuit_speed


def follow_path(route, pose, *, route_age, tf_age, blocked=False,
                max_age=5.0, max_tf_age=1.0, speed=.014, turn=.10,
                lookahead=.06, tolerance=.025, max_deviation=.08):
    """Return semantic (linear, angular, reason); no odometry fallback."""
    if blocked:
        return 0.0, 0.0, 'hazard'
    if not route:
        return 0.0, 0.0, 'no_route'
    if not math.isfinite(route_age) or not 0 <= route_age <= max_age:
        return 0.0, 0.0, 'stale_route'
    if pose is None or not math.isfinite(tf_age) or not 0 <= tf_age <= max_tf_age:
        return 0.0, 0.0, 'no_map_tf'
    if not all(math.isfinite(v) for p in route for v in p):
        return 0.0, 0.0, 'invalid_route'
    if not all(math.isfinite(v) for v in pose):
        return 0.0, 0.0, 'no_map_tf'
    x, y, yaw = pose
    deviation = math.hypot(route[0][0] - x, route[0][1] - y)
    for a, b in zip(route, route[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        fraction = 0.0 if length2 == 0 else max(0.0, min(
            1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length2))
        deviation = min(deviation, math.hypot(
            x - a[0] - fraction * dx, y - a[1] - fraction * dy))
    if deviation > max_deviation:
        return 0.0, 0.0, 'off_route'
    if math.hypot(route[-1][0] - x, route[-1][1] - y) <= tolerance:
        return 0.0, 0.0, 'arrived'
    aim = route[pursuit_index(route, x, y, lookahead)]
    distance = math.hypot(aim[0] - x, aim[1] - y)
    error = math.atan2(aim[1] - y, aim[0] - x) - yaw
    error = math.atan2(math.sin(error), math.cos(error))
    linear = pursuit_speed(max(0.0, speed), distance, error)
    angular = max(-abs(turn), min(abs(turn), 1.4 * error))
    return linear, angular, 'forward' if linear > 0 else 'align'


class ProgressGuard:
    """Latch a stop after commanded motion produces no measured progress."""
    def __init__(self, timeout=8.0):
        self.timeout = timeout
        self.reset()

    def reset(self):
        self.anchor = None
        self.since = None
        self.stalled = False

    def check(self, now, pose, moving):
        if self.stalled:
            return True
        if not moving or pose is None:
            # Empty replans / obstacle holds must not replenish the push budget.
            # Only measured progress or an explicit new command resets it.
            return False
        if self.anchor is None:
            self.anchor, self.since = pose, now
            return False
        distance = math.hypot(pose[0] - self.anchor[0], pose[1] - self.anchor[1])
        angle = pose[2] - self.anchor[2]
        angle = abs(math.atan2(math.sin(angle), math.cos(angle)))
        if distance >= .005 or angle >= .05:
            self.anchor, self.since = pose, now
        elif now - self.since >= self.timeout:
            self.stalled = True
        return self.stalled
