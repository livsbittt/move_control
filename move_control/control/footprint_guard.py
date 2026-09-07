"""Straight-line clearance from observed points to a verified chassis box."""
import math


def translation_clearance(points, bounds, margin=.010, cap=.5):
    """Return travel before the inflated box touches an observed obstacle.

    Rotation is deliberately excluded. Unknown scan coverage must be rejected
    by the caller before using this result. Side obstacles inside the swept
    strip count even when outside the old 45-degree bumper cone.
    """
    rear, front, half_width = bounds
    if not all(math.isfinite(v) and v > 0 for v in (*bounds, margin, cap)):
        return None
    forward = reverse = cap
    front_seen = rear_seen = False
    for x, y in points:
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if abs(y) > half_width + margin:
            continue
        if -rear-margin <= x <= front+margin:
            return (0., 0.)
        if x > front+margin:
            front_seen = True
            forward = min(forward, x-front-margin)
        elif x < -rear-margin:
            rear_seen = True
            reverse = min(reverse, -rear-margin-x)
    return (forward, reverse) if front_seen and rear_seen else None
