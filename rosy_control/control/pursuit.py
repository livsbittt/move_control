"""Subject: position-anchored route pursuit geometry."""
import math


def pursuit_speed(speed, distance, heading_error):
    """Align before advancing; a forward arc cuts tight maze corners."""
    if abs(heading_error) > 0.3:
        return 0.0
    return min(speed, 1.2 * distance) * max(0.0, math.cos(heading_error))


def pursuit_index(route, x, y, lookahead=0.25):
    """Choose an aim from measured position, preserving sharp corners."""
    nearest = min(range(len(route)),
                  key=lambda i: math.hypot(route[i][0] - x, route[i][1] - y))
    if 0 < nearest < len(route) - 1:
        px, py = route[nearest - 1]
        cx, cy = route[nearest]
        nx, ny = route[nearest + 1]
        bend = math.atan2(ny - cy, nx - cx) - math.atan2(cy - py, cx - px)
        incoming_length = math.hypot(cx-px, cy-py)
        remaining = -((x-cx)*(cx-px)+(y-cy)*(cy-py))/max(incoming_length, 1e-12)
        # Use progress along the incoming leg. Radial distance grows again
        # after turning and otherwise selects the corner behind us forever.
        # Five mm preserves the narrow route's rotation-clearance margin.
        if (remaining > 0.005
                and abs(math.atan2(math.sin(bend), math.cos(bend))) > 0.6):
            return nearest
    aim = nearest
    while aim + 1 < len(route):
        nx, ny = route[aim + 1]
        if aim > nearest:
            px, py = route[aim - 1]
            cx, cy = route[aim]
            bend = math.atan2(ny - cy, nx - cx) - math.atan2(cy - py, cx - px)
            if abs(math.atan2(math.sin(bend), math.cos(bend))) > 0.6:
                break
        aim += 1
        if math.hypot(nx - x, ny - y) >= lookahead:
            break
    return aim
