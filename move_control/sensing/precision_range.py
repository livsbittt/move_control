"""Subject: forward-axis wall distance for short calibration, never braking."""
import math
from statistics import median

from .lidar import robot_yaw


def precision_axis_range(scan, nose):
    """Intersect a continuous local wall x=a*y+b with the robot's forward axis.

    Cone percentiles measure an off-axis ray, whose change is not forward
    travel beside an oblique wall. Reject ambiguous surfaces rather than
    interpreting their edge or an isolated return as a speed correction.
    """
    points = []
    for i, value in enumerate(scan.ranges):
        angle = robot_yaw(scan.angle_min + i * scan.angle_increment, nose)
        if abs(angle) > math.radians(4) + 1e-9:
            continue
        if not math.isfinite(value) or not max(.05, scan.range_min) < value < min(8., scan.range_max):
            return math.inf
        points.append((angle, value * math.sin(angle), value * math.cos(angle)))
    points.sort()
    if len(points) < 3 or points[0][0] >= 0 or points[-1][0] <= 0:
        return math.inf
    # A gap at the axis could be a doorway; never interpolate across it.
    if any(b[0] - a[0] > math.radians(2) + 1e-9 for a, b in zip(points, points[1:])):
        return math.inf
    slopes = [(b[2] - a[2]) / (b[1] - a[1])
              for i, a in enumerate(points) for b in points[i+1:]
              if abs(b[1] - a[1]) > 1e-6]
    if not slopes:
        return math.inf
    slope = median(slopes)
    intercept = median(x - slope*y for _, y, x in points)
    # Median fit prevents a single outlier distorting the model, but every
    # return must still support it: fitting one of two walls is not evidence.
    if abs(slope) > 2.5 or any(abs(x - slope*y - intercept) > .003 for _, y, x in points):
        return math.inf
    return intercept if math.isfinite(intercept) and .05 < intercept < 8. else math.inf
