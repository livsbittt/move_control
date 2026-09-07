"""LiDAR nose angle derived from the base<-scan TF rotation."""
import math


def nose_from_quaternion(x, y, z, w):
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        raise ValueError('Non-finite LiDAR rotation')
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9:
        raise ValueError('Empty LiDAR rotation')
    x, y, z, w = (v / norm for v in (x, y, z, w))
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    # robot_yaw = scan_angle - nose; TF instead adds its rotation to scan_angle.
    return (-yaw) % (2 * math.pi)
