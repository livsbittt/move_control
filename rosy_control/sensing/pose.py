"""Finite map pose validation shared by route planning and execution."""
import math


def planar_pose(x, y, quaternion):
    if not all(math.isfinite(value) for value in (x, y, *quaternion)):
        return None
    norm = math.sqrt(sum(value*value for value in quaternion))
    if not .9 <= norm <= 1.1:
        return None
    qx, qy, qz, qw = (value/norm for value in quaternion)
    return x, y, math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
