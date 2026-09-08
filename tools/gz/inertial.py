"""Gravity-only simulated accelerometer in the actual body orientation."""
import math


def body_gravity(quaternion):
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError('Finite attitude quaternion required')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if not .9 <= norm <= 1.1:
        raise ValueError('Unit attitude quaternion required')
    x, y, z, w = (v/norm for v in quaternion)
    return (9.81*2*(x*z-w*y), 9.81*2*(y*z+w*x), 9.81*(1-2*(x*x+y*y)))
