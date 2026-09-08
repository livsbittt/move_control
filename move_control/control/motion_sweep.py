"""Conservative finite-command sweep for an explicitly commissioned simulation.

This geometry grants no motion authority. The caller supplies a fresh complete
scan in base coordinates, a matching pivot estimate, the final limited command,
and a horizon covering observation age, command hold and measured stopping.
It does not establish physical stopping behavior or authorize a full rotation.
"""
import math
from numbers import Real


def _finite_number(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def bounded_sweep_clearance(points, center, uncertainty, body_radius, v, w,
                            horizon, margin=.010):
    """Signed obstacle clearance from a continuous, bounded swept body circle.

    The planar base trajectory is (I-R)c + integral(R(v,0) dt). A circle of
    body_radius + 2*uncertainty covers the trusted body and pivot discrepancy.
    Uniform samples include both endpoints; speed_bound*dt/2 dilation covers
    every intermediate center because its nearest time sample is at most
    dt/2 away. Positive clearance means all supplied returns lie outside this
    conservative sweep. Unknown, malformed or out-of-domain input returns None.
    """
    try:
        cx, cy = center
        scalars = (cx, cy, uncertainty, body_radius, v, w, horizon, margin)
        if (not all(_finite_number(value) for value in scalars) or
                not 0. <= uncertainty <= .03 or body_radius <= 0. or
                abs(v) > .014 or abs(w) > .1 or not .75 <= horizon <= 1.5 or
                margin < .010):
            return None
        obstacles = []
        for x, y in points:
            if not _finite_number(x) or not _finite_number(y):
                return None
            obstacles.append((float(x), float(y)))
        if len(obstacles) < 3:
            return None
        intervals = math.ceil(horizon/.05)
        dt = horizon/intervals
        speed_bound = abs(v)+abs(w)*math.hypot(cx, cy)
        radius = body_radius+2*uncertainty+margin+speed_bound*dt/2
        if not math.isfinite(radius):
            return None
        nearest = math.inf
        for index in range(intervals+1):
            t = horizon*index/intervals
            angle = w*t
            sine = math.sin(angle)
            one_minus_cosine = 2*math.sin(angle/2)**2
            # Stable through w=0: v*t*sinc(angle), v*t*cosc(angle).
            sinc = sine/angle if angle else 1.
            cosc = one_minus_cosine/angle if angle else 0.
            x = one_minus_cosine*cx+sine*cy+v*t*sinc
            y = -sine*cx+one_minus_cosine*cy+v*t*cosc
            if not math.isfinite(x) or not math.isfinite(y):
                return None
            for px, py in obstacles:
                nearest = min(nearest, math.hypot(px-x, py-y))
        return nearest-radius if math.isfinite(nearest) else None
    except (TypeError, ValueError, OverflowError):
        return None
