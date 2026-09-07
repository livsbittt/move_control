"""Effective safety limits with a stable revision; no ROS or inferred geometry."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math

from ..sensing.body import LIDAR_X, URDF_RADIUS


@dataclass(frozen=True)
class SafetyProfile:
    radius: float
    stop: float
    clear: float
    turn_clear: float
    half_width_deg: float
    max_linear: float
    max_angular: float

    @classmethod
    def build(cls, *, radius=URDF_RADIUS, stop=.12, clear=.14, half_width_deg=45.,
              stop_floor=.12, clear_floor=.14, max_linear=.014, max_angular=.10):
        values = (radius, stop, clear, half_width_deg, stop_floor, clear_floor, max_linear, max_angular)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Non-finite safety profile')
        if not 0 < radius <= .15 or not 0 < max_linear <= .014 or not 0 < max_angular <= .10:
            raise ValueError('Profile exceeds nominal hardware envelope')
        if stop_floor < .12 or clear_floor < .14:
            raise ValueError('Bootstrap floors need commissioning before reduction')
        radius = max(URDF_RADIUS, radius)
        stop = max(radius+abs(LIDAR_X)+.018, stop_floor, stop)
        clear = max(stop+.010, clear_floor, clear)
        half = max(45., min(90., half_width_deg))
        return cls(radius, stop, clear, radius+abs(LIDAR_X)+.010, half, max_linear, max_angular)

    @property
    def revision(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()[:16]

    def report(self):
        return {'schema_version': 1, 'revision': self.revision,
                'source': 'nominal_bootstrap', 'commissioned': False,
                'limits_frame': 'lidar_origin', 'geometry_frame': 'base_link',
                'effective': asdict(self),
                'unverified': ['physical_geometry', 'stopping_response', 'rotational_response']}


def bounded_command(v, w, max_linear, max_angular):
    """Limit the pair together so saturation does not change curvature."""
    if not all(math.isfinite(x) for x in (v, w, max_linear, max_angular)) or min(max_linear, max_angular) <= 0:
        return 0., 0., 'invalid_command'
    factor = min(1., max_linear/abs(v) if v else 1., max_angular/abs(w) if w else 1.)
    return v*factor, w*factor, 'speed_limit' if factor < 1. else 'allow'
