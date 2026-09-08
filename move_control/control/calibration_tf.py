"""TF evidence checks for bounded calibration recovery; no fabricated poses."""
import math


def transform_health(now, stamp, position, quaternion):
    finite = all(math.isfinite(v) for v in (now, stamp, *position, *quaternion))
    norm = math.sqrt(sum(v*v for v in quaternion)) if finite else None
    age = now-stamp if finite else None
    reason = ('invalid_geometry' if not finite or not .9 <= norm <= 1.1 else
              'future' if age < -.75 else 'stale' if age > 1. else 'ok')
    return dict(valid=reason == 'ok', reason=reason, source_age_s=age,
                source_stamp_s=stamp if math.isfinite(stamp) else None, quaternion_norm=norm)


def map_motion_continuous(evidence):
    errors = (evidence['map_forward_m']-evidence['forward_m'],
              evidence['map_lateral_m']-evidence['lateral_m'],
              evidence['map_yaw_drift_rad']-evidence['yaw_drift_rad'])
    return (all(math.isfinite(v) for v in errors) and
            abs(errors[0]) <= .015 and abs(errors[1]) <= .015 and
            abs(math.atan2(math.sin(errors[2]), math.cos(errors[2]))) <= .1)
