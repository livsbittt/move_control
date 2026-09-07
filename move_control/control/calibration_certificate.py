"""Versioned evidence for reusing a completed calibration, never live health."""
import hashlib
import json
import math


def _json_copy(value):
    return json.loads(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False))


def _fingerprint(configuration):
    if not isinstance(configuration, dict):
        raise ValueError('Calibration configuration must be an object')
    encoded = json.dumps(configuration, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(encoded.encode('utf-8')).hexdigest()


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _motion_valid(motion):
    if not isinstance(motion, dict) or motion.get('done') is not True or motion.get('error') is not None:
        return False
    for key in ('forward_scale', 'reverse_scale'):
        value = motion.get(key)
        if not _number(value) or not .75 <= value <= 1.25:
            return False
    legs = motion.get('legs')
    if not isinstance(legs, list) or len(legs) != 4:
        return False
    for index, leg in enumerate(legs):
        if not isinstance(leg, dict) or leg.get('direction') != ('forward' if index % 2 == 0 else 'reverse'):
            return False
        if leg.get('cycle') != index//2 or isinstance(leg.get('cycle'), bool):
            return False
        measured, odom, home = (leg.get(key) for key in ('measured_m', 'odom_m', 'home_error_m'))
        if not all(_number(v) for v in (measured, odom, home)):
            return False
        if not .018 <= measured <= .05 or abs(odom-measured) > .012:
            return False
        commanded = leg.get('commanded_m')
        if not _number(commanded) or commanded <= 0:
            return False
    return abs(legs[-1]['home_error_m']) <= .006


def make_certificate(configuration: dict, motion_report: dict) -> dict:
    """Raise ValueError rather than certify incomplete or nonfinite evidence."""
    try:
        motion = _json_copy(motion_report)
        fingerprint = _fingerprint(configuration)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError('Calibration evidence must be finite JSON') from exc
    if not _motion_valid(motion):
        raise ValueError('A complete bounded four-leg calibration is required')
    return {'schema': 1, 'configuration_fingerprint': fingerprint, 'motion': motion}


def validate_certificate(record, configuration) -> dict | None:
    """Return an independent evidence copy; callers must recheck live health."""
    try:
        record = _json_copy(record)
        if (not isinstance(record, dict) or type(record.get('schema')) is not int
                or record['schema'] != 1
                or record.get('configuration_fingerprint') != _fingerprint(configuration)):
            return None
        motion = _json_copy(record.get('motion'))
        return motion if _motion_valid(motion) else None
    except (ValueError, TypeError, OverflowError):
        return None
