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


def make_certificate(configuration: dict, motion_report: dict, rotation_report=None, geometry_revision=None) -> dict:
    """Raise ValueError rather than certify incomplete or nonfinite evidence."""
    try:
        motion = _json_copy(motion_report)
        fingerprint = _fingerprint(configuration)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError('Calibration evidence must be finite JSON') from exc
    if not _motion_valid(motion):
        raise ValueError('A complete bounded four-leg calibration is required')
    record = {'schema': 1, 'configuration_fingerprint': fingerprint, 'motion': motion}
    if rotation_report is not None:
        rotation = _json_copy(rotation_report)
        if not _rotation_valid(rotation):
            raise ValueError('Complete independently repeated bilateral rotation evidence required')
        record.update(schema=2, rotation=rotation, rotation_identity={
            'configuration_fingerprint': fingerprint, 'evidence_sha256': _fingerprint(rotation)})
    if geometry_revision is not None:
        if not isinstance(geometry_revision, str) or not geometry_revision:
            raise ValueError('Safety geometry identity must be nonempty')
        record['safety_geometry_revision'] = geometry_revision
    return record


def validate_certificate(record, configuration) -> dict | None:
    """Return an independent evidence copy; callers must recheck live health."""
    try:
        record = _json_copy(record)
        if (not isinstance(record, dict) or type(record.get('schema')) is not int
                or record['schema'] not in (1, 2)
                or record.get('configuration_fingerprint') != _fingerprint(configuration)):
            return None
        if 'safety_geometry_revision' in record and (not isinstance(record['safety_geometry_revision'], str) or not record['safety_geometry_revision']):
            return None
        if record['schema'] == 2:
            rotation = record.get('rotation')
            if (not _rotation_valid(rotation) or record.get('rotation_identity') != {
                    'configuration_fingerprint': _fingerprint(configuration),
                    'evidence_sha256': _fingerprint(rotation)}):
                return None
        elif 'rotation' in record:
            return None  # A legacy translation certificate cannot claim rotation.
        motion = _json_copy(record.get('motion'))
        return motion if _motion_valid(motion) else None
    except (ValueError, TypeError, OverflowError):
        return None


def _rotation_valid(rotation):
    if (not isinstance(rotation, dict) or rotation.get('done') is not True or
            rotation.get('error') is not None or rotation.get('max_angular_rad_s') != .06 or
            rotation.get('geometry_commissioned') is not False):
        return False
    gains, legs = rotation.get('angular_gains'), rotation.get('legs')
    if not isinstance(gains, list) or len(gains) != 2 or not all(_number(v) and .75 <= v <= 1.25 for v in gains):
        return False
    if not isinstance(legs, list) or len(legs) != 8:
        return False
    for i, leg in enumerate(legs):
        if not isinstance(leg, dict) or type(leg.get('direction')) is not int or leg['direction'] != (1,-1,-1,1)[i%4]:
            return False
        measured, commanded, ratio = [leg.get(k) for k in ('measured_rad','commanded_rad','ratio')]
        if not all(_number(v) for v in (measured,commanded,ratio)):
            return False
        if not math.radians(7) <= measured <= math.radians(13) or not .75 <= ratio <= 1.25:
            return False
        if abs(commanded/measured-ratio) > 1e-9:
            return False
        if i >= 4 and abs(ratio-gains[int(leg['direction'] < 0)]) > .12:
            return False
    for direction, gain in zip((1, -1), gains):
        observed = [leg['ratio'] for leg in legs[:4] if leg['direction'] == direction]
        if max(observed)-min(observed) > .12 or abs(sum(observed)/len(observed)-gain) > 1e-9:
            return False
    return True
