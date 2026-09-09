"""Calibration consumes the final gate's current rotation clearance decision."""
import math


def calibration_rotation_clearance(ranges, increment, scan_age, gate, gate_age, geometry_revision):
    finite = [math.isfinite(value) and value > 0 for value in ranges]
    report = {'clear': False, 'reason': 'invalid_scan', 'scan_age_s': scan_age,
              'gate_age_s': gate_age, 'missing_bins': len(ranges)-sum(finite),
              'fully_observed': bool(ranges) and all(finite), 'maximum_gap_deg': None,
              'nearest_range_m': min((v for v, ok in zip(ranges, finite) if ok), default=None),
              'authority': 'safety_motion_limits'}
    if (not 360 <= len(ranges) <= 1440 or not math.isfinite(increment) or increment <= 0 or
            abs(len(ranges)*increment-math.tau) >= .02 or not any(finite)):
        return report
    longest = run = 0
    for valid in finite+finite:
        run = 0 if valid else run+1
        longest = max(longest, min(run, len(ranges)))
    report['maximum_gap_deg'] = math.degrees(min(math.tau, (longest+1)*increment))
    if not math.isfinite(scan_age) or not 0 <= scan_age <= .25:
        report['reason'] = 'stale_scan'
    elif not math.isfinite(gate_age) or not 0 <= gate_age <= .25:
        report['reason'] = 'stale_safety_motion_limits'
    elif not isinstance(gate, dict) or not geometry_revision or gate.get('geometry_revision') != geometry_revision:
        report['reason'] = 'safety_geometry_mismatch'
    elif gate.get('can_rotate') is not True:
        report['reason'] = 'safety_rotation_clearance_blocked'
    else:
        # No synthetic return replaces a missing ray. The body/sector guard
        # remains authoritative; scan-alignment observability is a separate
        # prerequisite checked before the adapter issues a calibration turn.
        report.update(clear=True, reason='current_safety_gate_clearance')
    return report
