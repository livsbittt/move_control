"""Reserve actual motion travel against the safety gate's metric limits."""
import math


def motion_clearance(limits, requested, target=None, forward=0., round_trip=True):
    keys = ('front_m','rear_m','front_stop_m','rear_stop_m','us_stop_m')
    if not all(isinstance(limits.get(k), (int, float)) and math.isfinite(limits[k]) and limits[k] > 0 for k in keys):
        return {'reason': 'Missing valid safety distance limits', 'target_m': None}
    if not math.isfinite(requested) or not .02 <= requested <= .04:
        return {'reason': 'Round-trip target must be 2 to 4 cm', 'target_m': None}
    if not math.isfinite(forward) or (target is not None and (not math.isfinite(target) or not .02 <= target <= .04)):
        return {'reason': 'Invalid motion travel evidence', 'target_m': None}
    us = limits.get('us_m', math.nan)
    if not isinstance(us, (int, float)) or not math.isfinite(us) or us <= 0:
        return {'reason': 'Missing valid ultrasonic guard range', 'target_m': None}
    margin = .008  # Existing maximum negative home excursion; also reserve stopping headroom.
    room = min(limits['front_m']-limits['front_stop_m'], us-limits['us_stop_m'])-margin
    selected = max(0., min(requested, math.floor((room+1e-9)*1000)/1000)) if target is None else target
    remaining = max(0., selected-forward)
    required = limits['front_stop_m']+remaining+margin
    rear_required = limits['rear_stop_m']+margin
    reason = None
    if selected < .02:
        reason = 'Insufficient clearance for minimum 2 cm motion evidence'
    elif limits['front_m'] < required-1e-6 or us < limits['us_stop_m']+remaining+margin-1e-6:
        reason = 'Insufficient remaining forward travel clearance'
    elif round_trip and limits['rear_m'] < rear_required:
        reason = 'Insufficient rear clearance for bounded return'
    return dict(available_front_m=limits['front_m'], required_front_m=required,
                available_rear_m=limits['rear_m'], required_rear_m=rear_required,
                front_stop_m=limits['front_stop_m'], rear_stop_m=limits['rear_stop_m'],
                available_us_m=us, required_us_m=limits['us_stop_m']+remaining+margin,
                target_m=selected, reason=reason)
