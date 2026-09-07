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
    us_valid = isinstance(us, (int, float)) and math.isfinite(us) and us > 0
    if not us_valid and not limits.get('us_optional', False):
        return {'reason': 'Missing valid ultrasonic guard range', 'target_m': None}
    if not us_valid:
        us = None
    margin = .008  # Existing maximum negative home excursion; also reserve stopping headroom.
    footprint = limits.get('translation_mode') is True
    if footprint and not all(isinstance(limits.get(k), (int, float)) and
                             math.isfinite(limits[k]) and limits[k] >= 0
                             for k in ('forward_travel_m', 'reverse_travel_m')):
        return {'reason': 'Missing valid chassis travel clearance', 'target_m': None}
    front_room = limits['forward_travel_m'] if footprint else limits['front_m']-limits['front_stop_m']
    rear_room = limits['reverse_travel_m'] if footprint else limits['rear_m']-limits['rear_stop_m']
    room = min(front_room, us-limits['us_stop_m'] if us is not None else front_room)-margin
    selected = max(0., min(requested, math.floor((room+1e-9)*1000)/1000)) if target is None else target
    remaining = max(0., selected-forward)
    required = limits['front_stop_m']+remaining+margin
    rear_required = limits['rear_stop_m']+margin
    reason = None
    if selected < .02:
        reason = 'Insufficient clearance for minimum 2 cm motion evidence'
    elif front_room < remaining+margin-1e-6 or (us is not None and us < limits['us_stop_m']+remaining+margin-1e-6):
        reason = 'Insufficient remaining forward travel clearance'
    elif round_trip and rear_room < margin:
        reason = 'Insufficient rear clearance for bounded return'
    return dict(translation_mode=footprint, available_travel_m=front_room,
                required_travel_m=remaining+margin, reverse_travel_m=rear_room,
                required_reverse_travel_m=margin,
                available_front_m=limits['front_m'], required_front_m=required,
                available_rear_m=limits['rear_m'], required_rear_m=rear_required,
                front_stop_m=limits['front_stop_m'], rear_stop_m=limits['rear_stop_m'],
                available_us_m=us, required_us_m=limits['us_stop_m']+remaining+margin,
                target_m=selected, reason=reason)
