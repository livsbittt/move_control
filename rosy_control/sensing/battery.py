"""Battery telemetry presentation; never infer charge from voltage alone."""
import math


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def battery_values(percentage, voltage, current, present, status):
    if not present:
        return {'available': False, 'reason': 'not_present'}
    percent = round(percentage*100, 1) if finite(percentage) and 0 <= percentage <= 1 else None
    volts = round(voltage, 2) if finite(voltage) and voltage > 0 else None
    amps = round(current, 2) if finite(current) else None
    return {'available': percent is not None or volts is not None, 'reason': 'live' if percent is not None or volts is not None else 'invalid',
            'percent': percent, 'voltage': volts, 'current': amps,
            'status': {1:'charging', 2:'discharging', 3:'not_charging', 4:'full'}.get(status, 'unknown')}


def battery_snapshot(value, received, now, ttl=5.):
    if received is None:
        return {'available': False, 'reason': 'missing'}
    if not 0 <= now-received <= ttl:
        return {'available': False, 'reason': 'stale'}
    return dict(value)
