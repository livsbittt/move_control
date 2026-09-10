"""Stationary diagnostic reporting; exclusion never authorizes calibration or motion."""
from .calibration import SENSORS


def partial_sensing_report(sensors, excluded=('imu',)):
    report = {name: dict(item) for name, item in sensors.items()}
    excluded = tuple(excluded)
    reason = 'operator_requested_' + '_'.join(excluded) + '_exclusion'
    for name in excluded:
        report[name] = {**report.get(name, {}), 'ok': False, 'eligible': False,
                        'excluded': True, 'status': 'excluded', 'reason': reason,
                        'detail': name.upper() + ' excluded for stationary diagnostics; motion prohibited'}
    required = [name for name in SENSORS if name not in excluded]
    qualified = all(report.get(name, {}).get('eligible', report.get(name, {}).get('ok', False))
                    for name in required)
    return {'mode': 'sensing_only', 'partial': True, 'excluded_sensors': list(excluded),
            'exclusion_reason': reason, 'partial_baseline_ready': qualified,
            'ready': False, 'calibration_verified': False, 'motion_allowed': False,
            'rotation_verified': False, 'settings_applied': False, 'sensors': report}
