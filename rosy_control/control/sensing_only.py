"""Stationary diagnostic reporting; exclusion never authorizes calibration or motion."""
from .calibration import SENSORS


def partial_sensing_report(sensors):
    report = {name: dict(item) for name, item in sensors.items()}
    reason = 'operator_requested_imu_exclusion'
    report['imu'] = {**report.get('imu', {}), 'ok': False, 'eligible': False,
                     'excluded': True, 'status': 'excluded', 'reason': reason,
                     'detail': 'IMU excluded for stationary diagnostics; motion prohibited'}
    required = [name for name in SENSORS if name != 'imu']
    qualified = all(report.get(name, {}).get('eligible', report.get(name, {}).get('ok', False))
                    for name in required)
    return {'mode': 'sensing_only', 'partial': True, 'excluded_sensors': ['imu'],
            'exclusion_reason': reason, 'partial_baseline_ready': qualified,
            'ready': False, 'calibration_verified': False, 'motion_allowed': False,
            'rotation_verified': False, 'settings_applied': False, 'sensors': report}
