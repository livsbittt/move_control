"""Existing parameter operation is live permission, never calibration evidence."""


def configured_waiting_reasons(sensors, localization_required, geometry_fresh, estop, hazards, now, excluded=()):
    required = ('lidar', 'odom', 'ir', 'imu', 'us', 'tf')
    required = tuple(name for name in required if name not in excluded)
    if localization_required:
        required += ('map', 'map_tf')
    reasons = [name for name in required if not sensors.get(name, {}).get('eligible', False)]
    if not geometry_fresh:
        reasons.append('safety_geometry')
    if estop is not False:
        reasons.append('estop')
    for name in ('blocked', 'cliff', 'tilt', 'pickup'):
        stamp, active = hazards.get(name, (float('-inf'), True))
        # A fresh front obstacle restricts direction in the safety gate; revoking
        # the operating lease here also prevents rear-clear escape and avoidance.
        if (not 0 <= now-stamp <= .75 or type(active) is not bool or
                (name != 'blocked' and active is not False)):
            reasons.append('safety_' + name)
    return reasons


def configured_status(ready, waiting_reasons, limited_sensors=False, excluded_sensors=None):
    if excluded_sensors is None:
        excluded_sensors = ('imu',) if limited_sensors else ()
    return {'mode': 'limited_sensors' if limited_sensors else 'existing_settings', 'existing_settings': True,
            'limited_sensors': limited_sensors, 'degraded': limited_sensors,
            'excluded_sensors': list(excluded_sensors),
            **({'speed_limits': {'linear_mps': .005, 'angular_rad_s': .05}} if limited_sensors else {}),
            'settings_source': 'configured_parameters', 'calibration_skipped': True,
            'calibration_complete': True, 'completion_source': 'operator_override',
            'calibration_verified': False, 'rotation_verified': False,
            'ready': bool(ready), 'operating_ready': bool(ready), 'motion_allowed': bool(ready),
            'waiting_reasons': list(waiting_reasons)}
