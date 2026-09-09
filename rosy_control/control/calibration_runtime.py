"""Completed calibration evidence is independent of live operating permission."""


def calibration_runtime(sensors, *, localization_required, geometry_fresh,
                        established_revision, current_revision, current_geometry_fresh):
    required = ('lidar','odom','ir','imu','us','tf')
    if localization_required:
        required += ('map','map_tf')
    report = {name: {**item, 'required_for_operation': name in required}
              for name, item in sensors.items()}
    waiting = [name for name in required if not sensors.get(name, {}).get('eligible', False)]
    changed = bool(current_geometry_fresh and isinstance(established_revision, str)
                   and established_revision and isinstance(current_revision, str)
                   and current_revision and established_revision != current_revision)
    if changed:
        waiting.append('calibration_identity_changed')
    elif not geometry_fresh:
        waiting.append('safety_geometry')
    return {'sensors': report, 'waiting_reasons': waiting, 'recalibration_required': changed}


def precision_scan_required(phase):
    return phase in ('collecting','waiting_motion','validating_motion','validating_rotation',
                     'relocating_calibration','returning_calibration')
