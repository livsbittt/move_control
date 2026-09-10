"""Exclusion tiers; the only place that decides which sensors a partial calibration may drop."""
from .calibration import SENSORS

# Motion permission is impossible without these. They are never excludable.
REQUIRED_FOR_MOTION = ('lidar', 'odom', 'ir', 'us', 'tf')
# The map pair is required only while localization is in use.
REQUIRED_FOR_LOCALIZATION = ('map', 'map_tf')
# Dropping imu costs rotation verification, because rotation is measured as
# IMU-against-odometry agreement. Dropping camera costs nothing: it is advisory
# and never gates motion.
EXCLUDABLE = ('imu', 'camera')


def failed_sensors(sensors):
    return tuple(name for name in SENSORS
                 if not sensors.get(name, {}).get('eligible', sensors.get(name, {}).get('ok', False)))


def partial_plan(sensors, localization_required=False):
    """Report what a partial calibration can do about the current sensor failures.

    Availability is never a motion permission; the runtime gates still apply.
    """
    failed = failed_sensors(sensors)
    required = REQUIRED_FOR_MOTION + (REQUIRED_FOR_LOCALIZATION if localization_required else ())
    blocking = tuple(name for name in failed if name in required)
    if blocking:
        return {'available': False, 'reason': 'required_sensor_failed',
                'blocking_sensors': blocking, 'excluded_sensors': (), 'rotation_available': False}
    excluded = tuple(name for name in failed if name in EXCLUDABLE)
    if not excluded:
        return {'available': False, 'reason': 'no_failed_sensor',
                'blocking_sensors': (), 'excluded_sensors': (), 'rotation_available': True}
    rotation = 'imu' not in excluded
    return {'available': True, 'reason': 'calibration_continues' if rotation else 'rotation_unavailable',
            'blocking_sensors': (), 'excluded_sensors': excluded, 'rotation_available': rotation}
