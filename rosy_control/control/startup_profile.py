"""Pure startup selection; process selection never grants motion readiness."""
FEATURES = ('imu', 'camera', 'wander', 'lcd', 'web', 'watch', 'calibration')


def _option(value, default):
    value = str(value).strip().lower()
    if value == 'auto':
        return default
    if value not in ('true', 'false'):
        raise ValueError(f'Expected auto, true, or false; got {value!r}')
    return value == 'true'


def startup_profile(profile='full', overrides=None, calibration_sensing_only='auto'):
    if profile not in ('full', 'sensing'):
        raise ValueError(f'Unknown startup profile: {profile!r}')
    overrides = overrides or {}
    if set(overrides) - set(FEATURES):
        raise ValueError('Only optional feature nodes can be overridden; safety is mandatory')
    nodes = {'safety': True}
    for name in FEATURES:
        nodes[name] = _option(overrides.get(name, 'auto'), profile == 'full' or name in ('camera', 'web', 'calibration'))
    sensing = _option(calibration_sensing_only, profile == 'sensing')
    if profile == 'sensing' and (not sensing or not nodes['calibration']):
        raise ValueError('Sensing profile requires stationary partial calibration to remain enabled')
    return {'nodes': nodes, 'calibration_sensing_only': sensing}
