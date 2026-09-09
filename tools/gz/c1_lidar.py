"""C1-equivalent GPU lidar mount for isolated Gazebo rigs.

Scan 0 is the rear (yaw=π), matching rplidar_link. Auxiliary sensors stay
synthetic; callers must advertise that on /robot/evidence_scope.
"""
import math

C1_SCAN_YAW = math.pi
C1_RANGE_MIN = 0.05
C1_RANGE_MAX = 40.0
C1_SAMPLES = 720

EVIDENCE_SCOPE = {
    'lidar': 'gpu_lidar',
    'imu': 'gt_pose',
    'ir': 'synthetic',
    'us': 'lidar_derived',
    'camera': 'synthetic',
}


def scan_tf_quaternion():
    """base_link <- lidar rotation for yaw=π. Nose from TF is π."""
    return (0.0, 0.0, 1.0, 0.0)


def align_gpu_lidar(sensor):
    pose = sensor.find('pose')
    if pose is None:
        raise ValueError('GPU lidar is missing a pose')
    parts = (pose.text or '0 0 0.10 0 0 0').split()
    if len(parts) < 6:
        raise ValueError('GPU lidar pose must have 6 values')
    parts[5] = str(C1_SCAN_YAW)
    pose.text = ' '.join(parts)
    samples = sensor.find('.//samples')
    horizontal = sensor.find('.//horizontal')
    rng = sensor.find('.//range')
    if samples is None or horizontal is None or rng is None:
        raise ValueError('GPU lidar is missing scan or range elements')
    samples.text = str(C1_SAMPLES)
    span = 2.0 * math.pi / C1_SAMPLES
    min_el = horizontal.find('min_angle')
    max_el = horizontal.find('max_angle')
    min_r = rng.find('min')
    max_r = rng.find('max')
    if None in (min_el, max_el, min_r, max_r):
        raise ValueError('GPU lidar is missing angle or range limits')
    min_el.text = str(-math.pi)
    max_el.text = str(math.pi - span)
    min_r.text = str(C1_RANGE_MIN)
    max_r.text = str(C1_RANGE_MAX)
    return sensor
