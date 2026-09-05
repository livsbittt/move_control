"""Sensing package: raw sensor data -> robot-frame geometry.

lidar  — C1 scan geometry (the nose-yaw trap, sectors, frontiers).
filt   — median / low-pass filters (IR, ranges).
body   — physical footprint constants (radius, chassis-hit ignore).
camera — HSV floor/void/obstacle classification.
"""
from .body import (CASTER_EXTRA, CASTER_X, FRONT_X, LIDAR_X, RADIUS_HI,
                   RADIUS_LO, URDF_RADIUS, WHEEL_R, WHEEL_Y, ignore_m,
                   turn_clear_m, urdf_radius, use_radius)
from .camera import classify_frame
from .filt import IrMedian, MedianLp
from .lidar import (MOUNT_YAW_DEG, NOSE_YAW, find_frontiers, is_robot_scan,
                    opening_max, robot_yaw, sector_min, sector_range, wrap_pi)

__all__ = ['CASTER_EXTRA', 'CASTER_X', 'FRONT_X', 'IrMedian', 'LIDAR_X',
           'MOUNT_YAW_DEG', 'MedianLp', 'NOSE_YAW', 'RADIUS_HI', 'RADIUS_LO',
           'URDF_RADIUS', 'WHEEL_R', 'WHEEL_Y', 'classify_frame',
           'find_frontiers', 'ignore_m', 'is_robot_scan', 'opening_max',
           'robot_yaw', 'sector_min', 'sector_range', 'turn_clear_m',
           'urdf_radius', 'use_radius', 'wrap_pi']
