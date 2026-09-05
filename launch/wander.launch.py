import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('move_control'), 'config')
    return LaunchDescription([
        Node(
            package='pinky_imu_bno055',
            executable='main_node',
            name='pinky_imu_bno055',
            output='screen',
        ),
        Node(
            package='move_control',
            executable='camera_detect_node',
            output='screen',
            parameters=[os.path.join(cfg, 'camera.yaml')],
        ),
        Node(
            package='move_control',
            executable='safety_node',
            output='screen',
            parameters=[
                os.path.join(cfg, 'safety.yaml'),
                os.path.join(cfg, 'cliff_calib.yaml'),
                os.path.join(cfg, 'auto_calib.yaml'),
            ],
        ),
        Node(
            package='move_control',
            executable='wander_node',
            output='screen',
            parameters=[os.path.join(cfg, 'wander.yaml')],
        ),
    ])
