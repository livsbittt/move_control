"""Startup validation; stationary by default, no automatic motor release."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('move_control'), 'config', 'robot.yaml')
    return LaunchDescription([Node(package='move_control', executable='startup_calibration_node',
        parameters=[cfg], output='screen', respawn=True, respawn_delay=1.0)])
