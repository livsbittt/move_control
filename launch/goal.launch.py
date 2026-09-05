import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('move_control'), 'config')
    robot = os.path.join(cfg, 'robot.yaml')
    return LaunchDescription([
        Node(
            package='move_control',
            executable='goal_node',
            output='screen',
            parameters=[robot, os.path.join(cfg, 'goal.yaml')],
            respawn=True,
            respawn_delay=1.0,
        ),
    ])
