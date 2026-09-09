import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('rosy_control'), 'config')
    robot = os.path.join(cfg, 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('localization_required', default_value='false'),
        DeclareLaunchArgument('static_map', default_value='false'),
        Node(
            package='rosy_control',
            executable='goal_node',
            output='screen',
            parameters=[robot, os.path.join(cfg, 'goal.yaml'), {
                'localization_required': LaunchConfiguration('localization_required'),
                'static_map': LaunchConfiguration('static_map')}],
            respawn=True,
            respawn_delay=1.0,
        ),
    ])
