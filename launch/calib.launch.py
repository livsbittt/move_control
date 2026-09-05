import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot = os.path.join(
        get_package_share_directory('move_control'), 'config', 'robot.yaml'
    )
    return LaunchDescription([
        Node(
            package='move_control',
            executable='calib_node',
            output='screen',
            parameters=[robot],
        ),
    ])
