from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='move_control', executable='calib_node', output='screen'),
    ])
