"""Saved-map robot stack with localization authority enforced at the motor gate."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    folder = os.path.join(get_package_share_directory('move_control'), 'launch')

    def include(name, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(folder, name)),
                                        launch_arguments=arguments.items())

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute saved map YAML; stop mapping stack first'),
        include('robot.launch.py', {'localization_required': 'true'}),
        include('goal.launch.py', {'localization_required': 'true', 'static_map': 'true'}),
        include('localization.launch.py', {'map': LaunchConfiguration('map')}),
    ])
