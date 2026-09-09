import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('rosy_control'), 'config', 'mapper.yaml'
    )
    slam = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py',
    )
    return LaunchDescription([
        DeclareLaunchArgument('start_goal', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': params,
            }.items(),
        ),
        # Mapping sessions need a route producer as well as the SLAM raster.
        # goal.yaml starts idle; including the planner never starts driving.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rosy_control'), 'launch', 'goal.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_goal')),
        ),
    ])
