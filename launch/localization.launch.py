"""Fixed-map localization; do not run alongside map.launch.py / slam_toolbox.

The existing safety and goal nodes must use localization_required:=true;
goal_node also needs static_map:=true. This launch owns no motor publisher.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    folder = os.path.join(get_package_share_directory('move_control'), 'config')
    config = os.path.join(folder, 'localization.yaml')
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path of saved map YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[sim, {'yaml_filename': LaunchConfiguration('map')}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[config, sim], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='localization_lifecycle_manager', parameters=[sim, {
                 'autostart': True, 'node_names': ['map_server', 'amcl']}], output='screen'),
        Node(package='move_control', executable='localization_node',
             parameters=[os.path.join(folder, 'robot.yaml'), config,
                         os.path.join(folder, 'auto_calib.yaml'), sim], respawn=True, respawn_delay=1., output='screen'),
    ])
