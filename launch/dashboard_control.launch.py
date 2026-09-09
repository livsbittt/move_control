"""Mapping/control application over an already running hardware sensor stack."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rosy_control')
    cfg = os.path.join(share, 'config')
    robot = os.path.join(cfg, 'robot.yaml')

    def node(executable, extra=(), overrides=None):
        parameters = [robot] + [os.path.join(cfg, item) for item in extra]
        if overrides:
            parameters.append(overrides)
        return Node(package='rosy_control', executable=executable, output='screen',
                    parameters=parameters, respawn=True, respawn_delay=1.0)

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'map.launch.py')),
            launch_arguments={'start_goal': 'false'}.items()),
        TimerAction(period=1.5, actions=[node('safety_node',
            ('safety.yaml', 'cliff_calib.yaml', 'auto_calib.yaml'),
            {'start_estopped': True, 'lidar_use_tf': True,
             'stop_distance': .12, 'clear_distance': .14})]),
        TimerAction(period=3.0, actions=[node('wander_node', ('wander.yaml',),
            {'auto_start': False, 'calibration_required': True})]),
        TimerAction(period=3.5, actions=[node('goal_node', ('goal.yaml',), {'mode': 'stop'}),
            node('web_node', ('web.yaml',), {'teleop_topic': '/cmd_vel_raw'})]),
        TimerAction(period=4.0, actions=[node('startup_calibration_node')]),
    ])
