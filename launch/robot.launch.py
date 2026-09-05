"""One processing stack. Bringup (lidar+motors) must already be running.

Order: IMU+camera → safety → wander → LCD+web+watch.
Safety/wander/camera respawn if they die.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def _share(pkg, *parts):
    return os.path.join(get_package_share_directory(pkg), *parts)


def generate_launch_description():
    robot = _share('move_control', 'config', 'robot.yaml')
    mc = _share('move_control', 'config')
    imu = Node(
        package='pinky_imu_bno055',
        executable='main_node',
        name='pinky_imu_bno055',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
    )
    camera = Node(
        package='move_control',
        executable='camera_detect_node',
        output='screen',
        parameters=[os.path.join(mc, 'camera.yaml')],
        respawn=True,
        respawn_delay=1.0,
    )
    safety = Node(
        package='move_control',
        executable='safety_node',
        output='screen',
        parameters=[
            robot,
            os.path.join(mc, 'safety.yaml'),
            os.path.join(mc, 'cliff_calib.yaml'),
            os.path.join(mc, 'auto_calib.yaml'),
        ],
        respawn=True,
        respawn_delay=1.0,
    )
    wander = Node(
        package='move_control',
        executable='wander_node',
        output='screen',
        parameters=[robot, os.path.join(mc, 'wander.yaml')],
        respawn=True,
        respawn_delay=1.0,
    )
    lcd = Node(
        package='lcd_control',
        executable='lcd_node',
        output='screen',
        parameters=[robot, _share('lcd_control', 'config', 'lcd.yaml')],
        respawn=True,
        respawn_delay=1.0,
    )
    web = Node(
        package='pinky_web',
        executable='web_node',
        output='screen',
        parameters=[robot, _share('pinky_web', 'config', 'web.yaml')],
        respawn=True,
        respawn_delay=1.0,
    )
    watch = Node(
        package='move_control',
        executable='watch_node',
        output='screen',
        parameters=[{'hz': 1.0, 'once': False}],
        respawn=True,
        respawn_delay=1.0,
    )
    on_safety_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=safety,
            on_exit=[LogInfo(msg='safety_node died — respawn. wander has no bumper until it is back.')],
        )
    )
    return LaunchDescription([
        LogInfo(msg='robot.launch: imu+camera, then safety, wander, lcd+web+watch'),
        imu,
        camera,
        TimerAction(period=1.5, actions=[safety]),
        TimerAction(period=3.0, actions=[wander]),
        TimerAction(period=3.5, actions=[lcd, web, watch]),
        on_safety_exit,
    ])
