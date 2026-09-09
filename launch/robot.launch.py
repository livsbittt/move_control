"""Select processing features; hardware bringup and ADC are separate prerequisites."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rosy_control.control.startup_profile import FEATURES, startup_profile


def _share(pkg, *parts):
    return os.path.join(get_package_share_directory(pkg), *parts)


def _processing_actions(context):
    """Resolve selection before optional package lookup or process construction."""
    selected = startup_profile(
        LaunchConfiguration('profile').perform(context),
        {name: LaunchConfiguration('start_' + name).perform(context) for name in FEATURES},
        LaunchConfiguration('calibration_sensing_only').perform(context))
    enabled = selected['nodes']
    robot = _share('rosy_control', 'config', 'robot.yaml')
    cfg = _share('rosy_control', 'config')

    def node(executable, parameters, package='rosy_control', **kwargs):
        return Node(package=package, executable=executable, output='screen',
                    parameters=parameters, respawn=True, respawn_delay=1.0, **kwargs)

    actions = [LogInfo(msg='robot.launch selected processes: ' + ', '.join(
        name for name, value in enabled.items() if value) +
        '; disabled: ' + ', '.join(name for name, value in enabled.items() if not value) +
        f"; calibration_sensing_only={selected['calibration_sensing_only']}. " +
        'Delays order startup only; sensor and calibration gates determine readiness.')]
    if enabled['imu']:
        actions.append(node('main_node', [], package='pinky_imu_bno055', name='pinky_imu_bno055'))
    if enabled['camera']:
        actions.append(node('camera_detect_node', [robot, os.path.join(cfg, 'camera.yaml')]))
    safety = node('safety_node', [robot, os.path.join(cfg, 'safety.yaml'),
        os.path.join(cfg, 'cliff_calib.yaml'), os.path.join(cfg, 'auto_calib.yaml'),
        {'localization_required': LaunchConfiguration('localization_required')}])
    actions.append(TimerAction(period=1.5, actions=[safety]))
    if enabled['wander']:
        actions.append(TimerAction(period=3.0, actions=[
            node('wander_node', [robot, os.path.join(cfg, 'wander.yaml')])]))
    displays = []
    if enabled['lcd']:
        displays.append(node('lcd_node', [robot, _share('lcd_control', 'config', 'lcd.yaml')],
                             package='lcd_control'))
    if enabled['web']:
        displays.append(node('web_node', [robot, os.path.join(cfg, 'web.yaml')]))
    if enabled['watch']:
        displays.append(node('watch_node', [robot, {'hz': 1.0, 'once': False}]))
        actions.append(LogInfo(msg='watch_node checks the full hardware graph; disabled feature nodes remain reported missing.'))
    if displays:
        actions.append(TimerAction(period=3.5, actions=displays))
    if enabled['calibration']:
        actions.append(TimerAction(period=4.0, actions=[node('startup_calibration_node',
            [robot, {'calibration_sensing_only': selected['calibration_sensing_only']}])]))
    return actions


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('profile', default_value='full', choices=['full', 'sensing']),
        DeclareLaunchArgument('localization_required', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('calibration_sensing_only', default_value='auto', choices=['auto', 'true', 'false']),
    ]
    arguments += [DeclareLaunchArgument('start_' + name, default_value='auto',
                  choices=['auto', 'true', 'false']) for name in FEATURES]
    return LaunchDescription(arguments + [OpaqueFunction(function=_processing_actions)])
