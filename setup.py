import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'move_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='livsbittt',
    maintainer_email='56295815+livsbittt@users.noreply.github.com',
    description='Pinky Pro forward/back control node using odom and cmd_vel',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'control_node = move_control.control_node:main',
            'safety_node = move_control.safety_node:main',
            'wander_node = move_control.wander_node:main',
            'calib_node = move_control.calib_node:main',
            'startup_calibration_node = move_control.startup_calibration_node:main',
            'camera_detect_node = move_control.camera_detect_node:main',
            'obstacle_observer_node = move_control.obstacle_observer_node:main',
            'watch_node = move_control.watch_node:main',
            'goal_node = move_control.goal_node:main',
            'localization_node = move_control.localization_node:main',
            'web_node = move_control.web_node:main',
        ],
    },
)
