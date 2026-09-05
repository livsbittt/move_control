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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Pinky Pro forward/back control node using odom and cmd_vel',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'control_node = move_control.control_node:main',
            'safety_node = move_control.safety_node:main',
            'wander_node = move_control.wander_node:main',
            'calib_node = move_control.calib_node:main',
            'camera_detect_node = move_control.camera_detect_node:main',
            'watch_node = move_control.watch_node:main',
        ],
    },
)
