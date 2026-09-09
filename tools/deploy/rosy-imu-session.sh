#!/usr/bin/env bash
# The service owns IMU restarts; robot.launch.py must use start_imu:=false.
set -e
source /opt/ros/jazzy/setup.bash
source "${ROSY_HARDWARE_WS:-$HOME/pinky_pro}/install/setup.bash"
exec ros2 run "${ROSY_IMU_PACKAGE:-pinky_imu_bno055}" main_node
