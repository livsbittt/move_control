#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
export ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
exec python3 tools/dashboard_operations_rig.py --ros-args -p port:=28361 -p backend_port:=28362 -p teleop_topic:=/cmd_vel_raw
