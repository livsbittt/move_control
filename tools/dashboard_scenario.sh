#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1
exec python3 "$(dirname "$0")/dashboard_scenario.py" "$1"
