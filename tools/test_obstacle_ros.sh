#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
export ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
result=0
python3 -m pytest tools/test_safety_gate_ros.py tools/test_consolidated_safety_ros.py tools/test_goal_route_ros.py tools/test_obstacle_gate_ros.py tools/test_obstacle_goal_ros.py tools/test_obstacle_wander_ros.py tools/test_obstacle_observer_ros.py -q > /root/pinky-obstacle-ros-tests.log 2>&1 || result=$?
cat /root/pinky-obstacle-ros-tests.log
exit "$result"
