#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=pinky_calmap227
python3 -m tools.gz.obstacle_scenario --ros-args -p use_sim_time:=true
