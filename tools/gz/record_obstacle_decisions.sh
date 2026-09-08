#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1
exec python3 -m tools.gz.record_obstacle_decisions --ros-args -p use_sim_time:=true
