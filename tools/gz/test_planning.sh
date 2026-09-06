#!/usr/bin/env bash
# Gazebo test of the planning stack (GUI shows the maze + robot).
#   bash tools/gz/test_planning.sh
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
# Loopback discovery is mandatory on this machine (wifi discovery drops the
# graph for late joiners mid-run).
export ROS_DOMAIN_ID=13 ROS_LOCALHOST_ONLY=1 ROS_NETWORK_INTERFACE=lo GZ_IP=127.0.0.1
set --
mkdir -p /tmp/gztest
log() { echo "[runner] $*"; }

log "world: server + GUI (separate procs so a GUI crash can't kill the sim)"
# --headless-rendering: GPU lidar via EGL on the server; GUI is a client.
gz sim -s -r --headless-rendering tools/gz/pinky_maze.sdf \
  > /tmp/gztest/gz.log 2>&1 &
GZ=$!
sleep 3
# snap VS Code pollution (GTK_PATH etc.) breaks gz-sim-gui with
# GLIBC_PRIVATE — strip it for the GUI client only (server unaffected).
env -u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH \
  PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '^/snap/' | paste -sd:) \
  gz sim -g tools/gz/pinky_maze.sdf > /tmp/gztest/gui.log 2>&1 &
GUI=$!
sleep 6
log "bridge"
ros2 run ros_gz_bridge parameter_bridge \
  /lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /model/pinky/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /model/pinky/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  --ros-args -p use_sim_time:=true \
  -r /lidar/scan:=/scan -r /model/pinky/odometry:=/odom \
  -r /model/pinky/cmd_vel:=/cmd_vel \
  > /tmp/gztest/bridge.log 2>&1 &
BR=$!
sleep 2
log "goal_node"
python3 -c "import move_control.goal_node as g; g.main()" --ros-args \
  -p use_sim_time:=true --params-file config/goal.yaml \
     --params-file tools/gz/goal_sim.yaml \
  > /tmp/gztest/goal.log 2>&1 &
GO=$!
sleep 2
log "driver (TF glue first, slam needs it)"
python3 tools/gz/driver.py --ros-args -p use_sim_time:=true \
  -p v:=0.18 -p w:=1.0 > /tmp/gztest/driver.log 2>&1 &
DR=$!
log "waiting for static lidar TF from driver..."
for i in $(seq 1 60); do
  if timeout 2 ros2 topic echo /tf_static --once 2>/dev/null | grep -q frame_id; then
    log "static TF up (try $i)"
    break
  fi
  sleep 1
done
log "slam_toolbox (after static lidar TF exists)"
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:="$PWD/tools/gz/slam_sim.yaml" > /tmp/gztest/slam.log 2>&1 &
SL=$!
log "dashboard"
python3 tools/dashboard.py --ros-args -p use_sim_time:=true -p port:=8080 \
  > /tmp/gztest/dash.log 2>&1 &
DA=$!
log "all up: gz=$GZ bridge=$BR slam=$SL goal=$GO driver=$DR"
log "GUI: gz window | logs: /tmp/gztest/*.log"
trap 'kill $DR $GO $SL $BR $DA $GUI 2>/dev/null; sleep 1; kill $GZ 2>/dev/null' INT TERM
wait
