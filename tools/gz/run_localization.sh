#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=pinky_localization228 GZ_IP=127.0.0.1
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
export LOCALIZATION_OUTPUT="/tmp/pinky-localization228/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOCALIZATION_OUTPUT"
exec 9>/tmp/pinky-localization228.lock
flock -w 30 9 || exit 1
printf '%s\n' "$LOCALIZATION_OUTPUT" > /tmp/pinky-localization228/latest
python3 - <<'PY'
import hashlib, json, os
from pathlib import Path
root = Path.cwd()
paths = [p for folder in ('move_control', 'config', 'launch', 'tools/gz', 'test')
         for p in (root/folder).rglob('*') if p.suffix in ('.py', '.yaml', '.sh', '.npz')]
paths += [root/'map/calibrated_2026-09-08'/name for name in ('map.yaml', 'map.pgm', 'world.sdf')]
manifest = {'source_base': '8853261', 'ros_domain': 228, 'gz_partition': 'pinky_localization228',
            'sha256': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}}
(Path(os.environ['LOCALIZATION_OUTPUT'])/'run_manifest.json').write_text(json.dumps(manifest, indent=2))
PY
pids=()
start() {
  local label=$1
  shift
  setsid "$@" 9>&- > "$LOCALIZATION_OUTPUT/$label.log" 2>&1 &
  pids+=("$!")
}
cleanup() {
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
start gazebo gz sim -s -r --headless-rendering map/calibrated_2026-09-08/world.sdf
sleep 5
start bridge ros2 run ros_gz_bridge parameter_bridge \
  '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/odometry_gt@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/pinky/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  --ros-args -p use_sim_time:=true -r /lidar/scan:=/scan_source \
  -r /odometry_gt:=/ground_truth -r /model/pinky/cmd_vel:=/cmd_vel
start map /opt/ros/jazzy/lib/nav2_map_server/map_server --ros-args -p use_sim_time:=true \
  -p yaml_filename:="$PWD/map/calibrated_2026-09-08/map.yaml"
start amcl /opt/ros/jazzy/lib/nav2_amcl/amcl --ros-args -p use_sim_time:=true --params-file config/localization.yaml
start lifecycle /opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args \
  -r __node:=localization_lifecycle_manager -p use_sim_time:=true -p autostart:=true \
  -p 'node_names:=[map_server,amcl]'
start monitor env LOCALIZATION_COMPONENT=monitor python3 tools/gz/localization_rig.py \
  --ros-args -p use_sim_time:=true -p robot_radius:=0.105 --params-file config/localization.yaml
start safety env LOCALIZATION_COMPONENT=safety python3 tools/gz/localization_rig.py \
  --ros-args -p use_sim_time:=true -p localization_required:=true -p start_estopped:=false \
  -p robot_radius:=0.105 -p lidar_yaw_offset:=0.0 -p imu_angular_velocity_unit:=rad_s
start goal env LOCALIZATION_COMPONENT=goal python3 tools/gz/localization_rig.py \
  --ros-args -p use_sim_time:=true -p localization_required:=true -p static_map:=true
start rig python3 tools/gz/localization_rig.py --ros-args -p use_sim_time:=true
echo "$LOCALIZATION_OUTPUT"
wait "${pids[-1]}"
python3 - "$LOCALIZATION_OUTPUT/acceptance.json" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))['passed'], 'Localization acceptance failed'
PY
