#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 GZ_IP=127.0.0.1
export GZ_PARTITION=pinky_calmap227 PYTHONPATH="$PWD:${PYTHONPATH:-}"
out=/tmp/pinky-calmap227
mkdir -p "$out"
exec 9>"$out/run.lock"
flock -n 9 || { echo 'A calibration mapping rig already owns this partition'; exit 1; }
if [[ -n "${RIG_CALIBRATION_CASE:-}" ]]; then
  python3 -m tools.gz.calibration_spaces "$RIG_CALIBRATION_CASE" /tmp/pinky-calmap227
else
  python3 tools/gz/prepare_track_world.py
fi
python3 - "$out" <<'PY'
import sys, os, xml.etree.ElementTree as ET, yaml, shutil, time, json, hashlib, uuid, subprocess
from pathlib import Path
out=Path(sys.argv[1])
if (out/'stack.log').exists():
    archive=out/'archive'/str(time.time_ns())
    archive.mkdir(parents=True)
    camera_frames=out/'rendered-camera'
    if camera_frames.is_dir() and not camera_frames.is_symlink():
        camera_frames.rename(archive/'rendered-camera')
    for path in out.iterdir():
        if path.is_file() and path.suffix in ('.log','.json','.yaml','.pgm','.npz','.sdf','.txt'):
            shutil.copy2(path, archive/path.name)
run_id=uuid.uuid4().hex
# An old complete map is historical evidence, never this run's readiness.
for name in ('map.pgm','map.yaml','map_grid.npz','calibration.json','calibration.certificate.json','live_evidence.json',
             'track_samples.json','track_last_status.json','track_result.json',
             'track_map.npz','track_map.pgm','track_map.yaml','track_map_quality.json',
             'track_map_audit.json','track_result.png','obstacle-scenario.json',
             'decision-events.json','health-capture.json','scan-capture.json','realtime-adjustment.txt'):
    (out/name).unlink(missing_ok=True)
(out/'mapping_metrics.json').write_text(json.dumps({'run_id':run_id, 'status':'pending',
    'map_raster_complete':False, 'raster_and_sampled_clearance_ok':False}))
root=ET.parse(str(out/'track.sdf'))
model=root.find(".//model[@name='pinky']")
# Spawn is chosen against the exact oriented collision walls.
plant=os.environ.get('RIG_PLANT', 'wheel')
if plant == 'velocity':
    model.remove(model.find("plugin[@name='gz::sim::systems::DiffDrive']"))
    for joint in list(model.findall('joint')):
        model.remove(joint)
    for link in list(model.findall('link')):
        if link.get('name') != 'base':
            model.remove(link)
    base=model.find("link[@name='base']")
    ET.SubElement(base, 'gravity').text='false'
    ET.SubElement(model, 'plugin', filename='gz-sim-velocity-control-system', name='gz::sim::systems::VelocityControl')
(out/'plant.txt').write_text(plant)
root.find('.//real_time_factor').text=os.environ.get('RIG_REALTIME_FACTOR', '1.0')
sensor=model.find('.//sensor')
sensor.find('.//samples').text='720'
sensor.find('.//max_angle').text=str(3.141592653589793-2*3.141592653589793/720)
sensor.find('.//min_angle').text=str(-3.141592653589793)
sensor.find('.//range/max').text='8.0'
if os.environ.get('RIG_RENDERED_CAMERA') == '1':
    from tools.gz.obstacle_camera import add_camera
    add_camera(root.getroot())
root.write(out/'world.sdf')
identity=json.loads((out/'track_identity.json').read_text())
settings={'/**': {'ros__parameters': {'use_sim_time':True, 'robot_radius':identity['robot_radius_m'],
    'rotation_footprint_xy':[v for xy in identity['robot_geometry']['footprint_xy'] for v in xy],
    'simulation_motion_sweep_enabled':plant == 'wheel',
    'stop_distance':.14, 'clear_distance':.16, 'lidar_yaw_offset':0.,
    'imu_angular_velocity_unit':'rad_s', 'calibration_us_max_range':8.,
    'calibration_auto_motion':True, 'result_path':str(out/'calibration.json'),
    'obstacle_tracking_enabled':os.environ.get('RIG_TRACK_OBSTACLES') == '1'}}}
if os.environ.get('RIG_CALIBRATION_CASE'):
    settings['/**']['ros__parameters']['calibration_relocation_enabled']=True
    settings['/**']['ros__parameters']['calibration_after_relocation']=os.environ.get('RIG_CALIBRATION_AFTER','stay')
(out/'rig.yaml').write_text(yaml.safe_dump(settings))
slam=yaml.safe_load(Path('tools/gz/slam_sim.yaml').read_text())
slam['slam_toolbox']['ros__parameters']['resolution']=.02
if os.environ.get('RIG_TRACK_OBSTACLES') == '1':
    slam['slam_toolbox']['ros__parameters']['scan_topic']='/mapping/scan'
(out/'slam.yaml').write_text(yaml.safe_dump(slam))
source_paths=sorted(list(Path('move_control').rglob('*.py'))+list(Path('config').glob('*.yaml'))+
                    list(Path('tools/gz').glob('*.py'))+list(Path('tools/gz').glob('*.sh')))
manifest={'run_id':run_id, 'recorded_unix_s':time.time(), 'plant':plant,
    'calibration_case':os.environ.get('RIG_CALIBRATION_CASE'),
    'ros_domain':227, 'gazebo_partition':'pinky_calmap227',
    'world_sha256':hashlib.sha256((out/'world.sdf').read_bytes()).hexdigest(),
    'source_at_start':os.environ.get('RIG_SOURCE_COMMIT') or subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    'source_sha256':{str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths},
    'robot_radius_m':settings['/**']['ros__parameters']['robot_radius'],
    'robot_geometry':identity['robot_geometry'],
    'physical_robot_verified':False,
    'auxiliary_sensors':('Rendered camera; synthetic IR; GT-derived IMU; lidar-derived US'
        if os.environ.get('RIG_RENDERED_CAMERA') == '1' else
        'Synthetic camera/IR; GT-derived IMU; lidar-derived US')}
(out/'run_manifest.json').write_text(json.dumps(manifest,indent=2))
PY
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -TERM -- -"$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -KILL -- -"$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM
setsid gz sim -s -r --headless-rendering "$out/world.sdf" > "$out/gz.log" 2>&1 & pids+=($!)
sleep 4
if [[ "${RIG_TRACK_OBSTACLES:-0}" == 1 ]]; then
  setsid python3 -m move_control.obstacle_observer_node --ros-args -p use_sim_time:=true \
    > "$out/obstacle-observer.log" 2>&1 & pids+=($!)
  setsid python3 -m tools.gz.record_obstacle_decisions --ros-args -p use_sim_time:=true \
    > "$out/obstacle-recorder.log" 2>&1 & pids+=($!)
fi
if [[ "${RIG_RENDERED_CAMERA:-0}" == 1 ]]; then
  setsid ros2 run ros_gz_bridge parameter_bridge \
    '/pinky/rendered_camera@sensor_msgs/msg/Image[gz.msgs.Image' \
    --ros-args -p use_sim_time:=true > "$out/camera-bridge.log" 2>&1 & pids+=($!)
  setsid python3 -m tools.gz.rendered_camera_adapter --ros-args -p use_sim_time:=true \
    > "$out/camera.log" 2>&1 & pids+=($!)
fi
setsid ros2 run ros_gz_bridge parameter_bridge \
  '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/odometry_gt@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/pinky/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  --ros-args -p use_sim_time:=true -r /lidar/scan:=/scan -r /odometry_gt:=/odom_gz \
  -r /model/pinky/cmd_vel:=/cmd_vel > "$out/bridge.log" 2>&1 & pids+=($!)
for component in adapter safety calibration wander goal; do
  log="$out/$component.log"
  if [[ "$component" == adapter ]]; then log="$out/stack.log"; fi
  setsid env RIG_COMPONENT="$component" python3 tools/gz/calibration_mapping_rig.py --ros-args \
    --params-file config/robot.yaml --params-file config/wander.yaml \
    --params-file config/goal.yaml --params-file "$out/rig.yaml" \
    > "$log" 2>&1 & pids+=($!)
done
sleep 3
setsid ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:="$out/slam.yaml" > "$out/slam.log" 2>&1 & pids+=($!)

setsid python3 tools/gz/track_run_monitor.py > "$out/monitor.log" 2>&1 & pids+=($!)
if [[ "${RIG_OBSTACLE_SCENARIO:-0}" == 1 ]]; then
  setsid python3 -m tools.gz.obstacle_scenario --ros-args -p use_sim_time:=true \
    > "$out/obstacle-scenario.log" 2>&1 & pids+=($!)
  monitor_index=$((${#pids[@]}-2))
else
  monitor_index=$((${#pids[@]}-1))
fi
echo "Isolated exact-track run: $out"
wait "${pids[$monitor_index]}"
python3 - "$out/track_result.json" <<'PY'
import json, sys
result = json.load(open(sys.argv[1]))
import os
if not os.environ.get('RIG_CALIBRATION_CASE'):
    assert result['calibration_ready'] is True, result['message']
assert result['cmd_vel_publishers'] == ['safety_node'], 'Unexpected final command publisher'
import os
if os.environ.get('RIG_REQUIRE_COMPLETE') == '1':
    assert result['mapping_complete'] is True, 'Full fresh world-aligned map acceptance failed'
PY
if [[ "${RIG_REQUIRE_COMPLETE:-0}" == 1 ]]; then
  python3 -m tools.gz.track_map_audit "$out"
fi
