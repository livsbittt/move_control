"""Reload only explicitly identified isolated navigation processes for debugging."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

if os.environ.get('ROS_DOMAIN_ID') != '227' or os.environ.get('GZ_PARTITION') != 'pinky_calmap227':
    raise RuntimeError('Isolated simulation required')
if len(sys.argv) != 3 or len(set(sys.argv[1:])) != 2:
    raise RuntimeError('Exactly one goal PID and one wander PID required')
owned = {}
for raw in sys.argv[1:]:
    pid = int(raw)
    env = (Path('/proc')/str(pid)/'environ').read_bytes().split(b'\0')
    command = (Path('/proc')/str(pid)/'cmdline').read_bytes()
    if b'ROS_DOMAIN_ID=227' not in env or b'GZ_PARTITION=pinky_calmap227' not in env or b'calibration_mapping_rig.py' not in command or not any(
            value in env for value in (b'RIG_COMPONENT=goal', b'RIG_COMPONENT=wander')):
        raise RuntimeError('PID is not owned isolated navigation')
    component = 'goal' if b'RIG_COMPONENT=goal' in env else 'wander'
    if component in owned:
        raise RuntimeError('Both navigation components must be identified')
    owned[component] = pid
for pid in owned.values():
    os.kill(pid, signal.SIGTERM)
deadline = time.monotonic()+10.
while any((Path('/proc')/str(pid)).exists() for pid in owned.values()):
    if time.monotonic() >= deadline:
        raise RuntimeError('Previous navigation processes have not exited; no replacements started')
    time.sleep(.1)
children = []
try:
    for component in ('goal', 'wander'):
        env = dict(os.environ, RIG_COMPONENT=component)
        log = open(f'/tmp/pinky-calmap227/{component}.log', 'a')
        children.append(subprocess.Popen([sys.executable, 'tools/gz/calibration_mapping_rig.py',
            '--ros-args', '--params-file', 'config/robot.yaml', '--params-file', 'config/wander.yaml',
            '--params-file', 'config/goal.yaml', '--params-file', '/tmp/pinky-calmap227/rig.yaml'],
            env=env, stdout=log, stderr=subprocess.STDOUT))
    print('Reloaded navigation:', [child.pid for child in children], flush=True)
    for child in children:
        child.wait()
finally:
    for child in children:
        if child.poll() is None:
            child.terminate()
