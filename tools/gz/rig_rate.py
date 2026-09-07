"""Change wall-clock simulation rate, preserving physical command limits."""
import os
import subprocess
import sys
if os.environ.get('GZ_PARTITION') != 'pinky_calmap227':
    raise RuntimeError('Isolated simulation partition required')
rate = float(sys.argv[1])
if not .1 <= rate <= 4.:
    raise ValueError('Unsupported simulation rate')
subprocess.run(['gz', 'service', '-s', '/world/pinky_maze/set_physics',
    '--reqtype', 'gz.msgs.Physics', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
    '--req', f'real_time_factor: {rate} max_step_size: 0.005'], check=True)
