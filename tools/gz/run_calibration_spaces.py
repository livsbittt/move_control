"""Run isolated Gazebo calibration cases serially and retain every result."""
import argparse
import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases', nargs='*', default=['open','front_wall','rear_wall','corridor_exit','trapped'])
    parser.add_argument('--output', type=Path)
    parser.add_argument('--after', choices=['stay','return_origin'], default='stay')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = args.output or Path('/tmp/pinky-calibration-matrix')/datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for case in args.cases:
        if case not in ('open','front_wall','rear_wall','corridor_exit','trapped'):
            raise ValueError('Unknown case')
        target = output/case
        target.mkdir(exist_ok=True)
        env = dict(os.environ, RIG_CALIBRATION_CASE=case,
                   RIG_REALTIME_FACTOR=os.environ.get('RIG_REALTIME_FACTOR','0.2'),
                   RIG_CALIBRATION_AFTER=args.after,
                   RIG_DURATION='30' if case == 'trapped' else '180', RIG_WALL_TIMEOUT='700')
        print('Running calibration case: '+case, flush=True)
        with (target/'runner.log').open('w') as log:
            proc = subprocess.Popen(['bash',str(root/'tools/gz/run_track260905.sh')], cwd=root,
                                    env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = proc.wait(timeout=750)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                try: code = proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    code = proc.wait()
        rig = Path('/tmp/pinky-calmap227')
        names = ('track_result.json','track_samples.json','track_last_status.json','calibration.json',
                 'track_identity.json','run_manifest.json','stack.log','calibration.log','safety.log',
                 'track_map.npz','track_map.pgm','track_map.yaml')
        for name in names:
            if (rig/name).exists(): shutil.copy2(rig/name,target/name)
        result = {'case':case,'runner_exit':code,'pass':False}
        try:
            run = json.loads((target/'track_result.json').read_text())
            identity = json.loads((target/'track_identity.json').read_text())
            last = json.loads((target/'track_last_status.json').read_text())
            status = last['calibration']
            result.update(run_id=run['run_id'],phase=status.get('phase'),ready=status.get('ready'),
                          message=status.get('message'),relocation=status.get('relocation'),
                          after=args.after,return_motion=status.get('return_motion'),
                          elapsed_sim_s=run['elapsed_sim_s'],path_m=run.get('path_m'),
                          observation_error=run.get('observation_error'),
                          final_safe=last['safe'],min_center_to_wall_m=run.get('min_center_to_wall_m'))
            stopped = last['safe'] == [0.,0.] and run['cmd_vel_publishers'] == ['safety_node']
            if case == 'trapped':
                expected = (status.get('ready') is not True and run.get('path_m',1.) < .005 and
                            (status.get('phase') == 'waiting_space' or
                             (status.get('phase') in ('collecting','waiting_motion') and
                              'clearance' in status.get('message','').lower())))
            else:
                expected = status.get('ready') is True and status.get('settings_applied') is True
                if case != 'open': expected = expected and (status.get('relocation') or {}).get('done') is True
                if case != 'open' and args.after == 'return_origin':
                    expected = expected and (status.get('return_motion') or {}).get('done') is True
            result['pass'] = bool(code == 0 and not run.get('observation_error') and
                                  identity.get('case') == case and stopped and expected)
        except (OSError,ValueError,KeyError) as error:
            result['error'] = str(error)
        results.append(result)
        (output/'matrix.json').write_text(json.dumps(results,indent=2))
        print(json.dumps(result),flush=True)
    return 0 if all(row['pass'] for row in results) else 1


if __name__ == '__main__': sys.exit(main())
