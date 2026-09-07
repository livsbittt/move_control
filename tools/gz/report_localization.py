"""Create read-only evidence metrics and a plot from a completed Gazebo run."""
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    run = Path(sys.argv[1])
    acceptance = json.loads((run/'acceptance.json').read_text())
    samples = [json.loads(line) for line in (run/'samples.jsonl').read_text().splitlines()]
    events = {event['phase']: event for event in acceptance['events']}
    metrics = {'passed': acceptance['passed'], 'cmd_vel_publishers': acceptance['cmd_vel_publishers'],
               'route_messages': acceptance['route_messages']}
    for start, end, name in [('teleport_translation', 'rotate', 'translation'),
                              ('teleport_rotation', 'scan_outage', 'rotation')]:
        t0, t1 = events[start]['sim_s'], events[end]['sim_s']
        loss = [s for s in samples if s['phase'] == start and not s['ready']]
        stopped = [s for s in loss if max(map(abs, s['output'])) < 1e-9]
        hold = [s for s in loss if s['sim_s'] > t0+1.]
        metrics[name] = {'recovery_sim_s': t1-t0, 'position_error_m': events[end]['error'][0],
                         'yaw_error_deg': math.degrees(events[end]['error'][1]),
                         'detection_sim_s': loss[0]['sim_s']-t0,
                         'stop_sim_s': stopped[0]['sim_s']-t0,
                         'max_output_while_lost_after_1s': max(max(map(abs, s['output'])) for s in hold)}
    metrics['scan_restore_sim_s'] = samples[-1]['sim_s'] - events['scan_restored']['sim_s']
    metrics['odom_max_step_m'] = max(math.dist(a['odom'][:2], b['odom'][:2]) for a, b in zip(samples, samples[1:]))
    normal = [s for s in samples if s['ready'] and s['error'] and s['phase'] in ('drive', 'rotate')]
    metrics['tracking_max_position_error_m'] = max(s['error'][0] for s in normal)
    metrics['tracking_max_yaw_error_deg'] = math.degrees(max(s['error'][1] for s in normal))
    metrics['initial_position_error_m'], initial_yaw = events['drive']['error']
    metrics['initial_yaw_error_deg'] = math.degrees(initial_yaw)
    metrics['initial_acquisition_sim_s'] = events['drive']['sim_s'] - samples[0]['sim_s']
    manifest = json.loads((run/'run_manifest.json').read_text())
    root = Path(__file__).resolve().parents[2]
    metrics['saved_map_hashes_unchanged'] = all(hashlib.sha256((root/path).read_bytes()).hexdigest() == digest
        for path, digest in manifest['sha256'].items() if path.endswith(('map.pgm', 'map.yaml')))
    (run/'metrics.json').write_text(json.dumps(metrics, indent=2))

    t = np.array([s['sim_s'] for s in samples])
    known = np.array([s['ready'] for s in samples])
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    for ax in axes:
        ax.grid(alpha=.2)
        ax.fill_between(t, 0, 1, where=~known, transform=ax.get_xaxis_transform(), color='#fbe7d6', label='Localization held')
    error = np.array([s['error'] if s['error'] else [np.nan, np.nan] for s in samples])
    axes[0].plot(t, error[:, 0]*100, color='#2767ac', label='Position error')
    axes[0].set_ylabel('Position error (cm)')
    axes[0].set_yscale('symlog', linthresh=1)
    axes[0].legend(loc='upper right')
    axes[1].plot(t, np.degrees(error[:, 1]), color='#2767ac')
    axes[1].set_ylabel('Heading error (degrees)')
    output = np.array([s['output'] for s in samples])
    axes[2].plot(t, output[:, 0]*100, label='Linear output (cm/s)')
    axes[2].plot(t, output[:, 1], label='Angular output (rad/s)')
    axes[2].set_ylabel('Final safety output')
    axes[2].set_xlabel('Gazebo simulation time (s)')
    axes[2].legend(loc='upper right')
    for phase in ('teleport_translation', 'teleport_rotation', 'scan_outage', 'scan_restored'):
        for ax in axes:
            ax.axvline(events[phase]['sim_s'], color='#888', linestyle=':', linewidth=1)
    fig.suptitle('Saved-map localization: translation / yaw teleport and sensor outage\nGround truth used only for scoring; odometry does not jump with teleport')
    fig.savefig(run/'localization.png', dpi=160)
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
