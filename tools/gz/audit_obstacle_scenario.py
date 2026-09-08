"""Offline sensor/decision audit. Ground truth is evaluation-only."""
import collections
import gzip
import json
import math
from pathlib import Path
import sys


def measure(data):
    classes = collections.defaultdict(collections.Counter)
    reasons = collections.defaultdict(collections.Counter)
    separations, removed_positions = [], []
    actual_motion_samples = 0
    previous = None
    for row in data['rows']:
        stage, obs = row['stage'], row['observations']
        decision = obs.get('/safety/decision', {})
        reasons[stage][decision.get('reason', 'missing')] += 1
        box, robot = row['box'], row['robot']
        if box is not None and robot is not None:
            # Exact axis-aligned box distance from the model origin; not a
            # contact or calibrated body-clearance assertion.
            separations.append(math.hypot(max(abs(robot[0]-box[0])-.04, 0.),
                                          max(abs(robot[1]-box[1])-.04, 0.)))
            tracks = obs.get('/obstacles/tracks', {}).get('tracks', [])
            nearby = [t for t in tracks if t['observed'] and
                      math.dist(t['position'], box) <= .08]
            nearest = min(nearby, key=lambda t: math.dist(t['position'], box)) if nearby else None
            label = nearest['state'] if nearest else 'not_associated'
            classes[stage][label] += 1
            if previous is not None and previous['box'] is not None and math.dist(box, previous['box']) > .001:
                actual_motion_samples += 1
                classes['box_displaced'][label] += 1
        if stage == 'removed' and robot is not None:
            removed_positions.append(robot[:2])
        previous = row
    return dict(run_id=data.get('run_id'), stage=data['stage'], error=data['error'], events=data['events'],
                associated_classifications=dict(classes), safety_reasons=dict(reasons),
                scripted_displacement_samples=actual_motion_samples,
                min_model_origin_to_box_surface_m=min(separations) if separations else None,
                displacement_after_removal_m=(math.dist(removed_positions[0], removed_positions[-1])
                                             if len(removed_positions) > 1 else None),
                scope='Sampled diagnostic metrics; not contact-free or full-mapping acceptance')


if __name__ == '__main__':
    source = Path(sys.argv[1])
    raw = gzip.decompress(source.read_bytes()) if source.suffix == '.gz' else source.read_bytes()
    result = measure(json.loads(raw))
    source.with_name('obstacle-audit.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
