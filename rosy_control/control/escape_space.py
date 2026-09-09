"""Predict useful straight repositioning from observed geometry, not motor authority."""
import math
from numbers import Real
import numpy as np


def escape_space_plan(points, *, rotation_radius, center, pivot_radius,
                      fully_observed, can_rotate, forward_room, reverse_room):
    report = {'candidates': [], 'rotation_restored': False,
              'reason': 'geometry_unavailable'}
    try:
        values = (*center, rotation_radius, pivot_radius, forward_room, reverse_room)
        if (not all(isinstance(v, Real) and not isinstance(v, bool) and math.isfinite(v) for v in values)
                or min(rotation_radius, pivot_radius) <= 0
                or min(forward_room, reverse_room) < 0):
            return report
        obstacles = np.asarray(points)
        if (obstacles.ndim != 2 or obstacles.shape[1] != 2 or len(obstacles) < 3 or
                obstacles.dtype.kind not in 'fiu' or not np.isfinite(obstacles).all()):
            return report
        # Match the active rotation gate's reference frame and uncertainty.
        # A prediction never fills missing scan rays or authorizes a turn.
        cx, cy = center if fully_observed else (0., 0.)
        radius = pivot_radius if fully_observed else rotation_radius
        offsets = obstacles.astype(float)-np.array([cx, cy])
        current = float(np.hypot(offsets[:, 0], offsets[:, 1]).min())-radius
        report.update(current_rotation_clearance_m=current,
                      rotation_restored=bool(can_rotate),
                      reason='rotation_ready' if can_rotate else 'no_observed_improvement')
        if report['rotation_restored']:
            return report
        for direction, available in ((1, forward_room), (-1, reverse_room)):
            # Bound the local search, and reserve 5 mm beyond its destination
            # inside the independently checked straight-translation corridor.
            maximum = min(.08, max(0., available-.005))
            distances = np.arange(1, math.floor((maximum+1e-12)/.001)+1)*.001
            if not len(distances):
                continue
            predicted_values = np.hypot(offsets[:, 0, None]-direction*distances,
                                        offsets[:, 1, None]).min(axis=0)-radius
            best = None
            for distance, predicted in zip(distances.tolist(), predicted_values.tolist()):
                if predicted-current < .001:
                    continue
                candidate = {'direction': direction, 'target_m': distance,
                    'available_m': available, 'predicted_rotation_clearance_m': predicted,
                    'objective': 'restore_rotation' if predicted > .010 else 'improve_clearance'}
                if best is None or predicted > best['predicted_rotation_clearance_m']+1e-9:
                    best = candidate
                if predicted > .010:
                    best = candidate
                    break
            if best is not None:
                report['candidates'].append(best)
        report['candidates'].sort(key=lambda c: (
            c['objective'] != 'restore_rotation',
            c['target_m'] if c['objective'] == 'restore_rotation' else -c['predicted_rotation_clearance_m']))
        if report['candidates']:
            report['reason'] = 'observed_space_candidate'
        return report
    except (TypeError, ValueError, OverflowError):
        return report
