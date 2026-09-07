"""Conservative gap-based speed reduction, not a measured braking model."""
import math


def limit_for_space(v, w, ranges, profile):
    if not (v or w):
        return 0., 0., 'allow'
    front, rear, left, right = ranges
    required = [left, right]
    if v > 0 or w:
        required.append(front)
    if v < 0 or w:
        required.append(rear)
    if not all(math.isfinite(d) and d > 0 for d in required):
        return 0., 0., 'space_unknown'
    # Reduce speed across a 2 cm side-clearance band; retain the body floor.
    factor = min(1., max(0., (min(left, right)-profile.turn_clear)/.02))
    if v:
        distance = front if v > 0 else rear
        factor = min(factor, max(0., (distance-profile.stop)/(.5*profile.max_linear)))
    if w:
        factor = min(factor, max(0., (min(required)-profile.turn_clear)/.02))
    return v*factor, w*factor, 'space_limit' if factor < 1. else 'allow'
