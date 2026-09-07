"""Bounded polar scan alignment for near-pure rotation; reject ambiguity."""
import math
import numpy as np


def scan_rotation(reference, current, increment):
    a, b = np.asarray(reference, dtype=float), np.asarray(current, dtype=float)
    if (a.ndim != 1 or a.shape != b.shape or not 360 <= a.size <= 1440 or
            not math.isfinite(increment) or increment <= 0 or
            abs(a.size*increment-2*math.pi) > .02):
        return None
    limit = int(math.radians(20)/increment)
    scores = []
    for shift in range(-limit, limit+1):
        aligned = np.roll(b, shift)
        valid = np.isfinite(a) & np.isfinite(aligned) & (a > .05) & (aligned > .05) & (a < 8) & (aligned < 8)
        if np.count_nonzero(valid) < .9*a.size:
            return None
        error = np.abs(a[valid]-aligned[valid])
        scores.append((float(np.quantile(error, .8)), shift))
    best = min(scores)
    competing = [score for score, shift in scores if abs(shift-best[1])*increment >= math.radians(3)]
    if (not competing or best[0] > .015 or min(competing)-best[0] < .003 or
            abs(best[1]) >= limit):
        return None
    return {'yaw': best[1]*increment, 'residual_m': best[0],
            'separation_m': min(competing)-best[0], 'resolution_rad': increment}
