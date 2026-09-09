"""Bounded polar scan alignment for near-pure rotation; reject ambiguity."""
import math
import numpy as np


def percentile80(values):
    """NumPy's default linear 80th percentile without generic quantile overhead."""
    index = (len(values)-1)*.8
    lower, upper = int(index), math.ceil(index)
    ordered = np.partition(values, (lower, upper))
    return float(ordered[lower]+(ordered[upper]-ordered[lower])*(index-lower))


def scan_rotation(reference, current, increment, diagnostic=None):
    if diagnostic is not None:
        diagnostic.clear()
    def reject(reason):
        if diagnostic is not None:
            diagnostic['reason'] = reason
        return None
    a, b = np.asarray(reference, dtype=float), np.asarray(current, dtype=float)
    if (a.ndim != 1 or a.shape != b.shape or not 360 <= a.size <= 1440 or
            not math.isfinite(increment) or increment <= 0 or
            abs(a.size*increment-2*math.pi) > .02):
        return reject('invalid_scan_geometry')
    limit = int(math.radians(20)/increment)
    observed_a = np.isfinite(a) & (a > .05) & (a < 8)
    observed_b = np.isfinite(b) & (b > .05) & (b < 8)
    support_a, support_b = np.count_nonzero(observed_a), np.count_nonzero(observed_b)
    if diagnostic is not None:
        diagnostic.update(bins=int(a.size), reference_observed=int(support_a), current_observed=int(support_b), minimum_observed=.9*a.size)
    if min(support_a, support_b) < .9*a.size:
        return reject('insufficient_observed_returns')
    # Compare common support to the returns that actually exist. Unrelated
    # search shifts move dropout gaps apart; their weak overlap must not veto
    # an otherwise unique, densely observed alignment at the correct shift.
    required_support = .9*max(support_a, support_b)
    scores = []
    maximum_common = 0
    for shift in range(-limit, limit+1):
        aligned = np.roll(b, shift)
        valid = np.isfinite(a) & np.isfinite(aligned) & (a > .05) & (aligned > .05) & (a < 8) & (aligned < 8)
        common = np.count_nonzero(valid)
        maximum_common = max(maximum_common, common)
        if common < required_support:
            continue
        error = np.abs(a[valid]-aligned[valid])
        scores.append((percentile80(error), shift))
    if diagnostic is not None:
        diagnostic.update(required_common=required_support, maximum_common=int(maximum_common), qualifying_candidates=len(scores))
    if not scores:
        return reject('insufficient_common_returns')
    best = min(scores)
    competing = [score for score, shift in scores if abs(shift-best[1])*increment >= math.radians(3)]
    if diagnostic is not None:
        diagnostic.update(best_shift=best[1], residual_m=best[0], separation_m=min(competing)-best[0] if competing else None)
    if not competing:
        return reject('insufficient_competing_support')
    if best[0] > .015:
        return reject('excessive_residual')
    if min(competing)-best[0] < .003:
        return reject('ambiguous_alignment')
    if abs(best[1]) >= limit:
        return reject('search_boundary')
    if diagnostic is not None:
        diagnostic['reason'] = 'ok'
    return {'yaw': best[1]*increment, 'residual_m': best[0],
            'separation_m': min(competing)-best[0], 'resolution_rad': increment}
