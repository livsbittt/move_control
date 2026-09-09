"""Use the conservative body bound when exact pivot coverage is unavailable."""
import math


def rotation_clearance_allowed(conservative, fully_observed, fresh, pivot_margin, ranges):
    if not fresh:
        return False
    if not fully_observed:
        # The conservative check already uses the learned swept radius,
        # configured margin, nearest return and all required range sectors.
        return bool(conservative)
    return (pivot_margin is not None and math.isfinite(pivot_margin) and pivot_margin > .010
            and all(math.isfinite(v) and v > 0 for v in ranges))
