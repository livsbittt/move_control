"""Bound contact retries by displacement, not repeated forward attempts."""
import math


class RecoveryBudget:
    def __init__(self, limit=3, reset_distance=.15):
        self.limit = limit
        self.reset_distance = reset_distance
        self.anchor = None
        self.count = 0

    def attempt(self, x, y):
        if not (math.isfinite(x) and math.isfinite(y)):
            self.count += 1
            return self.count <= self.limit
        if self.anchor is None or math.hypot(
                x - self.anchor[0], y - self.anchor[1]) >= self.reset_distance:
            self.anchor = (x, y)
            self.count = 0
        self.count += 1
        return self.count <= self.limit
