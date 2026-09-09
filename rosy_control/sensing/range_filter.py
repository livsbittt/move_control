"""Short median window for calibration measurements, never obstacle braking."""
import math
from statistics import median


class CalibrationRangeFilter:
    def __init__(self):
        self.rows = []

    def update(self, value, now, valid=True):
        if not valid or not math.isfinite(value):
            self.rows.clear()
            return value
        self.rows = [(t, v) for t, v in self.rows if 0 <= now-t <= .4]
        self.rows.append((now, value))
        self.rows = self.rows[-5:]
        return median(v for _, v in self.rows)
