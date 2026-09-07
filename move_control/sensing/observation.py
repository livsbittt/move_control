"""Per-stream evidence clocks. Repeated reads never manufacture new samples."""
import math
from dataclasses import dataclass


@dataclass
class Observation:
    generation: int = 0
    received: float | None = None
    source: float | None = None
    source_age: float = 0.
    valid: bool = False
    reason: str = 'missing'


class Observations:
    def __init__(self, max_age=1., future_tolerance=.1):
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.rows = {}

    def add(self, name, received, *, source=None, source_now=None, valid=True):
        row = self.rows.setdefault(name, Observation())
        if not math.isfinite(received) or not valid:
            row.valid, row.reason = False, 'invalid'
            return False
        age = 0.
        if source is not None:
            if (source_now is None or not math.isfinite(source) or
                    not math.isfinite(source_now) or source <= 0.):
                row.valid, row.reason = False, 'invalid_stamp'
                return False
            age = source_now - source
            if not -self.future_tolerance <= age <= self.max_age:
                row.valid, row.reason = False, 'source_stale'
                return False
            if row.source is not None and source <= row.source:
                # Keep the original deadline. A duplicate is not a heartbeat.
                return False
        if row.received is not None and received < row.received:
            row.valid, row.reason = False, 'clock_reversed'
            return False
        row.generation += 1
        row.received, row.source, row.source_age = received, source, max(0., age)
        row.valid, row.reason = True, 'valid'
        return True

    def generation(self, name):
        row = self.rows.get(name)
        return row.generation if row else 0

    def fresh(self, name, now):
        row = self.rows.get(name)
        return bool(row and row.valid and row.received is not None and
                    math.isfinite(now) and 0 <= now-row.received and
                    now-row.received+row.source_age <= self.max_age)

    def report(self, now):
        return {name: {
            'generation': row.generation,
            'valid': self.fresh(name, now),
            'status': 'valid' if self.fresh(name, now) else (row.reason if not row.valid else 'stale'),
            'clock': 'source_and_receive' if row.source is not None else 'receive_only',
            'age_s': max(0., now-row.received+row.source_age) if row.received is not None and math.isfinite(now) else None,
        } for name, row in self.rows.items()}
