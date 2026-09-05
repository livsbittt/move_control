"""Canonical robot mode. One label. Subjects do not overlap.

  hazard   body: ESTOP PICK CLIFF TILT
  contact  nose vs world: WALL WARN
  judge    decide: LOOK PAUSE
  motion   move: BACK ESCAPE TURN FWD
  idle     WAIT STOP
"""
from dataclasses import dataclass


class Subject:
    HAZARD = 'hazard'
    CONTACT = 'contact'
    JUDGE = 'judge'
    MOTION = 'motion'
    IDLE = 'idle'


@dataclass(frozen=True)
class Mode:
    name: str
    subject: str
    meaning: str
    action: str


# Order inside a subject is display/priority, not execution order.
MODES = (
    Mode('ESTOP', Subject.HAZARD, 'e-stop latched', 'motors 0'),
    Mode('PICK', Subject.HAZARD, 'robot lifted (IMU)', 'motors 0'),
    Mode('CLIFF', Subject.HAZARD, 'IR sees no floor', 'backup then turn'),
    Mode('TILT', Subject.HAZARD, 'IMU tilt/gyro', 'backup if rear clear else stop'),
    Mode('WALL', Subject.CONTACT, 'nose on bumper: lidar or US ≤ wall_front',
         'look, then backup if rear clear else escape; no forward'),
    Mode('WARN', Subject.CONTACT, 'close but not contact: wall_front < d ≤ warn_front',
         'crawl forward'),
    Mode('LOOK', Subject.JUDGE, 'stopped, sample L/R/F', 'gather medians'),
    Mode('CALC', Subject.JUDGE, 'score openings from look samples',
         'pick forward, backup, or a locked turn'),
    Mode('RECON', Subject.JUDGE, 'opening was wrong while turning',
         'stop, look again, new plan'),
    Mode('PAUSE', Subject.JUDGE, 'brief stop', 'tell cliff vs wall'),
    Mode('BACK', Subject.MOTION, 'reversing', 'short reverse, rear must be clear'),
    Mode('ESCAPE', Subject.MOTION, 'spin to the locked opening',
         'maze corner or camera obstacle; not a bumper wall'),
    Mode('TURN', Subject.MOTION, 'spin after a cliff', 'fixed angle, locked sign'),
    Mode('FWD', Subject.MOTION, 'path open', 'drive and hug'),
    Mode('WAIT', Subject.IDLE, 'IR not ready', 'hold'),
    Mode('STOP', Subject.IDLE, 'wander disabled', 'hold'),
)

BY_NAME = {m.name: m for m in MODES}

# Wander finite-state names → mode label (action/idle). Hazards overlay on top.
WANDER_TO_MODE = {
    'forward': 'FWD',
    'pause': 'PAUSE',
    'look': 'LOOK',
    'calc': 'CALC',
    'recon': 'RECON',
    'wall': 'WALL',
    'backup': 'BACK',
    'turn': 'TURN',
    'escape': 'ESCAPE',
    'wait': 'WAIT',
    'stop': 'STOP',
}


def nose_range(front, us):
    """Closer of lidar front and a real US echo. Ignore US no-echo ~0.97 m."""
    hits = []
    if _hit(front, 8.0):
        hits.append(front)
    if _hit(us, 0.80):
        hits.append(us)
    return min(hits) if hits else float('inf')


def _hit(d, hi):
    try:
        x = float(d)
    except (TypeError, ValueError):
        return False
    return x == x and 0.0 < x <= hi


def pick_mode(
    pickup=False,
    cliff=False,
    tilt=False,
    estop=False,
    wander_state='stop',
    on_wall=False,
    front=float('inf'),
    us=float('inf'),
    wall_front=0.08,
    warn_front=0.11,
) -> str:
    """One mode. Hazard, then wander action, then contact while still driving."""
    if estop:
        return 'ESTOP'
    if pickup:
        return 'PICK'
    if cliff:
        return 'CLIFF'
    if tilt:
        return 'TILT'
    w = (wander_state or 'stop').strip().lower()
    if w in ('look', 'calc', 'recon', 'pause', 'wall', 'backup', 'turn', 'escape', 'wait', 'stop'):
        return WANDER_TO_MODE[w]
    # forward (or unknown): contact bands
    if on_wall:
        return 'WALL'
    d = nose_range(front, us)
    lo = float(wall_front or 0.08)
    hi = float(warn_front or 0.11)
    if d == d and lo < d <= hi:
        return 'WARN'
    return WANDER_TO_MODE.get(w, 'FWD')
