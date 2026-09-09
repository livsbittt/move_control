"""Canonical robot mode. One label. Subjects do not overlap.

Each subject answers ONE question; the /robot/mode label names that
answer. pick_mode priority, highest first:

  HAZARD   why the motors must be off or recovering (body endangerment)
           ESTOP PICK CLIFF TILT
  JUDGE    how the next move is being chosen (decision work, robot still)
           LOOK CALC RECON PAUSE
  CONTACT  what the nose touches or is about to touch (world geometry)
           WALL WARN
  MOTION   how the robot is moving (drive work)
           BACK ESCAPE TURN FWD
  IDLE     why nothing is happening (no job or disabled)
           WAIT STOP
"""
from dataclasses import dataclass

# US-016 echo beyond ~0.8 m is noise, not a nose contact; lidar beyond 8 m
# is the C1 trust horizon.
US_NOSE_MAX_M = 0.80
LIDAR_NOSE_MAX_M = 8.0


class Subject:
    HAZARD = 'hazard'
    JUDGE = 'judge'
    CONTACT = 'contact'
    MOTION = 'motion'
    IDLE = 'idle'


# One question per subject — the class's precise responsibility; the LCD
# and web render it next to the mode group.
SUBJECT_ROLE = {
    Subject.HAZARD: 'why the motors must be off or recovering',
    Subject.JUDGE: 'how the next move is being chosen',
    Subject.CONTACT: 'what the nose touches or is about to touch',
    Subject.MOTION: 'how the robot is moving',
    Subject.IDLE: 'why nothing is happening',
}

# Display order of the subject groups.
SUBJECTS_ORDER = (Subject.HAZARD, Subject.JUDGE, Subject.CONTACT,
                  Subject.MOTION, Subject.IDLE)


@dataclass(frozen=True)
class Mode:
    name: str
    subject: str
    meaning: str
    action: str


# Order inside a subject is display/priority, not execution order.
# Section comments mark each subject's precise slice of the taxonomy:
#   body endangerment | decision | nose-vs-world | drive | no job
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
    Mode('ESCAPE', Subject.MOTION, 'spin to the inspected exit',
         'maze corner or camera obstacle; not a bumper wall'),
    Mode('TURN', Subject.MOTION, 'spin after a cliff', 'fixed angle, locked sign'),
    Mode('FWD', Subject.MOTION, 'path open', 'drive and hug'),
    Mode('WAIT', Subject.IDLE, 'IR not ready', 'hold'),
    Mode('STOP', Subject.IDLE, 'wander disabled', 'hold'),
)

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

# Wander states that own the label outright; only 'forward' falls through
# to the contact bands.
NON_FORWARD_STATES = frozenset(WANDER_TO_MODE) - {'forward'}


def by_subject():
    """Modes grouped by subject in display order: [(subject, [names])]."""
    out = []
    for subj in SUBJECTS_ORDER:
        names = [m.name for m in MODES if m.subject == subj]
        if names:
            out.append((subj, names))
    return out


def nose_range(front, us):
    """Closer of lidar front and a real US echo. Ignore US no-echo ~0.97 m."""
    hits = []
    if _hit(front, LIDAR_NOSE_MAX_M):
        hits.append(front)
    if _hit(us, US_NOSE_MAX_M):
        hits.append(us)
    return min(hits) if hits else float('inf')


def nose_on_wall(front, us, wall_front=0.08):
    """Nose contact from lidar/US only — one definition for label and FSM.

    US counts only under US_NOSE_MAX_M: a long US-016 echo must not read as
    a wall. Camera obstacle is a corner (ESCAPE), not a wall.
    """
    wall = float(wall_front or 0.08)
    us_wall = (
        _hit(us, US_NOSE_MAX_M) and float(us) < US_NOSE_MAX_M and float(us) <= wall
    )
    return us_wall or (
        _hit(front, LIDAR_NOSE_MAX_M) and float(front) <= wall
    )


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
    """One mode label: HAZARD, then JUDGE, then CONTACT bands, then FWD.

    Each subject class answers exactly one question and the chain walks
    the questions in authority order: the body (hazard) always wins, a
    non-forward wander state means decision work owns the label, contact
    bands apply only while actually driving forward.
    """
    if estop:
        return 'ESTOP'
    if pickup:
        return 'PICK'
    if cliff:
        return 'CLIFF'
    if tilt:
        return 'TILT'
    w = (wander_state or 'stop').strip().lower()
    # Non-forward states own the label outright; only forward falls through
    # to the contact bands.
    if w in NON_FORWARD_STATES:
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
