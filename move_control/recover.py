"""Stuck / escape / backup policy. Pure functions — no ROS."""
import math

ESCAPE_MIN_TURN = math.radians(45.0)
STUCK_CLEAR_M = 0.03


def backup_limit_m(rear, stop=0.018, cap=0.12):
    """Reverse as far as the free rear gap allows. Not a 45% snippet."""
    try:
        r = float(rear)
    except (TypeError, ValueError):
        return min(float(cap), 0.02)
    if r != r or r < 0.0 or r > 10.0:
        return min(float(cap), 0.02)
    gap = max(0.0, r - float(stop))
    return max(0.01, min(float(cap), gap))


def have_turn_space(front, side, tail, clear):
    return (
        float(front) >= float(clear)
        and float(side) >= float(clear)
        and float(tail) >= float(clear)
    )


def need_space_to_turn(
    front, side, tail, clear, can_reverse, turn_backs, max_backs
):
    if not can_reverse or int(turn_backs) >= int(max_backs):
        return False
    c = float(clear)
    return float(front) < c or float(side) < c or float(tail) < c


def stuck_kind(can_reverse):
    return 'backup' if can_reverse else 'escape'


def stuck_flip(stuck_n):
    return int(stuck_n) >= 2


def escape_may_abort(turned_rad, on_wall, blocked, min_rad=ESCAPE_MIN_TURN):
    if blocked or on_wall:
        return False
    return float(turned_rad) >= float(min_rad)


def escape_may_desense(elapsed, turned_rad, from_stuck):
    """Desense only a real false escape, never a stuck timeout."""
    if from_stuck:
        return False
    return float(elapsed) < 0.85 and float(turned_rad) < math.radians(20.0)


def gate_keep_angular(wz, vx=0.0, cliff=False, tilt=False):
    """Cliff/tilt must not zero spin. If reverse is illegal, spin is the move."""
    del vx, cliff, tilt
    return wz
