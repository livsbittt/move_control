"""Stuck / escape / backup / hazard response and turn-sign rules. Pure — no ROS."""
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


def is_stuck_motion(moved, dt, v_cmd, stuck_m, stuck_sec):
    """Stall only if we asked to move more than stuck_m and did not.

    A think-speed crawl (3 mm/s) cannot cover 8 mm in 1.2 s — that is not stuck.
    """
    if float(dt) < float(stuck_sec):
        return False
    if float(moved) >= float(stuck_m):
        return False
    expected = abs(float(v_cmd)) * float(dt)
    if expected <= float(stuck_m):
        return False
    return True


def escape_may_abort(
    turned_rad, on_wall, blocked, min_rad=ESCAPE_MIN_TURN, pinched=False
):
    if blocked or on_wall or pinched:
        return False
    return float(turned_rad) >= float(min_rad)


def escape_may_desense(elapsed, turned_rad, from_stuck):
    """Desense only a real false escape, never a stuck timeout."""
    if from_stuck:
        return False
    return float(elapsed) < 0.85 and float(turned_rad) < math.radians(20.0)


def hazard_action(tilt, cliff, seen_forward, can_reverse):
    """One cliff/tilt answer for every FSM state.

    backup — hazard trusted (tilt always; cliff only after we drove once)
             and the tail is free.
    turn   — hazard and reverse illegal: spin is the only legal move
             (safety halts forward on cliff/tilt but never zeroes the spin).
    look   — cliff before any forward drive is untrusted (IR floor band may
             be unready, not a hole): stop and re-look, never blind-reverse.
    none   — no hazard.
    """
    if not (tilt or cliff):
        return 'none'
    if not can_reverse:
        return 'turn'
    if tilt or seen_forward:
        return 'backup'
    return 'look'


def wall_first_move(can_reverse, backed):
    """First move against a wall: reverse off it if the tail is clear and we
    have not just backed, else the wall becomes a spin (escape)."""
    return 'backup' if (can_reverse and not backed) else 'escape'


def side_sign(side, gate=0.30):
    """Turn-sign from the camera side score. ±1.0; 0.0 = no call. NaN-safe."""
    try:
        s = float(side)
    except (TypeError, ValueError):
        return 0.0
    if s != s or abs(s) < float(gate):
        return 0.0
    return 1.0 if s > 0.0 else -1.0


def ratio_sign(left, right, ratio=1.15):
    """Turn-sign from side distances: the wider side wins. No-echo = no call."""
    try:
        l, r = float(left), float(right)
    except (TypeError, ValueError):
        return 0.0
    if l != l or r != r or l <= 0.0 or r <= 0.0:
        return 0.0
    if l > r * float(ratio):
        return 1.0
    if r > l * float(ratio):
        return -1.0
    return 0.0
