"""Stuck / escape / backup / hazard response, narrow-squeeze policy, turn
signs. Pure — no ROS."""
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


def narrow_factor(clear, comfort):
    """Open-speed headroom left at the measured clearance: 1.0 open, → 0 tight.

    clear ≥ comfort (wander param narrow_comfort_m) → 1.0: open-space behavior.
    None/NaN → 1.0 (open, never slow on a broken reading).
    """
    c = float(comfort)
    if c <= 1e-6:
        return 1.0
    try:
        f = float(clear) / c
    except (TypeError, ValueError):
        return 1.0
    if f != f:
        return 1.0
    return max(0.0, min(1.0, f))


def frontier_gate(factor, wall, open_gate):
    """Forward gate for a last-seen frontier, on the narrow factor.

    Open space wants 0.16 m of visible run; a tight corridor needs only the
    wall_front band (0.08) — anything below pauses as on_wall anyway, so the
    floor never admits a closer-than-wall drive. Factor is clamped.
    """
    f = max(0.0, min(1.0, float(factor)))
    return float(wall) + (float(open_gate) - float(wall)) * f


def front_block(front, clear):
    """Forward guard: block driving while the nose arc reads < clear.

    The gz rig wedge ran full command into a wall corner with the scan
    already showing 7 cm — the driver had the lidar and never used it.
    inf (no return = open ahead) and NaN (broken reading) are open, same
    never-block-on-broken-sensor convention as narrow_factor.
    """
    try:
        f = float(front)
    except (TypeError, ValueError):
        return False
    if f != f or f == float('inf'):
        return False
    if f < 0.0:
        return False
    return f < float(clear)


def escape_open(front, clear):
    """Escape resume gate: forward again only on a confirmed nose gap.

    The rig's blind flee re-wedged corners; the replacement spins until the
    nose arc actually shows clear space. inf = open ahead = confirmed. NaN
    is no confirmation — keep spinning.
    """
    try:
        f = float(front)
    except (TypeError, ValueError):
        return False
    if f != f:
        return False
    return f >= float(clear)


def guard_speed(front, clear, v, hyst=0.08):
    """Proportional forward cap on the nose-arc clearance.

    Full v at clear+hyst and beyond, hard 0 at/below clear, linear crawl
    between. The binary block fought pure-pursuit at full command — measured
    on the gz rig: mean command 0.157 m/s against 3.7 cm/s actual, a standing
    wall-skim grind; scaling speed by clearance lets the guard steering win
    before contact. inf/NaN = open/broken reading → v (never cap on a broken
    sensor, same convention as narrow_factor).
    """
    try:
        f = float(front)
    except (TypeError, ValueError):
        return float(v)
    if f != f or f < 0.0 or f == float('inf'):
        return float(v)
    if f <= float(clear):
        return 0.0
    return float(v) * min(1.0, (f - float(clear)) / float(hyst))


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
