#!/usr/bin/env python3
import math
import unittest

from rosy_control.control.recover import (
    ESCAPE_MIN_TURN,
    ExitSteer,
    backup_limit_m,
    escape_may_abort,
    escape_may_desense,
    escape_open,
    frontier_gate,
    front_block,
    guard_speed,
    have_turn_space,
    hazard_action,
    is_stuck_motion,
    narrow_factor,
    need_space_to_turn,
    ratio_sign,
    side_sign,
    stuck_flip,
    stuck_kind,
    turn_toward_sign,
    wall_first_move,
)


class RecoverTest(unittest.TestCase):
    def test_stuck_forces_backup_or_escape_not_forward(self):
        self.assertEqual(stuck_kind(True), 'backup')
        self.assertEqual(stuck_kind(False), 'escape')
        self.assertNotEqual(stuck_kind(True), 'forward')
        self.assertNotEqual(stuck_kind(False), 'look')

    def test_stuck_flips_sign_on_second(self):
        self.assertFalse(stuck_flip(1))
        self.assertTrue(stuck_flip(2))
        self.assertTrue(stuck_flip(3))

    def test_backup_uses_full_rear_gap(self):
        # Old 0.45 * (0.12-0.018) = 4.6cm. Now the whole gap, capped.
        self.assertAlmostEqual(backup_limit_m(0.12, stop=0.018, cap=0.12), 0.102, places=3)
        self.assertAlmostEqual(backup_limit_m(0.05, stop=0.018, cap=0.12), 0.032, places=3)
        self.assertAlmostEqual(backup_limit_m(0.40, stop=0.018, cap=0.12), 0.12, places=3)
        self.assertAlmostEqual(backup_limit_m(float('inf'), cap=0.12), 0.02, places=3)

    def test_turn_space_includes_tail(self):
        clear = 0.086
        self.assertTrue(have_turn_space(0.20, 0.20, 0.20, clear))
        self.assertFalse(have_turn_space(0.20, 0.20, 0.04, clear))
        self.assertTrue(
            need_space_to_turn(0.20, 0.20, 0.04, clear, True, 0, 2)
        )
        self.assertFalse(
            need_space_to_turn(0.20, 0.20, 0.20, clear, True, 0, 2)
        )
        self.assertFalse(
            need_space_to_turn(0.04, 0.04, 0.04, clear, False, 0, 2)
        )
        self.assertFalse(
            need_space_to_turn(0.04, 0.04, 0.04, clear, True, 2, 2)
        )

    def test_crawl_is_not_stuck(self):
        stuck_m, stuck_sec = 0.008, 1.2
        # think 3 mm/s cannot cover 8 mm in 1.2 s
        self.assertFalse(is_stuck_motion(0.0036, 1.2, 0.003, stuck_m, stuck_sec))
        # cruise commanded, chassis did not move
        self.assertTrue(is_stuck_motion(0.001, 1.2, 0.014, stuck_m, stuck_sec))
        # actually moved
        self.assertFalse(is_stuck_motion(0.010, 1.2, 0.014, stuck_m, stuck_sec))
        # window not elapsed
        self.assertFalse(is_stuck_motion(0.0, 0.5, 0.014, stuck_m, stuck_sec))

    def test_narrow_factor_curves_open_to_tight(self):
        comfort = 0.10
        # open: no corridor (inf) or clearance at comfort → 1.0
        self.assertEqual(narrow_factor(float('inf'), comfort), 1.0)
        self.assertEqual(narrow_factor(comfort, comfort), 1.0)
        # NaN/None → open: never slow on a broken reading
        self.assertEqual(narrow_factor(float('nan'), comfort), 1.0)
        self.assertEqual(narrow_factor(None, comfort), 1.0)
        self.assertAlmostEqual(narrow_factor(0.048, comfort), 0.48, places=3)
        # zero/negative clearance: think-speed floor
        self.assertEqual(narrow_factor(0.0, comfort), 0.0)
        self.assertEqual(narrow_factor(-0.01, comfort), 0.0)

    def test_narrow_factor_ignores_zero_comfort(self):
        # comfort ≤ 0 would divide by ~0; treat as "always open".
        self.assertEqual(narrow_factor(0.048, 0.0), 1.0)

    def test_frontier_gate_lowers_to_wall_band_when_tight(self):
        # open = 0.16 as today; full narrow floors at the wall_front band
        # (below it on_wall pauses first, so the floor admits nothing closer).
        self.assertAlmostEqual(frontier_gate(1.0, 0.08, 0.16), 0.16, places=3)
        self.assertAlmostEqual(frontier_gate(0.0, 0.08, 0.16), 0.08, places=3)
        self.assertAlmostEqual(frontier_gate(0.5, 0.08, 0.16), 0.12, places=3)
        self.assertAlmostEqual(frontier_gate(2.0, 0.08, 0.16), 0.16, places=3)

    def test_escape_abort_needs_45deg_and_not_on_wall(self):
        self.assertFalse(escape_may_abort(math.radians(13.0), False, False))
        self.assertTrue(escape_may_abort(ESCAPE_MIN_TURN, False, False))
        self.assertFalse(escape_may_abort(ESCAPE_MIN_TURN, True, False))
        self.assertFalse(escape_may_abort(ESCAPE_MIN_TURN, False, True))
        self.assertFalse(
            escape_may_abort(ESCAPE_MIN_TURN, False, False, pinched=True)
        )

    def test_timeout_does_not_desense_stuck(self):
        # 5s / 13° used to call _escape_false. Must not, especially when stuck.
        self.assertFalse(escape_may_desense(5.0, math.radians(13.0), True))
        self.assertFalse(escape_may_desense(5.0, math.radians(13.0), False))
        self.assertTrue(escape_may_desense(0.50, math.radians(5.0), False))
        self.assertFalse(escape_may_desense(0.50, math.radians(5.0), True))

    def test_think_rate_cannot_pass_15deg_in_5s(self):
        w_think = 0.045
        self.assertLess(w_think * 5.0, math.radians(15.0))
        wturn = 0.10
        self.assertGreater(wturn * 8.0, ESCAPE_MIN_TURN)

    def test_hazard_action_matrix(self):
        # No hazard → none.
        self.assertEqual(hazard_action(False, False, True, True), 'none')
        # Tilt is always trusted: backup if the tail is free, else spin.
        self.assertEqual(hazard_action(True, False, False, True), 'backup')
        self.assertEqual(hazard_action(True, False, False, False), 'turn')
        # Cliff is trusted only after the first forward drive (IR settle).
        self.assertEqual(hazard_action(False, True, True, True), 'backup')
        self.assertEqual(hazard_action(False, True, False, True), 'look')
        self.assertEqual(hazard_action(False, True, True, False), 'turn')
        self.assertEqual(hazard_action(False, True, False, False), 'turn')

    def test_wall_first_move(self):
        # Reverse off the wall once if the tail is clear; after that, spin.
        self.assertEqual(wall_first_move(True, False), 'backup')
        self.assertEqual(wall_first_move(True, True), 'escape')
        self.assertEqual(wall_first_move(False, False), 'escape')

    def test_side_sign_gates_small_scores(self):
        # ±0.3 camera side gate; inside the gate is no call.
        self.assertEqual(side_sign(0.3), 1.0)
        self.assertEqual(side_sign(-0.3), -1.0)
        self.assertEqual(side_sign(0.29), 0.0)
        self.assertEqual(side_sign(0.0), 0.0)
        self.assertEqual(side_sign(float('nan')), 0.0)

    def test_ratio_sign_wider_side_wins(self):
        # 1.15 ratio; near-equal sides give no opinion, None = no-echo.
        self.assertEqual(ratio_sign(0.40, 0.30), 1.0)
        self.assertEqual(ratio_sign(0.30, 0.40), -1.0)
        self.assertEqual(ratio_sign(0.40, 0.36), 0.0)
        self.assertEqual(ratio_sign(None, 0.40), 0.0)
        self.assertEqual(ratio_sign(0.40, None), 0.0)

    def test_front_block_below_clear(self):
        # Wall inside the nose arc → forward blocked.
        self.assertTrue(front_block(0.07, 0.12))
        self.assertTrue(front_block(0.119, 0.12))

    def test_front_block_at_or_over_clear(self):
        # At the threshold and beyond, driving is allowed.
        self.assertFalse(front_block(0.12, 0.12))
        self.assertFalse(front_block(0.30, 0.12))

    def test_front_block_broken_reading_is_open(self):
        # inf = no return (open ahead); NaN = broken reading. Repo
        # convention (narrow_factor): never block on a broken sensor.
        self.assertFalse(front_block(float('inf'), 0.12))
        self.assertFalse(front_block(float('nan'), 0.12))

    def test_escape_open_requires_confirmed_gap(self):
        # Resume forward only on a confirmed opening at the nose.
        self.assertTrue(escape_open(0.20, 0.18))
        self.assertTrue(escape_open(float('inf'), 0.18))
        self.assertFalse(escape_open(0.12, 0.18))
        # NaN = no reading: keep spinning, keep waiting, never a blind resume.
        self.assertFalse(escape_open(float('nan'), 0.18))

    def test_guard_speed_proportional(self):
        # Full speed well clear of the band, zero at/below the hard limit.
        self.assertEqual(front_block(0.07, 0.12), True)
        self.assertEqual(guard_speed(0.30, 0.12, 0.18), 0.18)
        self.assertEqual(guard_speed(0.20, 0.12, 0.18), 0.18)
        self.assertEqual(guard_speed(0.12, 0.12, 0.18), 0.0)
        self.assertEqual(guard_speed(0.07, 0.12, 0.18), 0.0)
        # Linear crawl inside the band: midpoint of 0.12..0.20 = half speed.
        self.assertAlmostEqual(guard_speed(0.16, 0.16 - 0.04, 0.18), 0.09)
        self.assertAlmostEqual(guard_speed(0.14, 0.12, 0.18), 0.045)
        # inf (open ahead) and NaN (broken reading) never cap — repo
        # convention: never slow on a broken sensor.
        self.assertEqual(guard_speed(float('inf'), 0.12, 0.18), 0.18)
        self.assertEqual(guard_speed(float('nan'), 0.12, 0.18), 0.18)


class ExitSteerTest(unittest.TestCase):
    """Exit steering: the escape spin judges from safety's full-circle exit
    (/safety/exit_yaw/range) instead of a blind fixed sign, and never repeats
    a direction that already failed. Pure drivers for wander's stuck response.
    """

    def test_turn_toward_sign_points_at_the_gap(self):
        # Gap at +90° (left) → CCW (+1); gap at −90° (right) → CW (−1).
        self.assertEqual(turn_toward_sign(math.radians(90)), 1.0)
        self.assertEqual(turn_toward_sign(math.radians(-90)), -1.0)

    def test_turn_toward_sign_wraps_the_short_way(self):
        # +190° wraps to −170°: a 190° CCW grind is not the short way.
        self.assertEqual(turn_toward_sign(math.radians(190)), -1.0)
        self.assertEqual(turn_toward_sign(math.radians(-190)), 1.0)

    def test_turn_toward_sign_no_call_inside_deadband_or_broken(self):
        # Inside the deadband there is nothing to steer (aligned-ish; the
        # resume gate decides); NaN/inf/None never steer — repo convention.
        self.assertEqual(turn_toward_sign(math.radians(5), deadband=math.radians(8)), 0.0)
        self.assertEqual(turn_toward_sign(math.radians(9), deadband=math.radians(8)), 1.0)
        self.assertEqual(turn_toward_sign(float('nan')), 0.0)
        self.assertEqual(turn_toward_sign(float('inf')), 0.0)
        self.assertEqual(turn_toward_sign(None), 0.0)

    def test_exit_latch_shortest_sign_and_target(self):
        # Rear exit (−170°): the shortest way is CW, not a 190° grind.
        e = ExitSteer()
        s = e.latch(0.0, 0.0, 0.0, math.radians(-170), 0.30,
                    min_range=0.08, elapsed_s=0.0)
        self.assertEqual(s, -1.0)
        self.assertAlmostEqual(e.target, math.radians(-170), places=6)
        e = ExitSteer()
        s = e.latch(0.0, 0.0, 0.0, math.radians(90), 0.30,
                    min_range=0.08, elapsed_s=0.0)
        self.assertEqual(s, 1.0)
        self.assertAlmostEqual(e.target, math.radians(90), places=6)

    def test_exit_latch_refuses_small_or_invalid_gaps(self):
        # Below the escape_front floor there is no gap worth steering to.
        e = ExitSteer()
        s = e.latch(0.0, 0.0, 0.0, math.radians(90), 0.05,
                    min_range=0.08, elapsed_s=0.0)
        self.assertEqual(s, 0.0)
        self.assertIsNone(e.target)
        # inf = safety dead / no judge yet → no call, legacy spin.
        e = ExitSteer()
        s = e.latch(0.0, 0.0, 0.0, 0.0, float('inf'),
                    min_range=0.08, elapsed_s=0.0)
        self.assertEqual(s, 0.0)
        self.assertIsNone(e.target)

    def test_exit_latch_refuses_benched_bearing(self):
        e = ExitSteer()
        e.bench(0.0, 0.0, math.radians(90))
        s = e.latch(0.0, 0.0, 0.0, math.radians(90), 0.30,
                    min_range=0.08, elapsed_s=0.0)
        self.assertEqual(s, 0.0)
        self.assertIsNone(e.target)

    def test_exit_steer_drives_toward_latched_target(self):
        e = ExitSteer()
        e.latch(0.0, 0.0, 0.0, math.radians(90), 0.30,
                min_range=0.08, elapsed_s=0.0)
        self.assertEqual(
            e.steer(0.0, 0.0, 0.0, math.radians(90), 0.30, 0.1), 1.0
        )
        # 5° left of facing the target: inside the 8° deadband → hold sign.
        self.assertEqual(
            e.steer(0.0, 0.0, math.radians(85), math.radians(5), 0.30, 0.2),
            1.0,
        )

    def test_exit_steer_debounces_flip_requests(self):
        # A re-latch jump must not jitter the spin: the opposite sign must
        # persist 3 ticks before the spin flips.
        e = ExitSteer()
        e.latch(0.0, 1.0, 0.0, math.radians(90), 0.30,
                min_range=0.08, elapsed_s=0.0)
        self.assertEqual(e.steer(0.0, 1.0, 0.0, math.radians(90), 0.30, 0.1), 1.0)
        # Target jumps to −90°: opposite desired sign. Ticks 1-2 hold, 3 flips.
        e.target = math.radians(-90)
        self.assertEqual(e.steer(0.0, 1.0, 0.0, math.radians(90), 0.30, 0.2), 1.0)
        self.assertEqual(e.steer(0.0, 1.0, 0.0, math.radians(90), 0.30, 0.3), 1.0)
        self.assertEqual(e.steer(0.0, 1.0, 0.0, math.radians(90), 0.30, 0.4), -1.0)

    def test_exit_steer_refresh_reanchors_to_live_judge(self):
        # Odom drifts during the spin: every refresh_s the world target
        # re-anchors from the live full-circle judge.
        e = ExitSteer()
        e.latch(0.0, 0.0, 0.0, math.radians(90), 0.30,
                min_range=0.08, elapsed_s=0.0)
        e.steer(0.0, 0.0, 0.0, math.radians(90), 0.30, 0.1)
        e.steer(0.0, 0.0, math.radians(5), math.radians(75), 0.30, 2.1)
        self.assertAlmostEqual(e.target, math.radians(80), places=6)

    def test_exit_steer_refresh_never_reanchors_to_a_benched_bearing(self):
        e = ExitSteer()
        e.bench(0.0, 0.0, math.radians(120))
        e.latch(0.0, 0.0, 0.0, math.radians(90), 0.30,
                min_range=0.08, elapsed_s=0.0)
        # Live judge says +115° — within 25° of the benched +120° → refused,
        # the +90° target survives.
        e.steer(0.0, 0.0, 0.0, math.radians(115), 0.30, 2.1)
        self.assertAlmostEqual(e.target, math.radians(90), places=6)

    def test_exit_steer_drops_target_at_timeout(self):
        # A target that never confirms is dropped at timeout_s so the legacy
        # timeout flips take over instead of an infinite steered spin.
        e = ExitSteer()
        e.latch(0.0, 0.0, 0.0, math.radians(90), 0.30,
                min_range=0.08, elapsed_s=0.0)
        self.assertEqual(e.steer(0.0, 0.0, 0.0, math.radians(90), 0.30, 14.9), 1.0)
        self.assertIsNone(e.steer(0.0, 0.0, 0.0, math.radians(90), 0.30, 15.1))
        self.assertIsNone(e.target)

    def test_bench_blocks_repeat_directions_near_the_anchor(self):
        e = ExitSteer()
        e.bench(1.0, 2.0, 0.5)
        self.assertTrue(e.blocked(1.0, 2.0, 0.5))
        # 11° away, same pocket → still blocked (±25° tol).
        self.assertTrue(e.blocked(1.0, 2.0, 0.7))
        # 40° away → a different direction, allowed.
        self.assertFalse(e.blocked(1.0, 2.0, 0.5 + math.radians(40)))

    def test_bench_forgives_after_leaving_the_pocket(self):
        # Same bearing 0.5 m from the anchor → left the stuck spot → fresh
        # judgment.
        e = ExitSteer()
        e.bench(1.0, 2.0, 0.5)
        self.assertFalse(e.blocked(1.5, 2.0, 0.5))

    def test_bench_fifo_keep(self):
        # keep=2: the oldest failed direction is forgotten, the newest block.
        e = ExitSteer(bench_keep=2)
        e.bench(0.0, 0.0, 0.0)
        e.bench(0.0, 0.0, math.radians(120))
        e.bench(0.0, 0.0, 3.5)
        self.assertFalse(e.blocked(0.0, 0.0, 0.0))
        self.assertTrue(e.blocked(0.0, 0.0, 3.5))

    def test_bench_reset_on_success(self):
        # A real drive between stucks is a new pocket → forget failures.
        e = ExitSteer()
        e.bench(0.0, 0.0, 0.0)
        e.reset_bench()
        self.assertFalse(e.blocked(0.0, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
