#!/usr/bin/env python3
"""Mode label precedence + the shared nose-contact predicate."""
import math
import unittest

from move_control.control.modes import (
    MODES,
    NON_FORWARD_STATES,
    WANDER_TO_MODE,
    nose_on_wall,
    pick_mode,
)


class TaxonomyTest(unittest.TestCase):
    def test_labels_unique(self):
        names = [m.name for m in MODES]
        self.assertEqual(len(names), len(set(names)))

    def test_wander_to_mode_shape(self):
        self.assertEqual(WANDER_TO_MODE['forward'], 'FWD')
        self.assertNotIn('forward', NON_FORWARD_STATES)
        self.assertEqual(len(NON_FORWARD_STATES), len(WANDER_TO_MODE) - 1)


class PickModeTest(unittest.TestCase):
    def test_hazard_precedence_estop_pick_cliff_tilt(self):
        self.assertEqual(
            pick_mode(estop=True, pickup=True, cliff=True, tilt=True), 'ESTOP'
        )
        self.assertEqual(pick_mode(pickup=True, cliff=True, tilt=True), 'PICK')
        self.assertEqual(pick_mode(cliff=True, tilt=True), 'CLIFF')
        self.assertEqual(pick_mode(tilt=True), 'TILT')

    def test_estop_beats_the_idle_stop_label(self):
        # A real e-stop must never masquerade as the idle STOP label.
        self.assertEqual(pick_mode(estop=True, wander_state='stop'), 'ESTOP')

    def test_non_forward_states_own_the_label(self):
        for state, label in WANDER_TO_MODE.items():
            if state == 'forward':
                continue  # forward falls through to the contact bands
            self.assertEqual(pick_mode(wander_state=state), label)

    def test_hazard_shadows_action(self):
        # Hazard beats the running action: backing up into a cliff is CLIFF.
        self.assertEqual(pick_mode(cliff=True, wander_state='backup'), 'CLIFF')
        self.assertEqual(pick_mode(tilt=True, wander_state='turn'), 'TILT')

    def test_forward_clear_is_fwd(self):
        self.assertEqual(
            pick_mode(wander_state='forward', front=5.0, us=0.97), 'FWD'
        )

    def test_forward_warn_band_edges(self):
        p = lambda **kw: pick_mode(wander_state='forward', on_wall=False, **kw)
        # wall_front exclusive, warn_front inclusive.
        self.assertEqual(p(front=0.08), 'FWD')     # at lo: not WARN
        self.assertEqual(p(front=0.081), 'WARN')
        self.assertEqual(p(front=0.11), 'WARN')    # at hi: WARN
        self.assertEqual(p(front=0.12), 'FWD')

    def test_lidar_front_cap(self):
        # Beyond the 8 m lidar trust horizon the nose is "open".
        self.assertEqual(
            pick_mode(wander_state='forward', on_wall=False, front=8.5), 'FWD'
        )

    def test_us_echo_counts_toward_warn(self):
        self.assertEqual(
            pick_mode(wander_state='forward', on_wall=False, front=math.inf, us=0.10),
            'WARN',
        )

    def test_us_no_echo_ignored(self):
        # ~0.97 m is the US no-echo value, never a distance.
        self.assertEqual(
            pick_mode(wander_state='forward', on_wall=False, front=math.inf, us=0.97),
            'FWD',
        )

    def test_unknown_state_falls_to_fwd(self):
        self.assertEqual(pick_mode(wander_state='nonsense'), 'FWD')


class NoseOnWallTest(unittest.TestCase):
    def test_lidar_contact(self):
        self.assertTrue(nose_on_wall(0.05, math.inf, 0.08))
        self.assertFalse(nose_on_wall(0.09, math.inf, 0.08))

    def test_us_echo_contact_and_no_echo(self):
        self.assertTrue(nose_on_wall(math.inf, 0.05, 0.08))
        # 0.97 = US no-echo, never contact.
        self.assertFalse(nose_on_wall(math.inf, 0.97, 0.08))
        # == wall_front is contact.
        self.assertTrue(nose_on_wall(math.inf, 0.08, 0.08))

    def test_us_echo_respects_0_80_cap_even_with_wide_wall_front(self):
        # A wide wall_front must not let a long US echo read as a wall.
        self.assertFalse(nose_on_wall(math.inf, 0.85, 0.40))

    def test_zero_is_never_contact(self):
        self.assertFalse(nose_on_wall(0.0, math.inf, 0.08))
        self.assertFalse(nose_on_wall(math.inf, 0.0, 0.08))


if __name__ == '__main__':
    unittest.main()
