"""Camera proximity cues must distinguish visible walls from near obstacles."""
import unittest

import numpy as np

from move_control.sensing.camera import classify_frame


class CameraNearObstacleTest(unittest.TestCase):
    def frame(self):
        # Neutral floor avoids conflating hue changes with void detection.
        return np.full((240, 320, 3), 100, dtype=np.uint8)

    def classify(self, frame):
        return classify_frame(frame, floor_hsv=(0., 0., 100.), obst_frac=.45)

    def test_distant_wall_with_clear_near_floor_is_not_blocked(self):
        frame = self.frame()
        frame[:108] = 240  # Wall occupies upper 45%; lower floor is clear.
        result = self.classify(frame)
        self.assertGreater(result['mid_cols'][1]['obst'], .45)
        self.assertFalse(result['blocked'])
        self.assertFalse(result['cliff'])

    def test_close_wall_extending_into_near_centre_is_blocked(self):
        frame = self.frame()
        frame[:205] = 240
        self.assertTrue(self.classify(frame)['blocked'])

    def test_low_near_obstacle_is_blocked_even_when_mid_view_is_clear(self):
        frame = self.frame()
        frame[175:230, 106:213] = 240
        result = self.classify(frame)
        self.assertLess(result['mid_cols'][1]['obst'], .1)
        self.assertTrue(result['blocked'])

    def test_corridor_side_walls_do_not_block_clear_centre(self):
        frame = self.frame()
        frame[:, :80] = frame[:, 240:] = 240
        self.assertFalse(self.classify(frame)['blocked'])

    def test_near_dark_drop_still_reports_cliff(self):
        frame = self.frame()
        frame[165:230, :100] = 5
        result = self.classify(frame)
        self.assertTrue(result['cliff'])

    def test_colored_side_tape_is_not_a_dark_drop(self):
        frame = self.frame()
        # Blue tape on the measured maze changes hue, but retains 80% of
        # floor brightness. Color alone cannot establish missing floor.
        frame[165:230, :100] = (80, 20, 20)
        result = self.classify(frame)
        self.assertFalse(result['cliff'])
        self.assertFalse(result['blocked'])
        self.assertGreater(result['cols'][0]['obst'], .8)

    def test_colored_near_centre_remains_an_obstacle_cue(self):
        frame = self.frame()
        frame[165:230, 106:213] = (80, 20, 20)
        result = self.classify(frame)
        self.assertFalse(result['cliff'])
        self.assertTrue(result['blocked'])

    def test_dark_colored_drop_still_reports_cliff(self):
        frame = self.frame()
        frame[165:230, :100] = (20, 5, 5)
        self.assertTrue(self.classify(frame)['cliff'])
