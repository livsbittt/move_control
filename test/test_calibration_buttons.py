"""The operating row offers three choices; removed modes must not linger in the markup."""
import re
import unittest
from pathlib import Path


class CalibrationButtonTests(unittest.TestCase):
    def markup(self):
        return Path(__file__).parents[1].joinpath('web/dashboard.html').read_text(encoding='utf-8')

    def commands(self):
        return re.findall(r'data-calibration="([^"]+)"', self.markup())

    def test_only_the_three_mode_choices_and_the_motion_step_remain(self):
        self.assertEqual(sorted(self.commands()),
                         ['abort', 'partial_calibration', 'retry', 'validate_motion'])

    def test_removed_modes_are_gone_from_the_markup(self):
        for command in ('sensing_only', 'use_existing_settings', 'use_limited_sensors'):
            self.assertNotIn('data-calibration="' + command + '"', self.markup(), command)

    def test_scope_select_offers_full_and_skipped_motion(self):
        markup = self.markup()
        self.assertIn('id="calibrationscope"', markup)
        self.assertIn('value="full"', markup)
        self.assertIn('value="skip_motion"', markup)

    def test_partial_button_and_its_reason_line_exist(self):
        markup = self.markup()
        self.assertIn('id="calibrationpartialaction"', markup)
        self.assertIn('id="calibrationpartialreason"', markup)


if __name__ == '__main__':
    unittest.main()
