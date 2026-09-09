"""A restarted rig must never advertise the previous run's successful map."""
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import yaml


class RigManifestTest(unittest.TestCase):
    def test_restart_archives_success_and_starts_with_pending_evidence(self):
        script = Path('tools/gz/run_calibration_mapping.sh').read_text()
        program = script.split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            (out/'stack.log').write_text('old run')
            (out/'map.pgm').write_bytes(b'old successful map')
            (out/'mapping_metrics.json').write_text('{"map_raster_complete":true}')
            with patch.object(sys, 'argv', ['setup', str(out)]), \
                    patch.dict(os.environ, {'RIG_PLANT': 'velocity'}), \
                    patch('subprocess.check_output', return_value='test-source-revision\n'):
                exec(compile(program, 'rig setup', 'exec'), {})
            pending = json.loads((out/'mapping_metrics.json').read_text())
            manifest = json.loads((out/'run_manifest.json').read_text())
            self.assertFalse(pending['map_raster_complete'])
            self.assertEqual(pending['run_id'], manifest['run_id'])
            self.assertFalse((out/'map.pgm').exists())
            self.assertEqual(len(list((out/'archive').glob('*/map.pgm'))), 1)
            self.assertEqual(manifest['world_sha256'], hashlib.sha256((out/'world.sdf').read_bytes()).hexdigest())
            hashed = {path.replace('\\', '/') for path in manifest['source_sha256']}
            self.assertIn('rosy_control/planning/goals.py', hashed)
            self.assertEqual(manifest['lidar_mount'], 'c1_rear_zero')
            self.assertEqual(manifest['dashboard'], 'safety_fused')
            pose = ET.parse(out/'world.sdf').find(".//model[@name='pinky']//sensor/pose").text.split()
            self.assertAlmostEqual(float(pose[5]), math.pi, places=5)
            self.assertNotIn('lidar_yaw_offset', yaml.safe_load((out/'rig.yaml').read_text())['/**']['ros__parameters'])


if __name__ == '__main__':
    unittest.main()
