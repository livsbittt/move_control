"""Offline diagnostics only: truth is used to score search quality, never seed ROS."""
from pathlib import Path
import sys
import math
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from rosy_control.sensing.localization import MapAgreement
d = np.load(sys.argv[1])
m = MapAgreement(d['grid'], float(d['resolution']), d['origin'])
ranges, angles, truth = d['ranges'], d['angles'], d['truth']
offsets = np.array(np.meshgrid(np.arange(-.05, .051, .01), np.arange(-.05, .051, .01),
    np.radians(np.arange(-5., 5.1, 1.)), indexing='ij')).reshape(3, -1).T
poses = truth + offsets
quality = m._qualities(poses, ranges, angles)
for idx in np.argsort(quality)[-3:]:
    print('truth-near diagnostic', poses[idx], quality[idx], m.score(poses[idx], ranges, angles))
print('global', m.global_match(ranges, angles, .105))
