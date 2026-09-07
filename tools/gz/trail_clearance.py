#!/usr/bin/env python3
"""Trail-to-wall clearance from the monitor's map PNG.

The monitor renders trail=red(0,0,255) BGR, occupied=black, free=white,
unknown=gray. This computes the L2 distance transform from occupied cells
and reports clearance stats for the trail: the wall-hug verdict metric.

  python3 tools/gz/trail_clearance.py /tmp/gztest14/monitor/map_live.png

Pixels are 5 cm cells; the sim robot is 18 cm wide (9.6 cm circumradius
with wheels, ROBOT_RADIUS below), so the corridor centerline is ~6 cm from
a wall face (30 cm corridors). Chassis clearance = center-line clearance
- robot radius.
"""
import sys

import cv2
import numpy as np

# Sim robot circumradius (SDF: 0.14x0.10x0.06 chassis + wheels at +-0.082,
# wheel r 0.028 => half-width 0.096 with wheels): distance transform works
# on mapped wall FACES, which the chassis sides reach.
ROBOT_RADIUS = 0.096
HUG_THRESH = 0.05   # trail within 5 cm (mapped-face distance) of a wall


def stats(path, hug_thresh=HUG_THRESH):
    img = cv2.imread(path)
    if img is None:
        raise SystemExit(f"cannot read {path}")
    red = (img == [0, 0, 255]).all(axis=2)
    occ = (img == [0, 0, 0]).all(axis=2)
    if not occ.any():
        raise SystemExit("no occupied cells in map PNG")
    # L2 distance (cells) from every non-occupied pixel to the wall set.
    dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 3)
    d = dist[red]
    if d.size == 0:
        raise SystemExit("no trail pixels in map PNG")
    d_m = d * 0.05
    return {
        'trail_px': int(d.size),
        'min': float(d_m.min()),
        'p10': float(np.percentile(d_m, 10)),
        'median': float(np.percentile(d_m, 50)),
        'p90': float(np.percentile(d_m, 90)),
        'within_hug': float((d_m <= hug_thresh).mean()),
    }


def main():
    path = sys.argv[1]
    s = stats(path)
    print(f"{path}")
    print(f"  trail_px={s['trail_px']}  min={s['min']:.3f}m p10={s['p10']:.3f}m "
          f"median={s['median']:.3f}m p90={s['p90']:.3f}m")
    print(f"  trail within {HUG_THRESH*100:.0f}cm of mapped wall: "
          f"{s['within_hug']*100:.1f}%  "
          f"(chassis clearance = center-line - {ROBOT_RADIUS*100:.1f}cm radius)")


if __name__ == '__main__':
    main()
