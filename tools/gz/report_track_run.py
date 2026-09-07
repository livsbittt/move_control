"""Plot the exact rotated track walls, saved SLAM raster and observed trajectory."""
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np


def main():
    folder = Path(sys.argv[1])
    identity = json.loads((folder/'track_identity.json').read_text())
    result = json.loads((folder/'track_result.json').read_text())
    rows = json.loads((folder/'track_samples.json').read_text())
    fig, (ax, motion) = plt.subplots(2, 1, figsize=(12, 8), gridspec_kw={'height_ratios': [3, 1]})
    ax.set_facecolor('#aaaaaa')
    if (folder/'track_map.npz').exists():
        saved = np.load(folder/'track_map.npz')
        arr, (ox, oy), res = saved['data'], saved['origin'], float(saved['resolution'])
        ax.imshow(np.where(arr < 0, .5, 1-arr/100.), cmap='gray', vmin=0, vmax=1,
                  origin='lower', extent=(ox, ox+arr.shape[1]*res, oy, oy+arr.shape[0]*res))
    for pose, size in identity['walls']:
        c, s = math.cos(pose[5]), math.sin(pose[5])
        corners = np.array([[-1,-1], [1,-1], [1,1], [-1,1]])*np.array(size[:2])/2
        corners = corners @ np.array([[c,s],[-s,c]])+np.array(pose[:2])
        ax.add_patch(Polygon(corners, facecolor='#db594f', edgecolor='#9a251e', alpha=.65))
    points = np.array([r['pose'] for r in rows if r['pose'] is not None])
    if len(points):
        ax.plot(points[:,0], points[:,1], color='#1565c0', linewidth=2, label='Observed trajectory')
        ax.scatter(*points[0], color='#00a060', s=45, label='Start')
    ax.set(xlim=(-1.45,1.45), ylim=(-.73,.73), aspect='equal', xlabel='World x (m)', ylabel='World y (m)')
    ax.legend(loc='upper left')
    ax.set_title('map_260905.world — original scale and all 16 collision walls\n'
                 f"Calibration: {result['calibration_phase']} | Partial SLAM map; full mapping NOT established")
    motion.plot([r['sim_s'] for r in rows], [r['safe'][0] for r in rows], label='Final linear m/s')
    motion.plot([r['sim_s'] for r in rows], [r['safe'][1] for r in rows], label='Final angular rad/s')
    motion.set(xlabel='Simulation time (s)', ylabel='Safety output')
    motion.legend()
    motion.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(folder/'track_result.png', dpi=160)


if __name__ == '__main__':
    main()
