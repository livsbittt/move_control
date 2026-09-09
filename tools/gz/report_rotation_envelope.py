"""Plot recorded geometry and sensor estimates, not an inferred physical robot."""
import json
from pathlib import Path
import sys
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
from rosy_control.control.rotation_envelope import validate_envelope


def main():
    folder = Path(sys.argv[1])
    report = json.loads((folder/'calibration.json').read_text())['rotation']['envelope']
    assert validate_envelope(report), 'Recorded sensor estimate is invalid'
    center = np.array(report['center_m'])*100
    footprint = np.array(report['footprint_xy'], dtype=np.float32)*100
    fig, ax = plt.subplots(figsize=(8, 7))
    if len(footprint):
        hull = cv2.convexHull(footprint).reshape(-1,2)
        ax.add_patch(Polygon(hull, facecolor='#cbd5df', edgecolor='#465668', label='Configured collision envelope'))
    body = report['body_radius_m']*100
    radius = report['pivot_radius_m']*100
    ax.add_patch(Circle((0,0), body, fill=False, linestyle=':', color='#465668', label=f'Body circumradius {body:.2f} cm'))
    ax.add_patch(Circle(center, radius, fill=False, color='#007a73', linewidth=2,
                        label=f'Estimated pivot sweep {radius:.2f} cm'))
    ax.add_patch(Circle(center, radius+1, fill=False, linestyle='--', color='#b37700',
                        label='Required obstacle clearance: sweep + 1 cm'))
    ax.scatter(0,0,color='#465668',marker='+',s=90)
    ax.scatter(*center,color='#007a73',s=45)
    ax.annotate('Estimated pivot',center,xytext=(-18,7),arrowprops={'arrowstyle':'->','color':'#007a73'})
    ax.annotate('Forward',xy=(10,0),xytext=(1,0),arrowprops={'arrowstyle':'->'})
    uncertainty = report['center_uncertainty_m']*100
    ax.set(xlim=(-22,20),ylim=(-20,20),aspect='equal',xlabel='Base x (cm)',ylabel='Base y (cm)',
           title=f'Gazebo wheel model: recorded rotation estimate\nPivot ({center[0]:.2f}, {center[1]:.2f}) cm; empirical uncertainty {uncertainty:.2f} cm')
    ax.grid(alpha=.2)
    ax.legend(loc='lower left',fontsize=9)
    fig.text(.5,.02,'LiDAR + odometry + IMU consistency; simulation auxiliary sensors are synthetic.\nEmpirical uncertainty is not a guaranteed physical safety bound.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.06,1,1))
    fig.savefig(folder/'rotation_envelope.png',dpi=160)


if __name__ == '__main__':
    main()
