#!/usr/bin/env python3
"""Verify the saved maze map against the SDF ground truth.

map_saver output is only as right as the SLAM that painted it; this makes
"the map is correct" a measurement instead of a feeling. Rasterizes the SDF
wall boxes at the map's own resolution/origin, then measures:

  wall recall      SDF wall cells with an occupied map pixel within tol
  corridor purity  known corridor cells that are free (not painted wall)
  phantom walls    map occupied pixels farther than tol from any SDF wall
  interior unknown fraction of walkable interior left unknown

Writes map/gz_maze_overlay.png (map + SDF wall outlines in red) and prints
the four numbers. Exit 1 if any gate fails.
"""
import argparse
import os
import re
import sys

import cv2
import numpy as np

MAZE = (0.0, 3.9)   # SDF walls span [0, 3.9] x [0, 3.9]
INTERIOR = (0.3, 3.6)  # walkable interior, away from the outer shell


def parse_sdf_walls(path):
    """Wall rectangles (x0, x1, y0, y1) from the maze collision boxes.

    All boxes carry yaw 0 in the generated SDF, so pose +- half size is
    the axis-aligned rectangle.
    """
    sdf = open(path).read()
    pat = (r'<collision name="w\d+">\s*<pose>([^<]+)</pose>\s*'
           r'<geometry><box><size>([^<]+)</size>')
    walls = []
    for m in re.finditer(pat, sdf):
        p = [float(v) for v in m.group(1).split()]
        s = [float(v) for v in m.group(2).split()]
        walls.append((p[0] - s[0] / 2, p[0] + s[0] / 2,
                      p[1] - s[1] / 2, p[1] + s[1] / 2))
    return walls


def load_map(yaml_path):
    """(pgm uint8 h x w, res, ox, oy) from map_saver's yaml + pgm pair."""
    txt = open(yaml_path).read()
    image = re.search(r'image:\s*(\S+)', txt).group(1)
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                             image)
    res = float(re.search(r'resolution:\s*([\d.]+)', txt).group(1))
    ox, oy = [float(v) for v in
              re.search(r'origin:\s*\[([^\]]+)\]', txt).group(1).split(',')[:2]]
    img = load_pgm(image)
    if img is None:
        raise SystemExit(f'cannot load pgm {image}')
    return img, res, ox, oy


def load_pgm(path):
    data = open(path, 'rb').read()
    if not data.startswith(b'P5'):
        return None
    m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+(\d+)\s', data)
    if not m:
        return None
    w, h, maxv = int(m.group(1)), int(m.group(2)), int(m.group(3))
    start = m.end()
    return np.frombuffer(data[start:start + w * h],
                         np.uint8).reshape(h, w).copy()


def raster_walls(walls, raster_fn, h, w):
    """bool mask from world rectangles via world->pixel mapper."""
    mask = np.zeros((h, w), bool)
    for x0, x1, y0, y1 in walls:
        (px0, py0) = raster_fn(x0, y1)
        (px1, py1) = raster_fn(x1, y0)
        px0, px1 = sorted((int(px0), int(px1) + 1))
        py0, py1 = sorted((int(py0), int(py1) + 1))
        mask[py0:py1, px0:px1] = True
    return mask


def measure(img, res, ox, oy, walls, tol_m=0.10):
    """Metrics of one map raster vs SDF wall rects: dict of numbers+masks.

    Gates mirror the PRD: walls present, corridors free, few phantom walls,
    interior covered. Dashboard /result.json reuses this on the saved map.
    """
    h, w = img.shape

    def w2p(x, y):
        return (x - ox) / res, h - 1 - (y - oy) / res

    wall_px = raster_walls(walls, w2p, h, w)
    known = img != 205
    occ = img < 100
    free = img > 230

    def box(x0, y0, x1, y1):
        px0, py0 = w2p(x0, y1)
        px1, py1 = w2p(x1, y0)
        px0, px1 = sorted((int(px0), int(px1) + 1))
        py0, py1 = sorted((int(py0), int(py1) + 1))
        m = np.zeros((h, w), bool)
        m[py0:py1, px0:px1] = True
        return m

    maze_m = box(*MAZE, *MAZE)
    walkable_m = box(*INTERIOR, *INTERIOR) & ~wall_px
    corridor_m = maze_m & ~wall_px

    tol = max(1, int(round(tol_m / res)))
    occ_dil = cv2.dilate(occ.astype(np.uint8),
                         np.ones((2 * tol + 1,) * 2, np.uint8)) > 0
    wall_cells = wall_px & maze_m
    recall = (occ_dil[wall_cells].mean() if wall_cells.any() else 0.0)

    kcorr = known & corridor_m
    purity = (free & corridor_m)[kcorr].mean() if kcorr.any() else 0.0

    dist_px = cv2.distanceTransform((~wall_px).astype(np.uint8),
                                    cv2.DIST_L2, 3)
    ph = int((occ & (dist_px > tol)).sum())
    ph_frac = ph / max(1, int(occ.sum()))

    walk = int(walkable_m.sum())
    unk_frac = (walk - int((known & walkable_m).sum())) / max(1, walk)

    return {
        'w': w, 'h': h, 'res': res, 'ox': ox, 'oy': oy,
        'walls': len(walls), 'occ_px': int(occ.sum()),
        'known_px': int((known & maze_m).sum()),
        'wall_recall': round(float(recall), 3),
        'corridor_purity': round(float(purity), 3),
        'phantom_frac': round(float(ph_frac), 3),
        'interior_unknown': round(float(unk_frac), 3),
        'img': img, 'wall_px': wall_px, 'maze_m': maze_m,
        'gates': {
            'wall_recall>=0.85': bool(recall >= 0.85),
            'corridor_purity>=0.85': bool(purity >= 0.85),
            'phantom<0.10': bool(ph_frac < 0.10),
            'interior_unknown<0.10': bool(unk_frac < 0.10),
        },
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='map/gz_maze.yaml')
    ap.add_argument('--sdf', default='tools/gz/pinky_maze.sdf')
    ap.add_argument('--tol', type=float, default=0.10,
                    help='wall match tolerance, m')
    ap.add_argument('--out', default='map/gz_maze_overlay.png')
    ap.add_argument('--dump', default=None,
                    help='optional npz dump of masks for debugging')
    a = ap.parse_args()

    img, res, ox, oy = load_map(a.map)
    h, w = img.shape
    walls = parse_sdf_walls(a.sdf)
    print(f'map {w}x{h} res={res} origin=({ox:.3f},{oy:.3f}) '
          f'walls={len(walls)}')

    def w2p(x, y):
        return (x - ox) / res, h - 1 - (y - oy) / res

    wall_px = raster_walls(walls, w2p, h, w)
    known = img != 205
    occ = img < 100
    free = img > 230

    # Maze-area masks on the map grid.
    def box(x0, y0, x1, y1):
        (px0, py0) = w2p(x0, y1)
        (px1, py1) = w2p(x1, y0)
        px0, px1 = sorted((int(px0), int(px1) + 1))
        py0, py1 = sorted((int(py0), int(py1) + 1))
        m = np.zeros((h, w), bool)
        m[py0:py1, px0:px1] = True
        return m

    maze_m = box(*MAZE, *MAZE)
    inter_m = box(*INTERIOR, *INTERIOR)
    corridor_m = maze_m & ~wall_px
    walkable_m = inter_m & ~wall_px

    k = known & maze_m  # observed area, for context in the printout
    # 1) wall recall: SDF wall cells with occupied pixel within tol px.
    tol = max(1, int(round(a.tol / res)))
    kerdil = cv2.dilate(occ.astype(np.uint8), np.ones((2 * tol + 1,) * 2,
                                                      np.uint8))
    wall_cells = wall_px & maze_m
    recall = (kerdil > 0)[wall_cells].mean()
    print(f'wall_recall {recall:.3f} (n={wall_cells.sum()}, '
          f'tol={a.tol}m)')

    # 2) corridor purity: known corridor cells that are free.
    kcorr = known & corridor_m
    purity = (free & corridor_m)[kcorr].mean()
    print(f'corridor_purity {purity:.3f} (n={kcorr.sum()})')

    # 3) phantom walls: occupied far from any SDF wall.
    dist_px = cv2.distanceTransform((~wall_px).astype(np.uint8),
                                    cv2.DIST_L2, 3)
    phantoms = occ & (dist_px > tol)
    ph = phantoms.sum()
    ph_frac = ph / max(1, occ.sum())
    print(f'phantom {ph} px of {occ.sum()} occ ({ph_frac:.3f})')

    # 4) interior unknown fraction.
    walk = walkable_m.sum()
    unk_frac = (walk - (known & walkable_m).sum()) / max(1, walk)
    print(f'interior_unknown {unk_frac:.3f} (walkable={walk})')

    gates = {
        'wall_recall>=0.85': recall >= 0.85,
        'corridor_purity>=0.85': purity >= 0.85,
        'phantom<0.10': ph_frac < 0.10,
        'interior_unknown<0.10': unk_frac < 0.10
    }
    print('gates:', {k: v for k, v in gates.items()})
    ok = all(gates.values())
    if not ok:
        print('FAIL')
    # Machine-readable twin of the printout: the web dashboard's
    # /result.json serves this file verbatim next to the live state.
    import json
    with open(os.path.splitext(a.out)[0] + '_metrics.json', 'w') as f:
        json.dump({
            'wall_recall': round(recall, 3), 'n_wall': int(wall_cells.sum()),
            'corridor_purity': round(purity, 3), 'n_corridor': int(kcorr.sum()),
            'phantom_frac': round(ph_frac, 3), 'occ_px': int(occ.sum()),
            'interior_unknown': round(unk_frac, 3), 'n_walkable': int(walk),
            'gates': gates, 'ok': ok,
        }, f, indent=2)

    # Evidence: map | SDF truth | overlay (SDF wall outline on the map).
    truth = np.full((h, w, 3), 255, np.uint8)
    truth[wall_px] = (0, 0, 0)
    truth[~maze_m] = (205, 205, 205)
    over = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    edge = cv2.dilate(wall_px.astype(np.uint8), np.ones((3, 3), np.uint8)) \
        != wall_px
    over[edge & maze_m] = (0, 0, 255)
    over[wall_px] = np.minimum(over[wall_px], (0, 0, 255))
    panel = np.concatenate([cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), truth,
                            over], axis=1)
    cv2.imwrite(a.out, panel)
    print('overlay ->', a.out)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
