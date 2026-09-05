"""Subject: camera look-ahead classify. Floor / void / obstacle."""
import numpy as np


def classify_frame(
    bgr,
    floor_hsv=None,
    void_v_ratio=0.50,
    hue_shift=35.0,
    floor_h_tol=28.0,
    floor_v_tol=55.0,
    near_y0=0.68,
    near_y1=0.95,
    mid_y0=0.22,
    mid_y1=0.62,
    void_frac=0.12,
    obst_frac=0.35,
):
    """Return dict of flags and column scores. bgr is HxWx3 uint8 BGR."""
    import cv2

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    h_ch, s_ch, v_ch = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    h, w = v_ch.shape

    y_lo0, y_lo1 = int(0.70 * h), int(0.92 * h)
    x_lo0, x_lo1 = int(0.15 * w), int(0.85 * w)
    roi = hsv[y_lo0:y_lo1, x_lo0:x_lo1]
    if roi.size == 0:
        return _empty_result()
    vv = roi[:, :, 2]
    p20, p80 = np.percentile(vv, [20, 80])
    sample = roi[(vv >= p20) & (vv <= p80)]
    if sample.size == 0:
        sample = roi.reshape(-1, 3)
    inst = np.median(sample, axis=0)
    if floor_hsv is None:
        fh, fs, fv = (float(inst[0]), float(inst[1]), float(inst[2]))
        new_floor = (fh, fs, fv)
    else:
        a = 0.12
        fh = (1 - a) * floor_hsv[0] + a * float(inst[0])
        fs = (1 - a) * floor_hsv[1] + a * float(inst[1])
        fv = (1 - a) * floor_hsv[2] + a * float(inst[2])
        new_floor = (fh, fs, fv)

    d_h = np.minimum(np.abs(h_ch - fh), 180.0 - np.abs(h_ch - fh))
    floor = (d_h < floor_h_tol) & (np.abs(v_ch - fv) < floor_v_tol) & (v_ch > fv * 0.55)
    void = (
        (v_ch < fv * void_v_ratio)
        | ((d_h > hue_shift) & (v_ch < fv * 0.90) & (s_ch > 25.0))
    ) & (~floor)
    obst = (~floor) & (~void)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    void = cv2.morphologyEx(void.astype(np.uint8), cv2.MORPH_OPEN, k).astype(bool)
    obst = cv2.morphologyEx(obst.astype(np.uint8), cv2.MORPH_OPEN, k).astype(bool)

    ny0, ny1 = int(near_y0 * h), int(near_y1 * h)
    cols = []
    for x0, x1 in ((0, w // 3), (w // 3, 2 * w // 3), (2 * w // 3, w)):
        sl = (slice(ny0, ny1), slice(x0, x1))
        cols.append(
            {
                'void': float(void[sl].mean()) if void[sl].size else 0.0,
                'obst': float(obst[sl].mean()) if obst[sl].size else 0.0,
                'floor': float(floor[sl].mean()) if floor[sl].size else 0.0,
            }
        )
    my0, my1 = int(mid_y0 * h), int(mid_y1 * h)
    mx0, mx1 = int(0.20 * w), int(0.80 * w)
    mid_obst = float(obst[my0:my1, mx0:mx1].mean()) if obst[my0:my1, mx0:mx1].size else 0.0

    mid_cols = []
    for x0, x1 in ((0, w // 3), (w // 3, 2 * w // 3), (2 * w // 3, w)):
        sl = (slice(my0, my1), slice(x0, x1))
        mid_cols.append(
            {
                'void': float(void[sl].mean()) if void[sl].size else 0.0,
                'obst': float(obst[sl].mean()) if obst[sl].size else 0.0,
                'floor': float(floor[sl].mean()) if floor[sl].size else 0.0,
            }
        )

    on_floor = any(c['floor'] >= 0.20 for c in cols)
    cliff = on_floor and any(c['void'] >= void_frac for c in cols)
    # Narrow maze: side walls fill the mid band. Blocked = CENTER column only
    # (corner / dead-end ahead), not the walls hugging left/right.
    blocked = mid_cols[1]['obst'] >= obst_frac
    lo, lc, lr = mid_cols[0]['obst'], mid_cols[1]['obst'], mid_cols[2]['obst']
    if lr > lo + 0.08:
        side = 1.0
    elif lo > lr + 0.08:
        side = -1.0
    else:
        scores = [c['void'] + 0.5 * c['obst'] for c in cols]
        imax = int(np.argmax(scores))
        if max(scores) - min(scores) < 0.08:
            side = 0.0
        else:
            side = float((-1, 0, 1)[imax])
    return {
        'cliff': cliff,
        'blocked': blocked,
        'side': side,
        'cols': cols,
        'mid_cols': mid_cols,
        'mid_obst': mid_obst,
        'floor_hsv': new_floor,
    }

def _empty_result():
    return {
        'cliff': False,
        'blocked': False,
        'side': 0.0,
        'cols': [{'void': 0.0, 'obst': 0.0, 'floor': 0.0}] * 3,
        'mid_obst': 0.0,
        'mid_cols': [{'void': 0.0, 'obst': 0.0, 'floor': 0.0}] * 3,
        'floor_hsv': None,
    }
