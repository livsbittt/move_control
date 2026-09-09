"""Subject: camera look-ahead classify. Floor / void / obstacle."""
import numpy as np
from .camera_evidence import REGION_CAP
from .camera_regions import foreground_regions


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
    obst_frac=0.45,
    allow_floor_update=True,
    region_min_area_fraction=.0005,
    ground=None,
):
    """Return dict of flags and column scores. bgr is HxWx3 uint8 BGR."""
    import cv2

    if (not isinstance(bgr, np.ndarray) or bgr.dtype != np.uint8 or
            bgr.ndim != 3 or bgr.shape[2] != 3 or min(bgr.shape[:2]) < 8):
        return _empty_result('invalid_image')
    if floor_hsv is not None and (len(floor_hsv) != 3 or not np.isfinite(floor_hsv).all()):
        return _empty_result('invalid_floor_reference')
    if not np.isfinite(region_min_area_fraction) or not 0 < region_min_area_fraction <= 1:
        return _empty_result('invalid_region_threshold')

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    h_ch, s_ch, v_ch = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    h, w = v_ch.shape
    quality = dict(valid=True, reason='usable', reference='previous' if floor_hsv is not None else 'bootstrap')
    if np.mean(v_ch < 8) > .95 or np.mean(v_ch > 247) > .95:
        return _empty_result('underexposed' if np.mean(v_ch < 8) > .95 else 'overexposed')

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
        new_floor = (fh, fs, fv) if allow_floor_update else None
    else:
        fh, fs, fv = map(float, floor_hsv)
        ih, iss, iv = map(float, inst)
        hue_delta = (ih - fh + 90.) % 180. - 90.
        # A wall or blue tape filling the ROI must not become the floor.
        # Gray pixels have unstable hue, so compare their saturation/value.
        compatible = (abs(iss-fs) < 40. and abs(iv-fv) < floor_v_tol and
                      ((iss <= 40. and fs <= 40.) or abs(hue_delta) < floor_h_tol))
        if allow_floor_update and compatible:
            a = 0.12
            fh = (fh + a*hue_delta) % 180.
            fs, fv = (1-a)*fs+a*iss, (1-a)*fv+a*iv
        new_floor = (fh, fs, fv)

    d_h = np.minimum(np.abs(h_ch - fh), 180.0 - np.abs(h_ch - fh))
    same_color = (((s_ch <= 40.) & (fs <= 40.)) | (d_h < floor_h_tol)) & (np.abs(s_ch-fs) < 50.)
    floor = same_color & (np.abs(v_ch - fv) < floor_v_tol) & (v_ch > fv * 0.55)
    # Blue maze tape can differ in hue while retaining 80% of floor
    # brightness. Hue alone is not missing-floor evidence: keep those
    # pixels as obstacle cues, and require the existing darkness ratio
    # for a visual drop. Floor IR remains an independent safety input.
    # hue_shift stays in the call signature for existing callers.
    void = (v_ch < fv * void_v_ratio) & (~floor)
    obst = (~floor) & (~void)
    regions = foreground_regions(obst | void, void, near_y0, near_y1,
                                region_min_area_fraction, ground)

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

    # The camera does not assert a drop, and the flag stays False by design.
    #
    # Monocularly, a dark wall and a dark hole at the same ground line produce
    # the same image, so any appearance test picks one by fiat. Both choices were
    # measured on the recorded frames and both are wrong:
    #   darkness alone       -> cliff=True on the dark red wall above a bright
    #                           wood floor in camera-before-wander.jpg
    #   "ignore dark regions touching the image top, they are walls"
    #                        -> a desk edge whose beyond-scene is dark reaches
    #                           row 0 too, so a drop filling 49% of all three
    #                           near columns reported nothing. On that same
    #                           reference frame 100% of void pixels form one
    #                           row-0-touching component, so the test could never
    #                           fire in the environment the robot works in.
    # A false positive costs a needless turn; a false negative costs the desk.
    # Neither is acceptable when the floor IR already owns this decision, so the
    # darkness stays published as evidence -- cols[*]['void'] and regions with
    # kind='dark_region' -- and no verdict is drawn from it here.
    cliff = False
    # A visible wall in the upper/mid view does not imply nearby contact:
    # the measured corridor frame had a clear lower half and a wall 0.65 m
    # ahead. Require obstacle in the near centre, also catching low objects
    # below the mid band. Lidar owns metric stopping distance; cliff logic
    # above and mid-band directional scores remain independent.
    blocked = cols[1]['obst'] >= obst_frac or any(r['near_path'] for r in regions)
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
        'quality': quality,
        'regions': regions[:REGION_CAP],
        'region_count': len(regions),
        'regions_truncated': len(regions) > REGION_CAP,
    }


def _empty_result(reason='empty_roi'):
    return {
        'cliff': False,
        'blocked': False,
        'side': 0.0,
        'cols': [{'void': 0.0, 'obst': 0.0, 'floor': 0.0}] * 3,
        'mid_obst': 0.0,
        'mid_cols': [{'void': 0.0, 'obst': 0.0, 'floor': 0.0}] * 3,
        'floor_hsv': None,
        'quality': {'valid': False, 'reason': reason},
        'regions': [], 'region_count': 0, 'regions_truncated': False,
    }
