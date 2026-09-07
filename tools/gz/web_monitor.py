#!/usr/bin/env python3
"""Run monitor over the web backend (HTTP polling) - no DDS join.

The rclpy monitor is a late DDS joiner and this box's graph can refuse
late joiners under load; the web_node joined early and keeps receiving.
Polls /state.json at 1 Hz: done / stall / timeout + periodic map PNGs.

  exit 0  'coverage done' held --done-hold s and known cells sane
  exit 2  known cells unchanged for --stall s while not done
  exit 1  --timeout s elapsed without done
"""
import argparse
import json
import os
import sys
import time
import urllib.request

import numpy as np
import cv2

SAVE_EVERY = 30.0
HEARTBEAT = 10.0
MIN_KNOWN = 500


def log(t0, s):
    print(f'[WMON {time.monotonic() - t0:6.0f}s] {s}', flush=True)


def fetch(url, timeout=2.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.load(r)
    except Exception:
        pass
    return None


def fetch_png(url, timeout=2.0):
    try:
        buf = urllib.request.urlopen(url, timeout=timeout).read()
        import cv2
        return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--backend', default='http://localhost:28162')
    ap.add_argument('--done-hold', type=float, default=60.0)
    ap.add_argument('--stall', type=float, default=600.0)
    ap.add_argument('--timeout', type=float, default=4200.0)
    ap.add_argument('--out', default='/tmp/gztest/monitor')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    t0 = time.monotonic()
    known = -1
    known_t = t0
    done_t = None
    last_save = 0.0
    last_beat = 0.0
    while True:
        t = time.monotonic()
        st = fetch(a.backend + '/state.json')
        if st is None:
            if t - t0 > a.timeout:
                log(t0, 'TIMEOUT without a usable backend')
                sys.exit(1)
            time.sleep(1.0)
            continue
        path_m = st.get('path_m') or 0.0
        gstate = st.get('gstate') or ''
        # The web node ships map CELLS only as /map.png (gen counter); the
        # known-cell count comes from decoding that PNG (occ=0, free=255,
        # unknown=205 gray).
        if t - last_save >= SAVE_EVERY:
            png = fetch_png(a.backend + '/map.png')
            if png is not None:
                k = int(((png != 205).any(axis=2)).sum())
                if k != known:
                    known = k
                    known_t = t
                cv2.imwrite(os.path.join(a.out, 'map_live.png'), png)
            last_save = t
        if t - last_beat >= HEARTBEAT:
            last_beat = t
            log(t0, f"state={gstate[:44]!r} path={path_m:.1f}m "
                    f"known={known} trail={len(st.get('trail') or [])}")
        done = gstate.startswith('coverage done')
        if done and known >= MIN_KNOWN:
            if done_t is None:
                done_t = t
            elif t - done_t >= a.done_hold:
                log(t0, f'DONE: coverage done held, path={path_m:.1f}m '
                        f'known={known}')
                sys.exit(0)
        elif done_t is not None:
            done_t = None
        if not done and known > 0 and t - known_t > a.stall:
            log(t0, f'STALL: known={known} unchanged {a.stall}s, '
                    f"state={gstate[:50]!r}")
            sys.exit(2)
        if t - t0 > a.timeout:
            log(t0, f'TIMEOUT: state={gstate[:50]!r} path={path_m:.1f}m '
                    f'known={known}')
            sys.exit(1)
        time.sleep(1.0)


if __name__ == '__main__':
    main()
