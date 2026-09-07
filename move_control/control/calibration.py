"""Stationary sensor-quality and bounded motion validation; no ROS imports."""
import math
from statistics import median

SENSORS = ('lidar', 'odom', 'ir', 'imu', 'us', 'camera', 'tf', 'map', 'map_tf')
MIN_SAMPLES = 20
BASELINE_SECONDS = 3.0
FRESH_SECONDS = 1.0
MOTION_SPEED = .008
MOTION_SECONDS = 4.0
MOTION_LIMIT = .04


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class StationaryBaseline:
    def __init__(self):
        self.samples = {name: [] for name in SENSORS}

    def add(self, name, values, now, valid=True):
        values = tuple(float(v) for v in values)
        valid = valid and all(math.isfinite(v) for v in values)
        rows = self.samples[name]
        rows.append((now, values, bool(valid)))
        self.samples[name] = [row for row in rows if now - row[0] <= 5.0][-512:]

    def latest(self, name):
        rows = self.samples[name]
        return rows[-1][1] if rows and rows[-1][2] else None

    def statistics(self, now):
        result = {}
        for name, rows in self.samples.items():
            rows = [row for row in rows if row[2] and 0 <= now - row[0] <= 5.]
            if not rows:
                continue
            columns = list(zip(*(row[1] for row in rows)))
            means = [sum(col) / len(col) for col in columns]
            result[name] = {'samples': len(rows), 'span_s': rows[-1][0] - rows[0][0],
                            'mean': means, 'std': [math.sqrt(sum((v - mean)**2 for v in col) / len(col))
                                                   for col, mean in zip(columns, means)]}
        return result

    def fresh(self, now):
        return all(rows and rows[-1][2] and 0 <= now - rows[-1][0] <= (5. if name == 'map' else FRESH_SECONDS)
                   for name, rows in self.samples.items())

    def report(self, now):
        result = {}
        for name, rows in self.samples.items():
            recent = [row for row in rows if 0 <= now - row[0] <= 5.0]
            status, detail = 'collecting', 'Need 20 valid samples over 3 stationary seconds'
            if recent and (not recent[-1][2]):
                status, detail = 'invalid', 'Latest sample is invalid'
            elif not recent or now - recent[-1][0] > (5. if name == 'map' else FRESH_SECONDS):
                status, detail = 'stale', 'No fresh sensor sample'
            elif (len(recent) >= (1 if name == 'map' else MIN_SAMPLES) and
                  recent[-1][0] - recent[0][0] >= (0. if name == 'map' else BASELINE_SECONDS)):
                if not all(row[2] for row in recent):
                    status, detail = 'invalid', 'Invalid samples remain in baseline window'
                else:
                    values = [row[1] for row in recent]
                    status, detail = self._quality(name, values)
            result[name] = {'ok': status == 'ok', 'status': status,
                            'samples': len(recent), 'detail': detail}
        return result

    @staticmethod
    def _quality(name, values):
        columns = list(zip(*values))
        span = lambda i: max(columns[i]) - min(columns[i])
        med = lambda i: median(columns[i])
        ok, detail = True, 'Stable baseline'
        if name in ('lidar', 'us'):
            limit = .03 if name == 'lidar' else .04
            ok = min(columns[0]) > .02 and span(0) <= limit
            detail = f'range={med(0):.3f}m span={span(0):.3f}m'
        elif name in ('odom', 'map_tf'):
            drift = max(math.hypot(v[0] - values[0][0], v[1] - values[0][1]) for v in values)
            yaw = max(abs(wrap(v[2] - values[0][2])) for v in values)
            ok = drift <= .005 and yaw <= math.radians(3)
            if name == 'odom':
                ok = ok and max(columns[3]) <= .003
            detail = f'drift={drift:.4f}m yaw={math.degrees(yaw):.1f}deg'
        elif name == 'ir':
            ok = all(0 < min(col) < max(col) + 1 < 4001 and max(col) - min(col) <= 250
                     for col in columns)
            detail = 'IR baselines=' + ','.join(str(round(median(col))) for col in columns)
        elif name == 'imu':
            ok = 8.0 <= med(0) <= 11.5 and max(columns[1]) < .15 and max(abs(v) for v in columns[2]) < math.radians(20)
            detail = f'gravity={med(0):.2f}m/s2 gyro={max(columns[1]):.3f}rad/s'
        elif name == 'camera':
            ok = 5 <= med(0) <= 250 and med(1) >= 2
            detail = f'brightness={med(0):.1f} contrast={med(1):.1f}'
        elif name == 'tf':
            ok = all(abs(wrap(v[0])) <= math.radians(30) for v in values)
            detail = f'nose alignment error={math.degrees(med(0)):.1f}deg'
            if len(columns) > 1:
                detail += f' TF nose={math.degrees(med(1)):.1f}deg'
        elif name == 'map':
            ok = med(0) > 0 and med(1) > 0
            detail = f'known cells={int(med(0))}, resolution={med(1):.3f}m'
        return ('ok' if ok else 'unstable'), detail


def motion_evidence(start, current):
    x0, y0, yaw0 = start['odom'][:3]
    x, y, yaw = current['odom'][:3]
    dx, dy = x - x0, y - y0
    forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    lidar_delta = start['lidar'][0] - current['lidar'][0]
    us_delta = start['us'][0] - current['us'][0]
    mx0, my0, myaw0 = start['map_tf'][:3]
    mx, my, myaw = current['map_tf'][:3]
    mdx, mdy = mx - mx0, my - my0
    return {'forward_m': forward, 'lateral_m': lateral, 'distance_m': math.hypot(dx, dy),
            'yaw_drift_rad': wrap(yaw - yaw0), 'lidar_delta_m': lidar_delta, 'us_delta_m': us_delta,
            'map_forward_m': mdx * math.cos(myaw0) + mdy * math.sin(myaw0),
            'map_lateral_m': -mdx * math.sin(myaw0) + mdy * math.cos(myaw0),
            'map_yaw_drift_rad': wrap(myaw - myaw0)}


def motion_result(evidence):
    forward = evidence['forward_m']
    tolerance = max(.018, .6 * abs(forward))
    checks = {
        'forward_response': .008 <= forward <= .045,
        'straight_response': abs(evidence['lateral_m']) <= .015 and abs(evidence['yaw_drift_rad']) <= .12,
        'lidar_agrees': evidence['lidar_delta_m'] >= .004 and abs(evidence['lidar_delta_m'] - forward) <= tolerance,
        'us_agrees': evidence['us_delta_m'] >= .004 and abs(evidence['us_delta_m'] - forward) <= tolerance,
        'map_pose_agrees': (evidence['map_forward_m'] >= .004 and
                           abs(evidence['map_forward_m'] - forward) <= .02 and
                           abs(evidence['map_lateral_m'] - evidence['lateral_m']) <= .02 and
                           abs(wrap(evidence['map_yaw_drift_rad'] - evidence['yaw_drift_rad'])) <= .1),
    }
    return all(checks.values()), checks
