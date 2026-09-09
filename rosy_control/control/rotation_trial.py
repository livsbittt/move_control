"""Bounded left/right trials with independent repeated response validation."""
import math

MAX_TRANSLATION_M = .008


def steady_response(samples, origin, direction, started):
    rows = [row for row in samples if math.radians(1.5) <= direction*(row[1]-origin) <= math.radians(8.5)]
    if len(rows) < 8 or direction*(rows[-1][1]-rows[0][1]) < math.radians(5):
        return None
    times = [row[0]-started for row in rows]
    center = sum(times)/len(times)
    variance = sum((t-center)**2 for t in times)
    if variance <= 0:
        return None
    rates = []
    for column in (1, 2, 3):
        values = [direction*row[column] for row in rows]
        mean = sum(values)/len(values)
        rate = sum((t-center)*(v-mean) for t, v in zip(times, values))/variance
        residual = math.sqrt(sum((v-mean-rate*(t-center))**2 for t,v in zip(times,values))/len(rows))
        if rate <= 0 or residual > math.radians(.5):
            return None
        rates.append(rate)
    if max(rates)-min(rates) > .2*rates[0]:
        return None
    progress = sum(direction*(row[1]-origin) for row in rows)/len(rows)
    return {'ratio': .06/rates[0], 'steady_rate_rad_s': rates[0],
            'sensor_rates_rad_s': rates, 'onset_latency_s': center-progress/rates[0],
            'fit_samples': len(rows), 'fit_span_rad': direction*(rows[-1][1]-rows[0][1])}


class RotationTrial:
    def __init__(self, now, endpoint_refinement=False):
        self.started = self.last_time = self.leg_started = now
        self.index = 0
        self.endpoint_refinement = endpoint_refinement
        self.targets = [math.radians(10), 0., -math.radians(10), 0.]*2
        if endpoint_refinement:
            self.targets += [math.radians(10), 0.]
        self.scales = [1., 1.]  # positive / negative angular direction
        self.legs = []
        self.origin = self.commanded = self.last_speed = 0.
        self.settle_until = None
        self.done = False
        self.error = None
        self.observation = None
        self.failed_leg = None
        self.response_samples = []
        self.command_started = None

    def fail(self, reason):
        self.failed_leg = dict(self.observation or {})
        self.failed_leg.update(index=self.index, commanded_rad=self.commanded,
                               origin_rad=self.origin, reason=reason)
        self.error, self.last_speed = reason, 0.
        return 0.

    def update(self, now, lidar_yaw, imu_yaw, odom_yaw, translation, clear):
        if self.error or self.done:
            return 0.
        self.observation = {'elapsed_s': now-self.started, 'leg_elapsed_s': now-self.leg_started,
                            'lidar_yaw_rad': lidar_yaw, 'imu_yaw_rad': imu_yaw,
                            'odom_yaw_rad': odom_yaw, 'translation_m': translation,
                            'clear': clear, 'target_rad': self.targets[self.index]}
        values = (now, lidar_yaw, imu_yaw, odom_yaw, translation)
        if not clear or not all(math.isfinite(v) for v in values):
            return self.fail('Rotation observation or clearance unavailable')
        dt = now-self.last_time
        if not 0 <= dt <= .5 or now-self.started > 60 or now-self.leg_started > 7:
            return self.fail('Rotation interrupted or stalled')
        self.commanded += abs(self.last_speed)*dt
        self.last_time = now
        if (translation > MAX_TRANSLATION_M or abs(lidar_yaw) > math.radians(15) or
                max(abs(lidar_yaw-imu_yaw), abs(lidar_yaw-odom_yaw)) > math.radians(3)):
            return self.fail('Rotation sensors disagree or travel envelope exceeded')
        target = self.targets[self.index]
        direction = 1 if target > self.origin else -1
        if self.settle_until is None and self.command_started is not None:
            if not self.response_samples or lidar_yaw != self.response_samples[-1][1]:
                self.response_samples.append((now, lidar_yaw, imu_yaw, odom_yaw))
        if self.settle_until is not None:
            self.last_speed = 0.
            if now < self.settle_until:
                return 0.
            measured = direction*(lidar_yaw-self.origin)
            if measured < math.radians(7) or abs(lidar_yaw-target) > math.radians(2):
                return self.fail('Rotation endpoint or return check failed')
            response = steady_response(self.response_samples, self.origin, direction, self.command_started)
            self.observation.update(direction=direction, measured_rad=measured,
                                    integrated_ratio=self.commanded/measured)
            if response is None:
                return self.fail('Rotation steady response insufficient or inconsistent')
            self.observation.update(response)
            ratio = response['ratio']
            if not -.2 <= response['onset_latency_s'] <= 1.:
                return self.fail('Rotation onset latency outside trial bounds')
            slot = int(direction < 0)
            if not .75 <= ratio <= 1.25:
                return self.fail('Rotation correction outside trial bounds')
            if self.index >= 4 and abs(ratio-self.scales[slot]) > .12:
                return self.fail('Rotation repeat disagrees with estimated response')
            self.legs.append({'direction': direction, 'measured_rad': measured,
                              'commanded_rad': self.commanded, 'integrated_ratio': self.commanded/measured,
                              **response})
            if self.index < 4:
                samples = [leg['ratio'] for leg in self.legs if leg['direction'] == direction]
                if max(samples)-min(samples) > .12:
                    return self.fail('Rotation directional response is inconsistent')
                self.scales[slot] = sum(samples)/len(samples)
            self.index += 1
            self.done = self.index == len(self.targets)
            self.origin, self.commanded, self.leg_started = lidar_yaw, 0., now
            self.settle_until = None
            self.response_samples = []
            self.command_started = None
            return 0.
        if direction*(target-lidar_yaw) <= math.radians(.5):
            self.settle_until = now+.6
            self.last_speed = 0.
            return 0.
        if self.command_started is None:
            self.command_started = now
        # Repeat the same bounded stimulus to validate response stability.
        # Compensation is deliberately not claimed or applied by this trial.
        self.last_speed = direction*.06
        return self.last_speed

    def report(self):
        report = {'done': self.done, 'error': self.error, 'legs': self.legs,
                'failed_leg': self.failed_leg,
                'angular_gains': [1., 1.], 'estimated_steady_gains': self.scales,
                'compensation_verified': False, 'max_angular_rad_s': .06,
                'response_verified': self.done and self.error is None,
                'geometry_commissioned': False}
        if self.endpoint_refinement:
            report['trial_sequence'] = 'cross_endpoint_v2'
            report['target_sequence_deg'] = [10,0,-10,0,10,0,-10,0,10,0]
        return report
