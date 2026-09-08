"""Bounded left/right trials with independent repeated response validation."""
import math


class RotationTrial:
    def __init__(self, now):
        self.started = self.last_time = self.leg_started = now
        self.index = 0
        self.targets = [math.radians(10), 0., -math.radians(10), 0.]*2
        self.scales = [1., 1.]  # positive / negative angular direction
        self.legs = []
        self.origin = self.commanded = self.last_speed = 0.
        self.settle_until = None
        self.done = False
        self.error = None

    def fail(self, reason):
        self.error, self.last_speed = reason, 0.
        return 0.

    def update(self, now, lidar_yaw, imu_yaw, odom_yaw, translation, clear):
        if self.error or self.done:
            return 0.
        values = (now, lidar_yaw, imu_yaw, odom_yaw, translation)
        if not clear or not all(math.isfinite(v) for v in values):
            return self.fail('Rotation observation or clearance unavailable')
        dt = now-self.last_time
        if not 0 <= dt <= .5 or now-self.started > 60 or now-self.leg_started > 7:
            return self.fail('Rotation interrupted or stalled')
        self.commanded += abs(self.last_speed)*dt
        self.last_time = now
        if (translation > .008 or abs(lidar_yaw) > math.radians(15) or
                max(abs(lidar_yaw-imu_yaw), abs(lidar_yaw-odom_yaw)) > math.radians(3)):
            return self.fail('Rotation sensors disagree or travel envelope exceeded')
        target = self.targets[self.index]
        direction = 1 if target > self.origin else -1
        if self.settle_until is not None:
            self.last_speed = 0.
            if now < self.settle_until:
                return 0.
            measured = direction*(lidar_yaw-self.origin)
            if measured < math.radians(7) or abs(lidar_yaw-target) > math.radians(2):
                return self.fail('Rotation endpoint or return check failed')
            ratio = self.commanded/measured
            slot = int(direction < 0)
            if not .75 <= ratio <= 1.25:
                return self.fail('Rotation correction outside trial bounds')
            if self.index >= 4 and abs(ratio-self.scales[slot]) > .12:
                return self.fail('Rotation repeat disagrees with estimated response')
            self.legs.append({'direction': direction, 'measured_rad': measured,
                              'commanded_rad': self.commanded, 'ratio': ratio})
            if self.index < 4:
                samples = [leg['ratio'] for leg in self.legs if leg['direction'] == direction]
                if max(samples)-min(samples) > .12:
                    return self.fail('Rotation directional response is inconsistent')
                self.scales[slot] = sum(samples)/len(samples)
            self.index += 1
            self.done = self.index == len(self.targets)
            self.origin, self.commanded, self.leg_started = lidar_yaw, 0., now
            self.settle_until = None
            return 0.
        if direction*(target-lidar_yaw) <= math.radians(.5):
            self.settle_until = now+.6
            self.last_speed = 0.
            return 0.
        gain = self.scales[int(direction < 0)] if self.index >= 4 else 1.
        # The explicit trial lease caps commands at 0.06 rad/s. Integrate the
        # emitted bounded command so slower response still yields its true gain.
        self.last_speed = direction*min(.06,.06*gain)
        return self.last_speed

    def report(self):
        return {'done': self.done, 'error': self.error, 'legs': self.legs,
                'angular_gains': self.scales, 'max_angular_rad_s': .06,
                'geometry_commissioned': False}
