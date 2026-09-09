"""LiDAR-referenced short round trip and independent corrected repeat."""
import math
from .calibration import motion_evidence


class RoundTrip:
    def __init__(self, now, snapshot, target=.03):
        if not .02 <= target <= .04:
            raise ValueError('Round-trip target must be 2 to 4 cm')
        self.target = target
        self.origin = snapshot
        self.leg_origin = snapshot
        self.started = self.leg_started = now
        self.stage = 'outbound'
        self.cycle = 0
        self.scales = [1., 1.]
        self.legs = []
        self.error = None
        self.done = False
        self.settle_until = None
        self.commanded = 0.
        self.last_time = now
        self.last_speed = 0.
        self.last_leg_evidence = None

    def fail(self, reason):
        self.error = reason
        return 0.

    def pause(self, now):
        """Account the last issued command, then hold zero without fake travel."""
        if now-self.started > 35.:
            return self.fail('Round-trip total timeout')
        if not self.stage.startswith('settle') and now-self.leg_started > 7.:
            return self.fail('Round-trip leg stalled or timed out')
        dt = now - self.last_time
        if dt < 0 or dt > .5:
            return self.fail('Round-trip control updates interrupted')
        self.commanded += abs(self.last_speed) * dt
        self.last_time = now
        self.last_speed = 0.
        return 0.

    def update(self, now, snapshot):
        if self.error or self.done:
            return 0.
        dt = now - self.last_time
        if dt < 0 or dt > .5:
            return self.fail('Round-trip control updates interrupted')
        self.commanded += abs(self.last_speed) * dt
        self.last_time = now
        if now - self.started > 35.:
            return self.fail('Round-trip total timeout')
        home = motion_evidence(self.origin, snapshot)
        if abs(home['lateral_m']) > .015 or abs(home['yaw_drift_rad']) > .10:
            return self.fail('Round-trip lateral or heading drift')
        if home['forward_m'] < -.008 or home['distance_m'] > .05:
            return self.fail('Round-trip travel envelope exceeded')
        if self.stage.startswith('settle'):
            self.last_speed = 0.
            if now < self.settle_until:
                return 0.
            returning = self.stage == 'settle_home'
            leg = motion_evidence(self.leg_origin, snapshot)
            measured = (-1 if returning else 1) * leg['lidar_delta_m']
            odom = (-1 if returning else 1) * leg['forward_m']
            self.last_leg_evidence = {
                'direction': 'reverse' if returning else 'forward',
                'measured_m': measured, 'odom_m': odom,
                'commanded_m': self.commanded,
                'ratio': self.commanded / measured if measured > 0 else None,
                'evidence': leg,
            }
            if measured < .018 or abs(measured - odom) > .012:
                return self.fail('LiDAR and wheel distance disagree during round trip')
            # SLAM can change the global map->odom correction during a leg.
            # It is derived from these same sensors, not independent metrology.
            # Keep the discrepancy visible for navigation without invalidating
            # the bounded LiDAR/wheel measurement and corrected return repeat.
            map_consistent = (abs(leg['map_forward_m']-leg['forward_m']) <= .015 and
                              abs(leg['map_lateral_m']-leg['lateral_m']) <= .015 and
                              abs(leg['map_yaw_drift_rad']-leg['yaw_drift_rad']) <= .1)
            ratio = self.commanded / measured
            if not .75 <= ratio <= 1.25:
                return self.fail('Required speed correction exceeds calibrated bounds')
            if self.cycle == 1 and abs(measured - self.commanded / self.scales[int(returning)]) > .006:
                return self.fail('Corrected speed failed independent repeat')
            self.legs.append({'cycle': self.cycle, 'direction': 'reverse' if returning else 'forward',
                              'measured_m': measured, 'odom_m': odom, 'commanded_m': self.commanded,
                              'home_error_m': home['lidar_delta_m'], 'evidence': leg,
                              'map_pose_consistent': map_consistent})
            if self.cycle == 0:
                self.scales[int(returning)] = ratio
            if returning:
                if abs(home['lidar_delta_m']) > .006 or abs(home['forward_m']) > .008:
                    return self.fail('Robot did not return to calibration origin')
                if self.cycle == 1:
                    self.done = True
                    return 0.
                self.cycle = 1
            self.stage = 'outbound' if returning else 'returning'
            self.leg_origin = snapshot
            self.leg_started = now
            self.commanded = 0.
            return 0.
        if now - self.leg_started > 7.:
            return self.fail('Round-trip leg stalled or timed out')
        returning = self.stage == 'returning'
        distance = home['lidar_delta_m']
        if (not returning and distance >= self.target - .001) or (returning and distance <= .001):
            self.stage = 'settle_home' if returning else 'settle_out'
            self.settle_until = now + .6
            self.last_speed = 0.
            return 0.
        scale = self.scales[int(returning)] if self.cycle else 1.
        self.last_speed = (-1 if returning else 1) * .008 * scale
        return self.last_speed

    def report(self):
        return {'stage': self.stage, 'cycle': self.cycle + 1, 'target_m': self.target,
                'forward_scale': self.scales[0], 'reverse_scale': self.scales[1],
                'legs': self.legs, 'done': self.done, 'error': self.error,
                'localization_consistent': all(leg['map_pose_consistent'] for leg in self.legs) if self.legs else None,
                'last_leg_evidence': self.last_leg_evidence}
