"""One measured straight relocation per navigation region, with no ROS state."""
import math


class RotationRelocation:
    """Caller supplies only fresh odometry and fresh safety-approved suggestions.

    ``safe`` must include directional clearance, localization and hazard checks
    on every tick. ``None`` pose/suggestion means unavailable or expired evidence.
    A returned speed is a straight-only override: the caller must command zero
    angular velocity. None speed means no override. Reset only on new navigation.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.active = False
        self.direction = 0
        self.target_distance = 0.
        self.budget_anchor = None
        self.used = False
        self.started = self.start_pose = None

    def _stop(self, reason):
        self.active = False
        self.direction = 0
        return 0., 'rotation_relocation_' + reason

    def update(self, now, pose, safe, can_rotate, suggestion, requested_turn):
        valid_pose = (pose is not None and len(pose) == 3 and
                      all(math.isfinite(v) for v in pose))
        valid_suggestion = (isinstance(suggestion, (int, float)) and
                            math.isfinite(suggestion) and 0 < abs(suggestion) <= .03)
        if self.active:
            if not safe:
                return self._stop('unsafe')
            if not valid_pose:
                return self._stop('stale_pose')
            if can_rotate:
                return self._stop('clear')
            if not valid_suggestion:
                return self._stop('stale_suggestion')
            if suggestion*self.direction <= 0:
                return self._stop('direction_changed')
            if not math.isfinite(now) or now < self.started or now-self.started >= 8.:
                return self._stop('timeout')
            delta = pose[2]-self.start_pose[2]
            if abs(math.atan2(math.sin(delta), math.cos(delta))) > .05:
                return self._stop('heading')
            # Count all displacement conservatively, including lateral slip.
            if math.dist(pose[:2], self.start_pose[:2]) >= self.target_distance:
                return self._stop('distance')
            return self.direction*.006, 'rotation_relocation'
        if valid_pose and self.budget_anchor is not None and math.dist(pose[:2], self.budget_anchor) >= .10:
            self.used = False
            self.budget_anchor = tuple(pose[:2])
        if (self.used or not safe or not valid_pose or not valid_suggestion or
                can_rotate or not requested_turn or not math.isfinite(now)):
            return None, 'inactive'
        self.active = self.used = True
        self.budget_anchor = tuple(pose[:2])
        self.start_pose = tuple(pose)
        self.started = now
        self.direction = 1 if suggestion > 0 else -1
        self.target_distance = abs(suggestion)
        return self.direction*.006, 'rotation_relocation'
