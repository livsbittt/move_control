"""Bounded execution feedback: repeated paths are not successful recovery."""
import math


class RouteRecovery:
    def __init__(self, wait_seconds=5., progress_seconds=45., max_attempts=3):
        self.wait_seconds, self.progress_seconds = wait_seconds, progress_seconds
        self.max_attempts = max_attempts
        self.reset()

    def reset(self):
        self.anchor = self.budget_anchor = None
        self.progress_since = self.blocked_since = None
        self.hold_since = None
        self.requested = None
        self.failed_goal = None
        self.failed_exit = None
        self.failed_reason = None
        self.attempts = 0
        self.waiting = self.exhausted = False
        self.alignment_heading = self.alignment_best_error = None

    def update(self, now, pose, reason, goal, route_stamp, safe, route_exit=None, *, time_bounded=False):
        if not safe or pose is None:
            if self.hold_since is None:
                self.hold_since = now
            return 'safety_hold'
        if self.hold_since is not None:
            duration = max(0., now-self.hold_since)
            for name in ('progress_since', 'blocked_since', 'requested'):
                stamp = getattr(self, name)
                if stamp is not None:
                    setattr(self, name, stamp+duration)
            self.hold_since = None
        if self.exhausted:
            return 'exhausted'
        xy = tuple(pose[:2])
        if self.anchor is None:
            self.anchor = self.budget_anchor = xy
            self.progress_since = now
        if math.dist(xy,self.anchor) >= .02:
            self.anchor, self.progress_since = xy, now
            self.alignment_heading = self.alignment_best_error = None
        if math.dist(xy,self.budget_anchor) >= .10:
            self.budget_anchor, self.attempts = xy, 0
        if self.waiting:
            different = goal is not None and (self.failed_goal is None or math.dist(goal,self.failed_goal)>.08)
            if route_exit is not None and self.failed_exit is not None:
                different = math.dist(route_exit,self.failed_exit)>.05
            if different and route_stamp is not None and route_stamp > self.requested:
                self.waiting = False
                self.failed_goal = self.failed_exit = None
                self.failed_reason = None
                self.progress_since, self.blocked_since = now, None
                self.alignment_heading = self.alignment_best_error = None
                return 'alternative'
            # Waiting for a distinct executable route is not a failed drive.
            return 'waiting'
        else:
            if reason in ('align', 'turn_away') and len(pose) >= 3:
                target = route_exit if route_exit is not None else goal
                if (self.alignment_heading is None and target is not None
                        and math.dist(xy, target) > .025
                        and all(math.isfinite(v) for v in (*target, pose[2]))):
                    # Lock the bearing for this translation episode. A
                    # refreshed/drifting goal must not manufacture progress.
                    self.alignment_heading = math.atan2(target[1]-xy[1], target[0]-xy[0])
                    delta = self.alignment_heading-pose[2]
                    self.alignment_best_error = abs(math.atan2(math.sin(delta), math.cos(delta)))
                if self.alignment_heading is not None and math.isfinite(pose[2]):
                    delta = self.alignment_heading-pose[2]
                    error = abs(math.atan2(math.sin(delta), math.cos(delta)))
                    if self.alignment_best_error-error >= .05:
                        self.alignment_best_error = error
                        self.progress_since = now
            blocked = reason in ('no_route','hazard','front_blocked','stalled_restart_required',
                                 'stale_route','off_route','arrived','gate_rotation_blocked')
            if blocked:
                if self.blocked_since is None:
                    self.blocked_since = now
            else:
                self.blocked_since = None
            due = (self.blocked_since is not None and now-self.blocked_since>=self.wait_seconds)
            if not due and now-self.progress_since < self.progress_seconds:
                return 'following'
        if not time_bounded and self.attempts >= self.max_attempts:
            self.exhausted = True
            return 'exhausted'
        self.attempts += 1
        self.waiting, self.requested = True, now
        # A revoked route has no failed executable exit. Carrying an older
        # episode's exit here can veto every fresh route after map replanning.
        self.failed_goal = goal
        self.failed_exit = route_exit
        self.failed_reason = reason
        return 'replan'
