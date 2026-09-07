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
        self.requested = None
        self.failed_goal = None
        self.failed_exit = None
        self.heading_target = self.heading_best = None
        self.hold_since = None
        self.attempts = 0
        self.waiting = self.exhausted = False

    def update(self, now, pose, reason, goal, route_stamp, safe, route_exit=None):
        if not safe or pose is None:
            if self.hold_since is None:
                self.hold_since = now
            return 'safety_hold'
        if self.hold_since is not None:
            duration = max(0., now-self.hold_since)
            if self.progress_since is not None:
                self.progress_since += duration
            if self.blocked_since is not None:
                self.blocked_since += duration
            self.hold_since = None
        if self.exhausted:
            return 'exhausted'
        xy = tuple(pose[:2])
        if self.anchor is None:
            self.anchor = self.budget_anchor = xy
            self.progress_since = now
        if math.dist(xy,self.anchor) >= .02:
            self.anchor, self.progress_since = xy, now
            self.heading_target = self.heading_best = None
        if math.dist(xy,self.budget_anchor) >= .10:
            self.budget_anchor, self.attempts = xy, 0
        if self.waiting:
            different = goal is not None and (self.failed_goal is None or math.dist(goal,self.failed_goal)>.08)
            if route_exit is not None and self.failed_exit is not None:
                different = math.dist(route_exit,self.failed_exit)>.05
            if different and route_stamp is not None and route_stamp > self.requested:
                self.waiting = False
                self.progress_since, self.blocked_since = now, None
                self.heading_target = self.heading_best = None
                return 'alternative'
            # The planner continuously replans while its temporary exclusions
            # expire. Waiting at zero speed is not a failed physical attempt.
            return 'waiting'
        else:
            if reason in ('align', 'turn_away') and route_exit is not None:
                if self.heading_target is None:
                    self.heading_target = math.atan2(route_exit[1]-xy[1], route_exit[0]-xy[0])
                    self.heading_best = abs(math.atan2(math.sin(self.heading_target-pose[2]),
                                                       math.cos(self.heading_target-pose[2])))
                error = abs(math.atan2(math.sin(self.heading_target-pose[2]),
                                       math.cos(self.heading_target-pose[2])))
                # A safety-limited turn can legitimately exceed 45 seconds.
                # Credit only convergence toward a fixed bearing, never total
                # yaw travel (which would excuse endless circles).
                if self.heading_best-error >= .15:
                    self.heading_best, self.progress_since = error, now
            blocked = reason in ('no_route','hazard','front_blocked','stalled_restart_required',
                                 'stale_route','off_route','arrived')
            if blocked:
                if self.blocked_since is None:
                    self.blocked_since = now
            else:
                self.blocked_since = None
            due = (self.blocked_since is not None and now-self.blocked_since>=self.wait_seconds)
            if not due and now-self.progress_since < self.progress_seconds:
                return 'following'
        if self.attempts >= self.max_attempts:
            self.exhausted = True
            return 'exhausted'
        self.attempts += 1
        self.waiting, self.requested = True, now
        if goal is not None:
            self.failed_goal = goal
        if route_exit is not None:
            self.failed_exit = route_exit
        return 'replan'
