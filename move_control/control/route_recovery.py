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
        self.attempts = 0
        self.waiting = self.exhausted = False

    def update(self, now, pose, reason, goal, route_stamp, safe):
        if not safe or pose is None:
            return 'safety_hold'
        if self.exhausted:
            return 'exhausted'
        xy = tuple(pose[:2])
        if self.anchor is None:
            self.anchor = self.budget_anchor = xy
            self.progress_since = now
        if math.dist(xy,self.anchor) >= .02:
            self.anchor, self.progress_since = xy, now
        if math.dist(xy,self.budget_anchor) >= .10:
            self.budget_anchor, self.attempts = xy, 0
        if self.waiting:
            different = goal is not None and (self.failed_goal is None or math.dist(goal,self.failed_goal)>.08)
            if different and route_stamp is not None and route_stamp > self.requested:
                self.waiting = False
                self.progress_since, self.blocked_since = now, None
                return 'alternative'
            if now-self.requested < self.wait_seconds:
                return 'waiting'
        else:
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
        return 'replan'
