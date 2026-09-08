"""Bound separate recovery regions using continuous measured odometry."""
import math


class EscapeBudget:
    def __init__(self):
        self.previous=None
        self.attempts=[]
        self.completed=False
        self.failed=False
        self.travel=0.

    def observe(self,now,pose):
        try:
            if len(pose)<2 or not all(math.isfinite(v) for v in (now,*pose)):
                raise ValueError()
            if self.previous is not None:
                stamp,point=self.previous
                dt=now-stamp; step=math.dist(pose[:2],point)
                if not 0<=dt<=.5 or step>.005+.03*dt:raise ValueError()
                if self.completed:self.travel+=step
            self.previous=(now,tuple(pose[:2]))
        except (TypeError,ValueError,OverflowError):
            if self.attempts:self.failed=True
            self.previous=None

    def eligible(self,now):
        return bool(not self.failed and self.previous is not None and
            0<=now-self.previous[0]<=.5 and len(self.attempts)<4 and
            (not self.attempts or self.completed and self.travel>=.25 and
             all(math.dist(self.previous[1],point)>=.25 for _,point in self.attempts)))

    def begin(self,identity,now):
        if not self.eligible(now) or any(identity==old for old,_ in self.attempts):return False
        self.attempts.append((identity,self.previous[1]))
        self.completed=False; self.travel=0.
        return True

    def complete(self):
        if not self.failed and self.attempts:
            self.completed=True
