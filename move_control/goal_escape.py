"""Simulation-only fixed-heading escape transport around the circle planner."""
import json
import math
import os
import time
import uuid
from std_msgs.msg import String
from .planning.straight_escape import straight_escape


class GoalEscape:
    def init_escape(self):
        self.escape_limits=None
        self.escape_seen=None
        self.escape_intent=None
        self.escape_used=False
        self.escape_pub=self.create_publisher(String,'/goal/straight_escape',10)
        self.create_subscription(String,'/safety/motion_limits',self.on_escape_limits,10)

    def on_escape_limits(self,msg):
        try:
            self.escape_limits=json.loads(msg.data)
            self.escape_seen=time.monotonic()
        except (TypeError,ValueError):
            self.escape_limits=None

    def publish_escape(self,intent):
        self.escape_pub.publish(String(data=json.dumps(intent)))

    def escape_plan(self,m,pose,status=None):
        """Return true when a separate straight intent owns this planning tick."""
        active=self.escape_intent
        if not active and (self.escape_used or status!='planning blocked: robot inside obstacle clearance'):
            return False
        limits=self.escape_limits
        valid=(os.environ.get('ROS_DOMAIN_ID')=='227' and os.environ.get('GZ_PARTITION')=='pinky_calmap227'
            and self.get_parameter('use_sim_time').value and isinstance(limits,dict)
            and self.escape_seen is not None and 0<=time.monotonic()-self.escape_seen<=.5
            and limits.get('bounded_geometry')=='trusted_footprint'
            and limits.get('bounded_motion_enabled') is True and limits.get('rotation_scan_observed') is True)
        target=None
        if valid:
            report=limits.get('rotation_estimate') or {}
            try:
                margin=max(.01,self.navigation_profile['uncertainty_margin_m']-m.res/2)+2*report['center_uncertainty_m']
                if active and (active['geometry']!=limits['geometry_revision'] or
                        abs(math.atan2(math.sin(pose[2]-active['yaw']),math.cos(pose[2]-active['yaw'])))>.02):
                    valid=False
                if active and math.dist(pose[:2],active['target'])<=.003:
                    self.escape_intent=None
                    self.publish_escape(None)
                    return False
                if valid:
                    target=straight_escape(m,pose,report['footprint_xy'],margin,
                        self.brain.start_escape_clear_m,active['target'] if active else None,
                        limits['planning_escape_limits_m'])
            except (KeyError,TypeError,ValueError):
                target=None
        if target is None:
            if active:
                self.escape_intent=None
                self.publish_escape(None)
                self._clear_route('planning blocked: straight escape evidence lost')
                return True
            return False
        if active is None:
            self.escape_used=True
            active=dict(id=uuid.uuid4().hex,target=target,yaw=pose[2],geometry=limits['geometry_revision'])
            self.escape_intent=active
        self._clear_route('escape: fixed-heading translation to planning clearance')
        self.publish_escape({**active,'issued_s':self.get_clock().now().nanoseconds*1e-9})
        return True
