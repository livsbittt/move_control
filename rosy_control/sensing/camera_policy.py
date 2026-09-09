"""Subject: the verdict, kept out of the measurement.

classify_frame reports what a frame looked like. It does not know how many
frames have agreed, whether the reference has bootstrapped yet, or that an
unreadable frame must hold rather than clear. Those are policy, and policy needs
memory, so it lives here instead of being recomputed by every caller.

It was recomputed by every caller, and they disagreed: camera_detect_node.tick
counted `hits` from a parameter and gated on `warmup_frames`, while
tools/gz/rendered_camera_adapter.on_image hardcoded hits >= 2, had no warmup, and
applied no hysteresis to cliff at all. A green Gazebo run therefore said nothing
about the node that drives the robot. One implementation, both callers.
"""
from .camera_evidence import legacy_flags


class CameraPolicy:
    """Frame verdicts with hysteresis. Holds on blindness, never clears on it."""

    def __init__(self, hits=2, warmup_frames=12):
        self.hits = max(1, int(hits))
        self.warmup_frames = max(0, int(warmup_frames))
        self.cliff = False
        self.blocked = False
        self._cliff_hits = 0
        self._block_hits = 0
        self._warmup = 0

    @property
    def ready(self):
        """False while the floor reference is still bootstrapping."""
        return self._warmup >= self.warmup_frames

    def update(self, result):
        """Fold one classify_frame result in. Returns (cliff, blocked).

        An unusable frame is not evidence of a new cliff, but it is a reason to
        stop: blindness holds. Until the reference has seen `warmup_frames` usable
        frames its notion of "floor" is a guess, so the answer stays blocked.
        """
        cliff, blocked = legacy_flags(result, self.cliff)
        valid = bool(result.get('quality', {}).get('valid', False))
        if not valid:
            self.blocked = True
        self._warmup += int(valid)
        if not self.ready:
            return self.cliff, True
        self._cliff_hits = self._cliff_hits + 1 if cliff else 0
        self._block_hits = self._block_hits + 1 if blocked else 0
        # Rising needs `hits` consecutive frames; falling needs a clean one. The
        # asymmetry is deliberate: a missed obstacle costs more than a late start.
        if self._cliff_hits >= self.hits:
            self.cliff = True
        elif self._cliff_hits == 0:
            self.cliff = False
        if self._block_hits >= self.hits:
            self.blocked = True
        elif self._block_hits == 0:
            self.blocked = False
        return self.cliff, self.blocked

    def reset(self):
        """Re-bootstrap, e.g. after the sensor gains were frozen underneath us.

        Explicit rather than a second __init__ call, so adding a constructor
        argument later cannot quietly turn a reset into a reconstruction.
        """
        self.cliff = False
        self.blocked = False
        self._cliff_hits = 0
        self._block_hits = 0
        self._warmup = 0
