"""Subject: image rows to metres on the floor. Pure geometry, no invented numbers.

Column-thirds obstacle fractions cannot be fused with anything: the lidar speaks
in metres and the camera speaks in percentages of a rectangle. This turns the one
pixel that matters -- the bottom edge of a region, where it touches the ground --
into a distance the safety gate can compare against a stopping distance.

Two assumptions, both borrowed from Ulrich & Nourbakhsh (AAAI-00) and both real
limits rather than formalities:

  flat floor       a slope shifts the contact point and the estimate with it
  no overhang      the base is what is measured, so a tabletop reads as the
                   distance to whatever is under it -- the ultrasonic owns that
                   case, exactly as that paper recommends

Sensitivity is dominated by pitch, and by the inverse-tangent blow-up near the
horizon: one row of segmentation jitter up there is metres of range error. So the
module refuses two things outright -- rows at or above the horizon, and anything
past a configured trustworthy range. Both come back None, which means unknown.
Unknown is not free.

Nothing here has a default. Without a measured height, pitch and focal length
there is no honest answer, and a plausible-looking fabricated distance is worse
than no distance at all: the caller keeps distance_m None and stays unranged.
"""
import math


class GroundPlane:
    """A calibrated pinhole above a flat floor. Angles in radians, lengths in metres."""

    def __init__(self, height_m, pitch_rad, focal_px, principal_x, principal_y,
                 max_range_m):
        self.height_m = float(height_m)
        self.pitch_rad = float(pitch_rad)
        self.focal_px = float(focal_px)
        self.principal_x = float(principal_x)
        self.principal_y = float(principal_y)
        self.max_range_m = float(max_range_m)

    @property
    def horizon_row(self):
        """Image row where the ground plane escapes to infinity. Below it is floor."""
        return self.principal_y - self.focal_px * math.tan(self.pitch_rad)

    def distance(self, row):
        """Metres ahead of the camera for an image row, or None when unknowable."""
        try:
            row = float(row)
        except (TypeError, ValueError):
            return None
        if not _finite(row):
            return None
        angle = self.pitch_rad + math.atan((row - self.principal_y) / self.focal_px)
        # At or above the horizon the ray never meets the floor. Below it, tan
        # grows without bound as the ray steepens, which is the well-behaved end.
        if angle <= 0.0:
            return None
        metres = self.height_m / math.tan(angle)
        if not _finite(metres) or metres <= 0.0 or metres > self.max_range_m:
            return None
        return metres

    def lateral(self, column, row):
        """Metres left(-)/right(+) of the optical axis for a pixel on the floor.

        Needs the row as well as the column: the naive Z*(u-cx)/f is the
        zero-pitch approximation, and with the camera pitched down it under-reads
        by 4% at mid-frame and 16% at the bottom row on the simulated rig. The
        pitched form divides by the ray's foreshortened forward component.
        """
        try:
            column, row = float(column), float(row)
        except (TypeError, ValueError):
            return None
        if not _finite(column, row) or self.distance(row) is None:
            return None
        # Cannot be zero here: it equals f*cos(pitch)*(tan(pitch) + (row-cy)/f),
        # and distance(row) returning a value already established that the second
        # factor is positive, with cos(pitch) > 0 for the pitches ground_plane
        # accepts. So no guard -- an unreachable branch is a lie about the maths.
        denominator = (self.focal_px * math.sin(self.pitch_rad) +
                       (row - self.principal_y) * math.cos(self.pitch_rad))
        return self.height_m * (column - self.principal_x) / denominator

    def region_distance(self, bbox_xyxy):
        """Range to a region, measured at the bottom edge where it meets the floor.

        Never the centroid: the centroid of a tall object sits well above its
        contact point, and the resulting overestimate is an error in the direction
        that causes collisions.
        """
        try:
            bottom = bbox_xyxy[3]
        except (TypeError, IndexError, KeyError):
            return None
        return self.distance(bottom)


def ground_plane(height_m=None, pitch_rad=None, focal_px=None, principal_x=None,
                 principal_y=None, max_range_m=None):
    """Build a usable GroundPlane, or None when the calibration is absent or unusable.

    Returning None is the whole point: an uncalibrated camera must keep saying
    "unranged" rather than start emitting numbers nobody measured.
    """
    try:
        values = [float(v) for v in
                  (height_m, pitch_rad, focal_px, principal_x, principal_y, max_range_m)]
    except (TypeError, ValueError):
        return None
    height, pitch, focal, _cx, _cy, max_range = values
    if not _finite(*values):
        return None
    # A camera at or below the floor, looking level or up, or with no focal
    # length, describes no ground plane at all.
    if height <= 0.0 or focal <= 0.0 or max_range <= 0.0:
        return None
    if not 0.0 < pitch < math.pi / 2:
        return None
    return GroundPlane(height, pitch, focal, _cx, _cy, max_range)


def focal_from_hfov(width_px, hfov_rad):
    """Focal length in pixels from a lens spec, for a first estimate before calibration.

    The OV5647 with the standard 3.6 mm lens is about 54 deg horizontally. This is
    a starting point for the checkerboard capture, not a substitute for it.
    """
    try:
        width_px, hfov_rad = float(width_px), float(hfov_rad)
    except (TypeError, ValueError):
        return None
    if not _finite(width_px, hfov_rad) or width_px <= 0 or not 0 < hfov_rad < math.pi:
        return None
    return (width_px / 2.0) / math.tan(hfov_rad / 2.0)


def _finite(*values):
    """Shared with camera_controls: ints are fine, NaN and the infinities are not."""
    try:
        return all(float(v) == float(v) and
                   float(v) not in (float('inf'), float('-inf')) for v in values)
    except (TypeError, ValueError, OverflowError):
        return False
