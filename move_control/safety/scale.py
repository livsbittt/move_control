"""Subject: narrow-map auto-cal. Corridor width from L+R → map_range / open_max.

A 50–120 cm map is too wide for a desk maze. Safety measures the live
corridor and publishes a tight HUD / opening cap.
"""
from std_msgs.msg import Float32


def _finite(d, lo, hi):
    try:
        x = float(d)
    except (TypeError, ValueError):
        return False
    return x == x and lo < x < hi


def corridor_width(left, right, lo=0.05, hi=0.80):
    """L+R when both sides look like maze walls, else None."""
    if not (_finite(left, lo, hi) and _finite(right, lo, hi)):
        return None
    w = float(left) + float(right)
    if w < 0.12 or w > 0.70:
        return None
    return w


def fit_map(corridor, scale=1.35, lo=0.18, hi=0.40):
    return max(lo, min(hi, float(corridor) * float(scale)))


def fit_open_max(corridor, scale=1.55, lo=0.20, hi=0.50):
    return max(lo, min(hi, float(corridor) * float(scale)))


class Scale:
    """Mixin. Call _scale_update(left, right) after filtered L/R."""

    def _scale_update(self, left, right):
        if not bool(self.get_parameter('auto_map').value):
            self.map_pub.publish(Float32(data=float(self.map_range)))
            self.open_max_pub.publish(Float32(data=float(self.open_max)))
            self.corr_pub.publish(Float32(data=-1.0))
            return
        w = corridor_width(left, right)
        if w is None:
            self.map_pub.publish(Float32(data=float(self.map_range)))
            self.open_max_pub.publish(Float32(data=float(self.open_max)))
            return
        self._corr_buf.append(w)
        if len(self._corr_buf) > 40:
            del self._corr_buf[0]
        if len(self._corr_buf) < 8:
            return
        s = sorted(self._corr_buf)
        med = s[len(s) // 2]
        lo_m = float(self.get_parameter('map_range_min').value)
        hi_m = float(self.get_parameter('map_range_max').value)
        map_r = fit_map(med, lo=lo_m, hi=hi_m)
        open_m = fit_open_max(med, lo=max(0.16, lo_m), hi=min(0.55, hi_m + 0.10))
        self.corr_pub.publish(Float32(data=float(med)))
        changed = (
            abs(map_r - self.map_range) >= 0.02
            or abs(open_m - self.open_max) >= 0.03
        )
        self.map_range = map_r
        self.open_max = open_m
        self.map_pub.publish(Float32(data=float(map_r)))
        self.open_max_pub.publish(Float32(data=float(open_m)))
        if changed:
            self.get_logger().info(
                f'auto map corridor={med*100:.0f}cm → map={map_r*100:.0f}cm '
                f'open_max={open_m*100:.0f}cm'
            )
