"""Atomic, expiring calibration evidence. Old split topics carry no authority."""
import hashlib
import json
import math
from .rotation_envelope import validate_envelope


def revision(packet):
    content = {k: v for k, v in packet.items() if k not in ('revision', 'sequence', 'issued_s')}
    return hashlib.sha256(json.dumps(content, sort_keys=True, allow_nan=False).encode()).hexdigest()[:16]


def make_profile(session, sequence, issued, enabled, gains, geometry, rotation=None, rotation_trial=False, translation_trial=False):
    packet = {'schema_version': 1, 'session': session, 'sequence': sequence,
              'issued_s': issued, 'ttl_s': 1.5, 'enabled': enabled,
              'linear_gains': list(gains), 'max_linear_mps': .014,
              'geometry_revision': geometry, 'rotation': rotation, 'rotation_trial': rotation_trial,
              'translation_trial': translation_trial,
              'domain': 'low_speed_straight', 'physical_commissioned': False}
    packet['revision'] = revision(packet)
    return packet


class ProfileLease:
    def __init__(self):
        self.active = None
        self.last_good = None
        self.session = None
        self.retired = set()
        self.sequence = -1
        self.deadline = 0.
        self.received = None
        self.reason = 'missing'
        self._rotation_required = False

    def accept(self, packet, wall_now, now, geometry):
        try:
            if not isinstance(packet, dict) or packet['schema_version'] != 1:
                raise ValueError('schema')
            if packet['revision'] != revision(packet):
                raise ValueError('revision')
            session, sequence = packet['session'], packet['sequence']
            if not isinstance(session, str) or not session or len(session) > 128:
                raise ValueError('session')
            if type(sequence) is not int or sequence < 1:
                raise ValueError('sequence')
            if session in self.retired or (session == self.session and sequence <= self.sequence):
                return False  # A replay cannot renew or revoke the current lease.
            ttl, issued = packet['ttl_s'], packet['issued_s']
            gains = packet['linear_gains']
            if (not all(math.isfinite(v) for v in (ttl, issued, wall_now, now)) or
                    not 0 < ttl <= 1.5 or not -.1 <= wall_now-issued <= ttl):
                raise ValueError('stale')
            if len(gains) != 2 or not all(math.isfinite(v) and .75 <= v <= 1.25 for v in gains):
                raise ValueError('gains')
            if (packet['geometry_revision'] != geometry or not geometry or
                    packet['domain'] != 'low_speed_straight' or packet['max_linear_mps'] != .014 or
                    type(packet['enabled']) is not bool or packet['physical_commissioned'] is not False):
                raise ValueError('domain')
            rotation = packet.get('rotation')
            trial = packet.get('rotation_trial', False)
            translation_trial = packet.get('translation_trial',False)
            if type(trial) is not bool or (trial and packet['enabled']):
                raise ValueError('rotation_trial')
            if (type(translation_trial) is not bool or
                    translation_trial and (packet['enabled'] or trial)):
                raise ValueError('translation_trial')
            if packet['enabled'] and rotation is not None:
                if (rotation['done'] is not True or rotation['error'] is not None or
                        rotation['max_angular_rad_s'] != .06 or len(rotation['legs']) != 8 or
                        len(rotation['angular_gains']) != 2 or
                        not all(math.isfinite(v) and .75 <= v <= 1.25 for v in rotation['angular_gains'])):
                    raise ValueError('rotation_domain')
                if 'envelope' in rotation and not validate_envelope(rotation['envelope']):
                    raise ValueError('rotation_envelope')
            if self.session and self.session != session:
                self.retired.add(self.session)
            self.session, self.sequence = session, sequence
            self.active = json.loads(json.dumps(packet, allow_nan=False))
            self.deadline = now + ttl - max(0., wall_now-issued)
            self.received = now
            self.reason = 'applied' if packet['enabled'] else 'revoked'
            if packet['enabled']:
                self.last_good = self.active.copy()
                if rotation and rotation.get('envelope'):
                    self._rotation_required = True
            return True
        except (KeyError, ValueError, TypeError, OverflowError):
            self.active = None
            self.deadline = 0.
            self.reason = 'invalid_profile'
            return False

    def live(self, now):
        return bool(self.active and self.active['enabled'] and
                    self.received is not None and math.isfinite(now) and self.received <= now <= self.deadline)

    def gains(self, now):
        return tuple(self.active['linear_gains']) if self.live(now) else (1., 1.)

    def rotation_trial_live(self, now):
        """Explicit short-lived trial authorization retains constraints, not gains."""
        return bool(self.active and not self.active['enabled'] and
                    self.active.get('rotation_trial') is True and
                    self.received is not None and math.isfinite(now) and self.received <= now <= self.deadline)

    def translation_trial_live(self, now):
        """Fresh disabled straight-trial lease; historical gains stay inactive."""
        return bool(self.active and not self.active['enabled'] and
                    self.active.get('translation_trial') is True and
                    self.received is not None and math.isfinite(now) and self.received <= now <= self.deadline)

    def angular_gains(self, now):
        if self.live(now) and self.active.get('rotation'):
            return tuple(self.active['rotation']['angular_gains'])
        return (1., 1.)

    def rotation_envelope(self, now, radius_floor=0., footprint=None):
        source = self.active if self.live(now) else None
        if (source is None and (self.rotation_trial_live(now) or self.translation_trial_live(now)) and self.last_good and
                self.last_good['geometry_revision'] == self.active['geometry_revision']):
            source = self.last_good
        if source is not None:
            report = (source.get('rotation') or {}).get('envelope')
            if validate_envelope(report, radius_floor):
                if footprint is not None:
                    try:
                        # Geometry comes from local configuration, never from
                        # a self-consistent but differently shaped certificate.
                        def normalized(points):
                            result = set()
                            for x,y in points:
                                if not math.isfinite(x) or not math.isfinite(y):
                                    raise ValueError('nonfinite footprint')
                                result.add((float(x),float(y)))
                            return result
                        if (not math.isclose(report['body_radius_m'],radius_floor,rel_tol=0.,abs_tol=1e-9) or
                                normalized(report['footprint_xy']) != normalized(footprint)):
                            return None
                    except (KeyError,TypeError,ValueError,OverflowError):
                        return None
                return report
        return None

    def rotation_estimate_required(self):
        """An expired learned envelope must not silently fall back to bootstrap."""
        return self._rotation_required

    def report(self, now):
        return {'schema_version': 1, 'applied': self.live(now),
                'revision': self.active['revision'] if self.active else None,
                'session': self.session, 'sequence': self.sequence,
                'reason': self.reason if not self.active or now <= self.deadline else 'expired',
                'last_good_revision': self.last_good['revision'] if self.last_good else None}
