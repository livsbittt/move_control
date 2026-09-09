import unittest
from rosy_control.control.calibration_profile import ProfileLease, make_profile, revision


class LimitedLeaseTest(unittest.TestCase):
    def packet(self, enabled=True):
        return make_profile('operator', 1, 100., enabled, [1., 1.], 'g', limited_sensors=True)

    def test_only_live_explicit_lease_excludes_imu_and_caps_commands(self):
        lease = ProfileLease()
        self.assertTrue(lease.accept(self.packet(), 100., 10., 'g'))
        self.assertEqual(lease.sensor_exclusions(10.), ('imu',))
        self.assertEqual(lease.motion_limits(10.), (.005, .05))
        self.assertEqual(lease.sensor_exclusions(12.), ())
        self.assertIsNone(lease.motion_limits(12.))
        self.assertTrue(lease.limited_sensor_hold(12.))
        self.assertFalse(lease.accept({}, 100.2, 12., 'g'))
        self.assertTrue(lease.limited_sensor_hold(12.))

    def test_disabled_or_ordinary_profile_never_excludes_imu(self):
        for packet in (self.packet(False), make_profile('ordinary', 1, 100., True, [1.,1.], 'g')):
            lease = ProfileLease()
            self.assertTrue(lease.accept(packet, 100., 10., 'g'))
            self.assertEqual(lease.sensor_exclusions(10.), ())

    def test_no_other_sensor_exclusion_or_relaxed_cap_is_accepted(self):
        for field, value in [('sensor_exclusions', ['lidar']), ('sensor_exclusions', ['imu','ir']),
                             ('max_linear_mps', .014), ('max_angular_rad_s', .1),
                             ('completion_source', 'verified'), ('rotation_trial', True)]:
            packet = self.packet()
            packet[field] = value
            packet['revision'] = revision(packet)
            self.assertFalse(ProfileLease().accept(packet, 100., 10., 'g'), field)

    def test_replay_and_geometry_mismatch_cannot_renew_permission(self):
        lease = ProfileLease()
        self.assertTrue(lease.accept(self.packet(), 100., 10., 'g'))
        self.assertFalse(lease.accept(self.packet(), 100.1, 11., 'g'))
        self.assertEqual(lease.sensor_exclusions(12.), ())
        self.assertFalse(ProfileLease().accept(self.packet(), 100., 10., 'other'))
