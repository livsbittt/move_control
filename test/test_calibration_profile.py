import unittest
from move_control.control.calibration_profile import ProfileLease, make_profile


class ProfileLeaseTest(unittest.TestCase):
    def packet(self, sequence=1, enabled=True, gains=(1.1, .9)):
        return make_profile('trial-a', sequence, 100., enabled, gains, 'geometry-a')

    def test_atomic_valid_profile_expires_and_duplicate_cannot_refresh(self):
        lease = ProfileLease()
        packet = self.packet()
        self.assertTrue(lease.accept(packet, 100., 10., 'geometry-a'))
        self.assertEqual(lease.gains(10.5), (1.1, .9))
        self.assertEqual(lease.gains(9.9), (1., 1.))
        self.assertFalse(lease.accept(packet, 100.5, 10.5, 'geometry-a'))
        self.assertEqual(lease.gains(11.6), (1., 1.))

    def test_invalid_revision_or_geometry_never_applies(self):
        lease = ProfileLease()
        packet = self.packet()
        packet['linear_gains'] = [1.25, 1.25]
        self.assertFalse(lease.accept(packet, 100., 10., 'geometry-a'))
        self.assertFalse(lease.accept(self.packet(), 100., 10., 'geometry-b'))
        self.assertEqual(lease.gains(10.), (1., 1.))

    def test_revocation_and_old_session_replay(self):
        lease = ProfileLease()
        self.assertTrue(lease.accept(self.packet(), 100., 10., 'geometry-a'))
        self.assertTrue(lease.accept(self.packet(2, False), 100., 10.1, 'geometry-a'))
        self.assertEqual(lease.gains(10.1), (1., 1.))
        new = make_profile('trial-b', 1, 100.2, False, (1., 1.), 'geometry-a')
        self.assertTrue(lease.accept(new, 100.2, 10.2, 'geometry-a'))
        self.assertFalse(lease.accept(self.packet(3), 100.2, 10.3, 'geometry-a'))

    def test_stale_latched_message_and_nonfinite_gain(self):
        lease = ProfileLease()
        self.assertFalse(lease.accept(self.packet(), 103., 10., 'geometry-a'))
        with self.assertRaises(ValueError):
            self.packet(gains=(float('nan'), 1.))
